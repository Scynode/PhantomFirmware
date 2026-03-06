/**
 * @file ble_battery.cpp
 * @brief Battery simulation and hardware-level battery checks.
 *
 * The Phantom sculpture uses a single Li-ion cell.  Direct ADC voltage
 * measurement is only done at startup (or on power-state transitions) because
 * continuous ADC reads interfere with the stepper motors.  Between those
 * snapshots the firmware estimates the remaining charge with a simple linear
 * simulation (Bluetooth::batterySim) that tracks how long the motors have been
 * moving vs. idle.
 *
 * ─── Battery simulation algorithm ──────────────────────────────────────────
 *
 *  State variables:
 *    batteryStatus_  – current estimated raw ADC level (updated every call)
 *    batteryInit     – ADC snapshot taken at the last power-state transition
 *    maxBattery      – calibrated full-charge ADC reading (persisted in NVS)
 *    initialTime     – millis() at the last "disconnected from wall" event
 *    initialTimeCharging – millis() at the last "connected to wall" event
 *    timeMoving      – cumulative milliseconds the motors ran since initialTime
 *                      (tracked externally and declared extern in config.h)
 *
 *  Discharge model (not on wall power):
 *    batteryStatus_ = batteryInit
 *                     - timeOnHold   × dischargeRateOnHold   (0.00001 / ms)
 *                     - timeMoving   × dischargeRateMoving   (0.00002 / ms)
 *
 *  Charge model (on wall power, still charging):
 *    batteryStatus_ = batteryInit + timeCharging × chargeRate  (0.00005 / ms)
 *
 *  Full-charge model (on wall power, charging complete):
 *    batteryStatus_ = maxBattery
 *
 *  Output string (batteryStatustoApp):
 *    "percent,wallStatus,charging,doneCharging"
 *    percent is mapped from [2500, maxBattery] → [0, 100] via Arduino map().
 *
 * ─── batteryCheck modes ────────────────────────────────────────────────────
 *
 *  The battery mux uses three GPIO select lines (batS0, batS1, batS2 defined
 *  in config.h) to route one of four battery-management signals to GPIO 4:
 *
 *  mode 0 – Raw VBAT ADC average (250 ms sampling window, returned as int).
 *             Returns -1 if the battery appears disconnected (erratic ADC).
 *  mode 1 – Is the charger actively charging?  1 = yes, 0 = no.
 *             Reads the CHG# pin (active-low → inverted before returning).
 *  mode 2 – Has charging completed?  1 = yes (done), 0 = still charging.
 *             Reads the DONE# pin (active-low → inverted before returning).
 *  mode 3 – Is wall power connected?  1 = yes, 0 = running on battery.
 *             Reads the ACOK pin (active-high, returned directly).
 */

#include "BLE.h"
#include <config.h>
#include <Arduino.h>
#include <Preferences.h>
#include "ble_state.h"

// ---------------------------------------------------------------------------
// Bluetooth::batterySim – run every loop iteration to update batteryStatus_
// ---------------------------------------------------------------------------
void Bluetooth::batterySim()
{
    // Discharge / charge rates expressed in ADC-units per millisecond
    const float dischargeRateOnHold = 0.00001f; // Idle power draw
    const float dischargeRateMoving = 0.00002f; // Additional draw while motors run
    const float chargeRate          = 0.00005f; // Charge rate from wall adapter

    int charging     = Bluetooth::batteryCheck(1); // Is the charger IC actively charging?
    int doneCharging = Bluetooth::batteryCheck(2); // Has the charge cycle finished?
    int wallStatus   = Bluetooth::batteryCheck(3); // Is AC wall power present?

    // ── Detect power-state transitions and reset reference points ──────────

    if (previousWallStatus && !wallStatus)
    {
        // Wall power was just DISCONNECTED: capture the current voltage as the
        // new discharge baseline and reset the moving-time accumulator.
        initialTime        = millis();
        timeMoving         = 0;
        previousWallStatus = wallStatus;
        batteryInit        = batteryStatus_;
    }
    else if (!previousWallStatus && wallStatus)
    {
        // Wall power was just CONNECTED: capture current voltage as charge baseline.
        initialTimeCharging = millis();
        previousWallStatus  = wallStatus;
        batteryInit         = batteryStatus_;
    }

    // ── Compute elapsed times ──────────────────────────────────────────────

    unsigned long totalTimeOn = millis() - initialTime;

    // Safety: if timeMoving somehow accumulated past totalTimeOn (e.g. power
    // was disconnected mid-move) discard it to avoid negative idle time.
    if (timeMoving > totalTimeOn)
    {
        timeMoving = 0;
    }

    unsigned long timeOnHold   = totalTimeOn - timeMoving;
    unsigned long timeCharging = millis() - initialTimeCharging;

    // ── Apply the appropriate model ────────────────────────────────────────

    if (wallStatus)
    {
        if (charging)
        {
            // Cell is actively charging: linear ramp up from batteryInit
            batteryStatus_ = batteryInit + (float)timeCharging * chargeRate;
        }
        if (doneCharging)
        {
            // Charge cycle complete: clamp to the calibrated maximum
            batteryStatus_ = maxBattery;
            batteryInit    = maxBattery;
        }
    }
    else
    {
        // Running on battery: subtract idle and moving discharge contributions
        batteryStatus_ = batteryInit
                         - (float)timeOnHold   * dischargeRateOnHold
                         - (float)timeMoving   * dischargeRateMoving;
    }

    // ── Build the status string for the app ───────────────────────────────
    // Map the raw ADC level to a 0–100% percentage using the calibrated range.
    int batteryPercent = map((long)batteryStatus_, 2500L, (long)maxBattery, 0L, 100L);

    batteryStatustoApp = String(batteryPercent) + "," +
                         String(wallStatus)      + "," +
                         String(charging)        + "," +
                         String(doneCharging);
}

// ---------------------------------------------------------------------------
// Bluetooth::getBatteryStatus – return the current estimated battery level
// ---------------------------------------------------------------------------

/**
 * Returns the current estimated battery level as a raw ADC value.
 * Updated every main loop iteration by Bluetooth::batterySim().
 * Returns -1 if no battery is detected.
 */
float Bluetooth::getBatteryStatus()
{
    return batteryStatus_;
}

// ---------------------------------------------------------------------------
// Bluetooth::batteryCheck – low-level mux + ADC battery measurement
// ---------------------------------------------------------------------------
int Bluetooth::batteryCheck(int mode)
{
    int analogValueVBAT    = 0;
    int digitalcharge      = 0;
    const int muxDelay     = 5; // ms to settle after switching the mux

    if (mode == 0) // Read raw VBAT ADC level
    {
        // First, check for a disconnected battery by looking for large, rapid
        // ADC swings on the battery-present detection channel.
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, HIGH);
        delay(muxDelay);

        unsigned long timer          = millis();
        int previousDigitalCharge    = analogRead(4);
        int cambios                  = 0;

        while (millis() - timer < 250)
        {
            digitalcharge = analogRead(4);
            // Swings larger than 2000 ADC counts indicate no battery / floating input
            if (abs(digitalcharge - previousDigitalCharge) > 2000)
            {
                cambios++;
            }
            previousDigitalCharge = digitalcharge;
        }

        if (cambios > 10)
        {
            // Too many large swings → battery is likely disconnected
            Serial.println("=================================Battery Disconnected");
            return -1;
        }
        else
        {
            // Battery present: switch to the VBAT channel and average over 250 ms
            digitalWrite(batS2, LOW);
            digitalWrite(batS1, HIGH);
            digitalWrite(batS0, HIGH);
            delay(muxDelay);

            timer         = millis();
            int counter   = 0;
            while (millis() - timer < 250)
            {
                analogValueVBAT += analogRead(4);
                counter++;
            }

            analogValueVBAT /= counter;
            Serial.printf("=================================RAW VBAT: %d \n", analogValueVBAT);
            return analogValueVBAT;
        }
    }
    else if (mode == 1) // Is the charger IC actively charging? (CHG# pin, active-low)
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, HIGH);
        delayMicroseconds(1);
        return !digitalRead(4); // Invert: LOW = charging
    }
    else if (mode == 2) // Has charging completed? (DONE# pin, active-low)
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, HIGH);
        digitalWrite(batS0, LOW);
        delayMicroseconds(1);
        return !digitalRead(4); // Invert: LOW = done
    }
    else if (mode == 3) // Is wall / AC power connected? (ACOK pin, active-high)
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, LOW);
        delayMicroseconds(1);
        return digitalRead(4); // HIGH = wall power present
    }

    return -1; // Unknown mode
}
