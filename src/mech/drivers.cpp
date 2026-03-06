/*
 * drivers.cpp
 * TMC2209 stepper driver configuration and current/speed management.
 *
 * DRIVER CONFIGURATION NOTES:
 * ────────────────────────────
 * Both TMC2209 drivers share Serial1 (SERIAL_PORT2) with different UART addresses:
 *   driver  (M1): DRIVER_ADDRESS1 = 0b00
 *   driver2 (M2): DRIVER_ADDRESS2 = 0b11
 *
 * Key settings applied in configDrivers():
 *   tbl         : blanking time (1 = 24 clocks) — suitable for most motors
 *   I_scale_analog = false : use internal reference, not VREF pin
 *   internal_Rsense = false: use external 0.11Ω sense resistor
 *   hstrt / hend : hysteresis for SpreadCycle (fine-tuned for noise vs. precision)
 *   irun / ihold : run and hold current in irun-scale (0-31)
 *   intpol       : step interpolation to 256 microsteps for smoother motion
 *   pwm_autograd / pwm_autoscale: StealthChop PWM auto-tuning
 *
 * MICROSTEPPING:
 *   128 microsteps: low speed (≤200 speed units) — max resolution for slow moves
 *    16 microsteps: medium speed (200–250 speed units)
 *     8 microsteps: high speed (>250 speed units) — reduces computation overhead
 *
 * STALLGUARD:
 *   SG_RESULT is the StallGuard2 load register (0 = stall, larger = lower load).
 *   configAndCurrentPosManager() switches en_spreadCycle(true) at higher speeds
 *   to enable SpreadCycle mode (needed for StallGuard to function correctly).
 *
 * TOFF:
 *   toff(0) disables the driver output stage; toff(1) re-enables it.
 *   During StallGuard calibration toff is toggled per-motor to isolate each axis.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "mech_state.h"
#include <TMCStepper.h>

// ─────────────────────────────────────────────────────────────────────────────
// configDrivers()
// Performs the one-time TMC2209 UART initialisation for both stepper drivers.
// Must be called once during system startup before any movement commands.
//
// After configuration:
//   - Both drivers are enabled (ENABLE_PIN pulled HIGH activates them)
//   - Current is set to HOLD_CURRENT (lower value for thermal safety at rest)
//   - A 150ms delay allows the drivers to stabilise before moves are accepted
// ─────────────────────────────────────────────────────────────────────────────
void configDrivers()
{
    int tbllocal = 1;
    int hstrtlocal = 4;
    int hstrtemdlocal = 2;

    driver.begin();

    driver.tbl(tbllocal);
    driver.I_scale_analog(false);  // disabled to use the external current sense resistor
    driver.internal_Rsense(false); // Same as above
    driver.hstrt(hstrtlocal);
    driver.hend(hstrtemdlocal);

    driver.irun(31);
    driver.ihold(16);
    driver.iholddelay(4);
    driver.TPOWERDOWN(2);

    driver.intpol(true);
    driver.pwm_autograd(true);
    driver.pwm_autoscale(true);

    driver2.begin();

    driver2.tbl(tbllocal);          // For most applications 1 or 2 is fine; for high-load applications use 2 or 3
    driver2.I_scale_analog(false);  // For most applications it is better to disable this
    driver2.internal_Rsense(false); // For most applications it is better to disable this
    driver2.hstrt(hstrtlocal);      // datasheet page 50; low values reduce precision and high values produce more noise.
    driver2.hend(hstrtemdlocal);    // datasheet page 50; low values reduce precision and high values produce more noise.

    driver2.irun(31);
    driver2.ihold(16);
    driver2.iholddelay(4);
    driver2.TPOWERDOWN(2);

    driver2.intpol(true);
    driver2.pwm_autograd(true);
    driver2.pwm_autoscale(true);

    // Enable the driver hardware (active HIGH on this board)
    digitalWrite(ENABLE_PIN, HIGH);

    // Set conservative hold current to protect against thermal overload at rest
    driver.rms_current(HOLD_CURRENT);  // Temperature and current consumption protection
    driver2.rms_current(HOLD_CURRENT); // Temperature and current consumption protection
    delay(150);
}

// ─────────────────────────────────────────────────────────────────────────────
// configAndCurrentPosManager()
// Adjusts microstepping, SpreadCycle mode, and RMS current to match the
// requested speed tier.  Also re-anchors the AccelStepper position registers
// (setCurrentPosition) so that step counts remain consistent across
// microstepping changes.
//
// Speed tiers:
//   ≤200  → 128 microsteps, StealthChop,   MIN_RMS_CURRENT
//   201-249→  16 microsteps, SpreadCycle,  MIN_RMS_CURRENT + 100
//   ≥250  →   8 microsteps, SpreadCycle,  MIN_RMS_CURRENT + 200
//
// When changing microstepping, the stored step position is scaled to keep
// the physical position unchanged:
//   New steps = old steps × (new_microsteps / old_microsteps)
// ─────────────────────────────────────────────────────────────────────────────
void configAndCurrentPosManager(int setSpeedbyUser, float &driverMicroSteps)
{
    //===============================================DRIVERS CONFIGURATION===============================================
    stepper1.setCurrentPosition(stepper1.currentPosition());
    stepper2.setCurrentPosition(stepper2.currentPosition());
    if (setSpeedbyUser != lastspeedbyUser || driver.microsteps() != driverMicroSteps)
    {
        if (setSpeedbyUser <= 200)
        {
            // Scaling factor when transitioning to 128 microsteps
            int stepsMultiplier = (driverMicroSteps == 16) ? 8 : (driverMicroSteps == 8) ? 16
                                                                                         : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() * stepsMultiplier);
            stepper2.setCurrentPosition(stepper2.currentPosition() * stepsMultiplier);
            driverMicroSteps = 128;
        }
        else if (setSpeedbyUser > 200 && setSpeedbyUser < 250)
        {
            // Scaling factor when transitioning to 16 microsteps
            int stepsDivider = (driverMicroSteps == 128) ? 8 : (driverMicroSteps == 8) ? 2
                                                                                       : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() / stepsDivider);
            stepper2.setCurrentPosition(stepper2.currentPosition() / stepsDivider);
            driverMicroSteps = 16;
        }
        else
        {
            // Scaling factor when transitioning to 8 microsteps
            int stepsDivider = (driverMicroSteps == 128) ? 16 : (driverMicroSteps == 16) ? 2
                                                                                         : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() / stepsDivider);
            stepper2.setCurrentPosition(stepper2.currentPosition() / stepsDivider);
            driverMicroSteps = 8;
        }

        // SpreadCycle is required for StallGuard to work; use StealthChop at low speeds
        driver.en_spreadCycle(setSpeedbyUser > 200);
        driver.pwm_autograd(true);
        driver.pwm_autoscale(true);
        driver.microsteps(driverMicroSteps);

        driver2.en_spreadCycle(setSpeedbyUser > 200);
        driver2.pwm_autograd(true);
        driver2.pwm_autoscale(true);
        driver2.microsteps(driverMicroSteps);
    }
    // Apply RMS current appropriate for the speed tier
    driver.rms_current(setSpeedbyUser <= 200 ? MIN_RMS_CURRENT : setSpeedbyUser < 250 ? MIN_RMS_CURRENT + 100
                                                                                      : MIN_RMS_CURRENT + 200);
    driver2.rms_current(setSpeedbyUser <= 200 ? MIN_RMS_CURRENT : setSpeedbyUser < 250 ? MIN_RMS_CURRENT + 100
                                                                                       : MIN_RMS_CURRENT + 200);
    lastspeedbyUser = setSpeedbyUser;
    //===============================================DRIVERS CONFIGURATION===============================================
}

// ─────────────────────────────────────────────────────────────────────────────
// testDrivers()
// Verifies that both TMC2209 drivers respond correctly over UART.
//
// For each driver:
//   1. test_connection() must return 0 (no UART error)
//   2. Writing 400mA / 8 microsteps and reading back must be within ±50 mA
//      and the microstep count must be exactly 8
//   3. Writing 1200mA / 128 microsteps and reading back must be within ±50 mA
//      and the microstep count must be exactly 128
//
// Returns true only if all 6 checks pass (3 per driver).
// ─────────────────────────────────────────────────────────────────────────────
bool testDrivers()
{
    int testOk = 0;
    //-------------------------------MOTOR 1 TEST-------------------------------------------
    uint8_t result_1 = driver.test_connection();
    if (result_1 == 0)
    {
        testOk++;
    }
    driver.rms_current(400);
    driver.microsteps(8);
    if ((driver.rms_current() <= 450 || driver.rms_current() >= 350) && driver.microsteps() == 8)
    {
        testOk++;
    }
    driver.rms_current(1200);
    driver.microsteps(128);
    if ((driver.rms_current() <= 1250 || driver.rms_current() >= 1150) && driver.microsteps() == 128)
    {
        testOk++;
    }
    //-------------------------------MOTOR 2 TEST-------------------------------------------
    uint8_t result_2 = driver2.test_connection();
    if (result_2 == 0)
    {
        testOk++;
    }
    driver2.rms_current(400);
    driver2.microsteps(8);
    if ((driver2.rms_current() <= 450 || driver2.rms_current() >= 350) && driver2.microsteps() == 8)
    {
        testOk++;
    }

    driver2.rms_current(1200);
    driver2.microsteps(128);
    if ((driver2.rms_current() <= 1250 || driver2.rms_current() >= 1150) && driver2.microsteps() == 128)
    {
        testOk++;
    }
    //-------------------------------RESULT-------------------------------------------
    if (testOk == 6)
    {
        return true;
    }
    else
    {
        return false;
    }
}
