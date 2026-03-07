/*
 * calibration.cpp
 * Mechanical homing / calibration using TMC2209 StallGuard sensorless detection.
 *
 * CALIBRATION OVERVIEW:
 * ──────────────────────
 * The board has no physical limit switches.  Instead, the TMC2209 StallGuard2
 * feature is used: when a motor stalls against a mechanical end-stop, the
 * SG_RESULT register drops sharply, and the code detects this as a "stall event".
 *
 * The stall detection threshold (stalltoUse / stalltoUseD2) is loaded from
 * NVS (Preferences) at the start of each calibration and updated at the end.
 * The stored value is the maximum reliable SG delta observed in the previous run,
 * plus a 10-unit safety margin.
 *
 * CALIBRATION MODES:
 * ───────────────────
 *   calibrationMode 0: Standard two-axis homing.
 *     - Drive motor 2 (Y axis) toward the −X, −Y corner until stall.
 *     - Optionally confirm position via the physical sensor at [0][9] (if
 *       "betterCalib" / enhancedCalib is enabled in preferences).
 *     - After homing, set step counters to correspond to the known corner
 *       position (−210mm, −198.5mm).
 *
 *   calibrationMode 1: Alternative two-step homing.
 *     Step 1: Drive +X to stall (finds the right wall).
 *             Optionally confirm with sensors at row 9.
 *     Step 2: Move to a known X reference, then drive −Y to stall
 *             (finds the bottom wall).
 *             Optionally confirm with sensor at [4][9].
 *     After homing, set step counters based on the stall counter (number of
 *     retries affects the assumed landed position by ±5mm correction factor).
 *
 * ENHANCED HOMING (enhancedCalib == 1):
 * ───────────────────────────────────────
 * When the preference "betterCalib" is 1, after each stall attempt the code
 * activates an electromagnet and checks whether the expected sensor reads
 * "occupied" (false).  If yes, the physical position is confirmed and homing
 * succeeds immediately.  If the sensor fails to trigger after 3 attempts, the
 * enhanced mode is disabled and simple tolerance-based homing is used instead.
 *
 * STALL THRESHOLD ADAPTATION:
 * ─────────────────────────────
 * After every successful calibration run, the highest reliable SG delta
 * measured during the move is saved to NVS.  The next run adds 10 to this
 * value to ensure detection even if motor conditions change slightly.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "mech_state.h"
#include "../sensors/sensor_state.h"
#include <CuteBuzzerSounds.h>

// ─────────────────────────────────────────────────────────────────────────────
// mechanicalCalibration()
// Performs sensorless homing for the XY motion system.
//
// calibrationMode:
//   0 → home both axes simultaneously toward the −X/−Y corner
//   1 → home X axis first, then Y axis independently (two-step)
// ─────────────────────────────────────────────────────────────────────────────
void mechanicalCalibration(int calibrationMode)
{
    Serial.println("HOMING STARTED " + String(calibrationMode));
    BleChess.setState("Homing");

    // Load previously stored stall thresholds from NVS
    preferences.begin("myApp", true);
    int stallOld = preferences.getInt("stallOld", 40);
    int stallOlder = preferences.getInt("stallOlder", 40);
    int stallOldD2 = preferences.getInt("stallOldD2", 40);
    int stallOlderD2 = preferences.getInt("stallOlderD2", 40);
    int enhancedCalib = preferences.getInt("betterCalib", 1);
    preferences.end();

    // Use the larger of the two stored values (more conservative), clamped to [1, 40], then add margin
    int stalltoUse = (stallOld > stallOlder) ? stallOld : stallOlder;
    stalltoUse = (stalltoUse <= 0 || stalltoUse > 40) ? 40 : stalltoUse;
    stalltoUse = stalltoUse + 10;
    int stalltoUseD2 = (stallOldD2 > stallOlderD2) ? stallOldD2 : stallOlderD2;
    stalltoUseD2 = (stalltoUseD2 <= 0 || stalltoUseD2 > 40) ? 40 : stalltoUseD2;
    stalltoUseD2 = stalltoUseD2 + 10;
    Serial.printf("Stall Old: %d Stall Older: %d Stall to use: %d StallD2 Old: %d StallD2 Older: %d StallD2 to Use: %d \n", stallOld, stallOlder, stalltoUse, stallOldD2, stallOlderD2, stalltoUseD2);

    float fastSpeed = 250000; // 250000
    float slowSpeed = fastSpeed / 10;
    driverMicroSteps = 32; // Ref 10000, curr 1000//128 -> 10,40 | 64 -> 22,50 | 32 -> 36, 66 | 16 -> 52,70 | 8 -> 74,94;70,80

    int activeMotors = -1;
    bool drv1 = true;
    bool drv2 = true;
    if (calibrationMode == 0)
    {
        Serial.println("===============================================================CALIBRATION MODE 0");
        activeMotors = 2;
    }
    else
    {
        Serial.println("===============================================================CALIBRATION MODE 1");
    }

    // Configure motors based on which axes are being homed
    if (activeMotors == 1)
    {
        drv1 = false;
        driver.rms_current(CALIB_CURRENT);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(0);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }
    else if (activeMotors == 2)
    {
        drv2 = false;
        driver.rms_current(0);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(CALIB_CURRENT);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }
    else
    {
        drv1 = false;
        drv2 = false;
        driver.rms_current(CALIB_CURRENT - 300);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(CALIB_CURRENT - 300);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }

    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);

    // Tolerance: 5mm in steps at current microstepping (used for tolerance-based homing)
    int toleranciaM = driverMicroSteps * 5 * 5; // equivalent to 800 steps, which is 5mm

    int stallCounter = 0;
    int maxValueStall = 0;
    int maxValueStallD2 = 0;
    if (calibrationMode == 0)
    {
        // ── Mode 0: Drive to −X/−Y corner via motor 2 ──────────────────────────
        do
        {
            readRawSensors(sensorMatrixSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(-500, -500, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;
            Serial.printf("current Position M1: %d current Position M2: %d toleranciaM: %d\n", stepper1.currentPosition(), stepper2.currentPosition(), toleranciaM);
            if (sensorMatrixSc[0][9] && enhancedCalib == 1)
            {
                // Enhanced homing: confirm position with the [0][9] sensor
                Serial.println("HOMING POR SENSOR 09");
                activateElectromagnetV2(4, 35);
                delay(300);

                int sensor44FirstStatus = sensorMatrixSc[4][4];
                readRawSensors(sensorMatrixSc);
                Serial.println("Sensor Status After Stall: " + String(sensorMatrixSc[0][9]));

                if (!sensorMatrixSc[0][9]) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR 09");
                    break;
                }
                else // Either the sensor is not working or too many attempts were made without reaching the position due to insufficient current.
                {
                    if (sensor44FirstStatus == 1 && sensorMatrixSc[4][4] == 0)
                    {
                        // Board is mechanically locked — alert the user
                        Serial.println("We are probably locked in, trying to unlock");
                        unsigned long timer = millis();
                        BleChess.sendTestModeError("MECHANISM LOCKED, REMOVE LOCK UNDER THE BOARD AND RESTART");
                        while (true)
                        {
                            if (millis() - timer > 5000)
                            {
                                BleChess.sendTestModeError("MECHANISM LOCKED, REMOVE LOCK UNDER THE BOARD AND RESTART");
                                timer = millis();
                            }
                            cute._tone(NOTE_G3, 50, 100);
                            delay(100);
                            cute._tone(NOTE_C0, 50, 100);
                        }
                    }
                    else
                    {
                        Serial.println("NO SE DETECTO EL SENSOR 09, REINTENTANDO");
                        deactivateAllMagnets();
                        if (stallCounter == 3) // TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION
                        {
                            BleChess.sendTestModeError("Enhanced Homing went wrong. Send SS of this to Phantom Team and Restart your board please. ");
                            enhancedCalib = 0;
                            preferences.begin("myApp", false);
                            preferences.putInt("betterCalib", enhancedCalib);
                            preferences.end();
                        }
                    }
                }
                deactivateAllMagnets();
            }
            else
            {
                // Tolerance-based homing fallback: accept if steps are within 5mm of zero
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                Serial.printf("PREPARANDO OTRO ROUND DE HOMING \n\n\n\n");
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                // Back off slightly before retrying
                rawMovementStallGuard(10, 10, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                Serial.printf("TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION \n\n\n\n");
                break;
            }

            Serial.println("=========================Stall counter: " + String(stallCounter));

        } while (true);

        // Persist the new threshold values for next boot
        preferences.begin("myApp", false);
        preferences.putInt("stallOld", maxValueStall);
        preferences.putInt("stallOlder", stallOld);
        preferences.putInt("stallOldD2", maxValueStallD2);
        preferences.putInt("stallOlderD2", stallOlder);
        preferences.end();
        Serial.printf("Max Value Stall: %d Max Value Stall D2: %d \n", maxValueStall, maxValueStallD2);

        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        // Set the step counters to match the known home corner position (−210, −198.5 mm)
        int coordXEnd = -210;
        int coordYEnd = -198.5;
        driverMicroSteps = 128;
        int stepsM1 = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
        int stepsM2 = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
        stepper1.setCurrentPosition(stepsM1);
        stepper2.setCurrentPosition(stepsM2);
    }
    else
    {
        // ── Mode 1: Two-step homing (X first, then Y) ───────────────────────────
        do
        {
            bool sensorsX9 = true;
            readRawSensors(sensorMatrixSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(500, 0, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;

            // Check whether any sensors in row 9 detect a piece (indicates arrival at X wall)
            for (int i = 1; i < 9; i++)
            {
                if (!sensorMatrixSc[9][i])
                {
                    Serial.println("sensor: X9 Y" + String(i) + " is on");
                    sensorsX9 = false;
                }
                else
                {
                    Serial.println("----------------------All sensors X9 turned off");
                }
            }

            if (sensorsX9 && enhancedCalib == 1)
            {
                // Enhanced homing: confirm X wall position with row-9 sensors
                Serial.println("HOMING POR SENSOR X9");
                activateElectromagnetV2(3, 35);
                delay(300);

                readRawSensors(sensorMatrixSc);

                for (int i = 1; i < 9; i++)
                {
                    if (!sensorMatrixSc[9][i])
                    {
                        Serial.println("sensor: X9 Y" + String(i) + " is on");
                        sensorsX9 = false;
                    }
                    else
                    {
                        Serial.println("----------------------All sensors X9 turned off");
                    }
                }

                if (!sensorsX9) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR X9");
                    break;
                }
                else // Sensor confirmation failed; fall back to tolerance-based
                {
                    Serial.println("NO SE DETECTO EL SENSOR X9, REINTENTANDO");
                    enhancedCalib = 0;
                    deactivateAllMagnets();
                }
                deactivateAllMagnets();
            }
            else
            {
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                rawMovementStallGuard(-10, 0, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                break;
            }

            Serial.println("=========================Stall counter: " + String(stallCounter));
        } while (true);

        // Re-enable enhanced calibration for the Y-axis step
        enhancedCalib = 1;
        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        // Move to a reference X position before homing Y
        float posX, posY;
        rawMovement(-208.5 - 25, 0, -2, posX, posY);

        // Reconfigure for Y-axis stall detection at lower current
        driverMicroSteps = 32;
        driver.rms_current(CALIB_CURRENT - 300);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop
        driver2.rms_current(CALIB_CURRENT - 300);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        stallCounter = 0;

        // ── Step 2 of mode 1: home Y axis ──────────────────────────────────────
        do
        {
            readRawSensors(sensorMatrixSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(0, -500, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;

            if (sensorMatrixSc[4][9] && enhancedCalib == 1)
            {
                // Enhanced homing: confirm Y wall position with sensor [4][9]
                Serial.println("HOMING POR SENSOR 49");
                activateElectromagnetV2(4, 35);
                delay(300);

                readRawSensors(sensorMatrixSc);
                Serial.println("Sensor Status After Stall: " + String(sensorMatrixSc[4][9]));

                if (!sensorMatrixSc[4][9]) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR 49");
                    break;
                }
                else // Sensor confirmation failed
                {
                    Serial.println("NO SE DETECTO EL SENSOR 49, REINTENTANDO");
                    deactivateAllMagnets();
                    if (stallCounter == 3) // TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION
                    {
                        BleChess.sendTestModeError("Enhanced Homing went wrong. Send SS of this to Phantom Team and Restart your board please. ");
                        enhancedCalib = 0;
                        preferences.begin("myApp", false);
                        preferences.putInt("betterCalib", enhancedCalib);
                        preferences.end();
                    }
                }
                deactivateAllMagnets();
            }
            else
            {
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                rawMovementStallGuard(0, 10, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                break;
            }

        } while (true);

        // Persist the new threshold values
        preferences.begin("myApp", false);
        preferences.putInt("stallOld", maxValueStall);
        preferences.putInt("stallOlder", stallOld);
        preferences.putInt("stallOldD2", maxValueStallD2);
        preferences.putInt("stallOlderD2", stallOlder);
        preferences.end();
        Serial.printf("Max Value Stall: %d Max Value Stall D2: %d \n", maxValueStall, maxValueStallD2);

        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        // Compute the landed position accounting for how many retries were needed
        int desfaseenX = -5;
        int desfaseenY = 3;
        int coordXEnd = -25 + (stallCounter * desfaseenX);
        int coordYEnd = -198.5 + desfaseenY;

        driverMicroSteps = 128;
        int stepsM1 = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
        int stepsM2 = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
        stepper1.setCurrentPosition(stepsM1);
        stepper2.setCurrentPosition(stepsM2);
    }

    // Restore conservative hold current after homing completes
    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
    BleChess.setState("");
    Serial.println("HOMING FINISHED");
}
