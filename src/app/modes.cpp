/**
 * @file modes.cpp
 * @brief Simple operating-mode handlers: error halt, test, pause, and sculpture.
 *
 * Each function in this file corresponds to one firmware operating mode and is
 * called directly from the loop() dispatcher in setup.cpp.
 *
 * ─── errorMessage ──────────────────────────────────────────────────────────
 *  A fatal error sink that broadcasts the error string over BLE and Serial
 *  every 5 seconds, beeps continuously, and never returns.  The board must be
 *  physically restarted to recover.
 *
 * ─── testMode (mode 4) ─────────────────────────────────────────────────────
 *  Full hardware self-test sequence:
 *    1. Driver connectivity test (TMC2209 UART ping).
 *    2. Mechanical calibration (homing).
 *    3. Playable-area sweep (carriage moves to all four corners).
 *    4. Electromagnet visual check (activates each magnet individually).
 *    5. Battery connection check.
 *    6. Sensor stability with no pieces (all should read HIGH / empty).
 *    7. Electromagnet-sensor cross-check (magnet on → sensor at that position
 *       must trigger; all others must remain idle).
 *    8. Full snake-pattern sensor scan (moves carriage to every non-corner
 *       position; verifies exactly one sensor reads LOW at a time).
 *    9. Cable-length test (measures repeatable positioning accuracy).
 *   10. If test was board-triggered (testFlag == -1): run a 10-minute
 *       sculptureMode endurance loop.
 *       If test was app-triggered: report PASS and loop forever.
 *
 * ─── pauseMode (mode 3) ────────────────────────────────────────────────────
 *  Blocks in a tight polling loop until the BLE mode changes away from 3.
 *
 * ─── sculptureMode (mode 1) ────────────────────────────────────────────────
 *  Calls the sculptureMain() state machine (defined in src/chess/pgn.cpp) then
 *  waits until the BLE mode changes away from 1 before returning to loop().
 */

#include <config.h>
#include "BLE.h"
#include "scultpureMode.h"
#include <CuteBuzzerSounds.h>
#include <Arduino.h>
#include "app_globals.h"

// =============================================================================
// errorMessage – fatal error halt
// =============================================================================

/**
 * @brief Deactivate all magnets, print @p error, and loop forever broadcasting
 *        the error over BLE and beeping the buzzer.
 *
 * Used for hardware failures where continuing would risk physical damage or
 * produce meaningless results.  The board must be power-cycled to recover.
 *
 * @param error  Human-readable description of the fault (displayed in the app
 *               and printed to Serial).
 */
void errorMessage(String error)
{
    deactivateAllMagnets(); /* Safety: turn off electromagnets immediately */
    int timer = millis();
    Serial.println(error);
    BleChess.sendTestModeError(error);
    while (true)
    {
        cute._tone(NOTE_G3, 50, 100); /* Continuous low-frequency alert tone */
        if (millis() - timer > 5000)
        {
            /* Re-broadcast every 5 seconds so the app can always catch it. */
            Serial.println(error);
            BleChess.sendTestModeError(error);
            timer = millis();
        }
    }
}

// =============================================================================
// testMode – hardware self-test (mode 4)
// =============================================================================

/**
 * @brief Run the complete factory / field hardware self-test sequence.
 *
 * Each test stage updates the BLE state string so the app can show progress.
 * Any failure calls errorMessage() which halts the firmware immediately.
 * A successful test ends in an infinite pass-notification loop (or triggers
 * a 10-minute sculptureMode endurance run if testFlag == -1).
 */
void testMode()
{
    float posX = 0;
    float posY = 0;
    bool flagElectromagneting = true;
    int allOnes = 0;
    int sensorsOn = 0;
    String error = "";
    int delayElectromagnet = 300; /* ms to settle after switching electromagnets */
    unsigned long timeElectromagneting = 0;

    Serial.println("================================Test Mode================================");
    BleChess.setMode(4);

    // ── 1. Driver connectivity test ──────────────────────────────────────────
    Serial.println("────────────────────────────────────────────────────");
    Serial.print("► Testing Drivers ");
    BleChess.setState("► Testing Drivers ");
    if (testDrivers())
    {
        Serial.println("✓ Drivers are working properly.");
    }
    else
    {
        errorMessage("✕ Drivers Failed");
    }

    // ── 2. Mechanical calibration (homing) ───────────────────────────────────
    mechanicalCalibration(BleChess.getCalibType());

    // ── 3. Playable-area sweep ────────────────────────────────────────────────
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Area Check ");
    BleChess.setState("► Area Check ");
    Serial.println(
        "If you see the end effector moving, and not cracking noises "
        "or any other issue, mechanism is ok");
    /* Visit all four corners and back to verify the full range of motion. */
    rawMovement(-180, 180, -3, posX, posY);
    rawMovement(180, 180, -3, posX, posY);
    rawMovement(180, -180, -3, posX, posY);
    rawMovement(-180, -180, -3, posX, posY);
    rawMovement(180, 180, -3, posX, posY);
    rawMovement(-180, 180, -3, posX, posY);
    rawMovement(180, -180, -3, posX, posY);
    Serial.println("► Area Check Completed ");

    // ── 4. Electromagnet visual check ─────────────────────────────────────────
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Electromagnets Check ");
    BleChess.setState("► Electromagnets Check ");
    Serial.println(
        "Using a piece of paper, check that the electromagnets are working properly");
    /* Activate each of the four electromagnets (plus index 0 = all off) in turn
     * for one second so the operator can verify attraction visually. */
    for (int electromagnet = 0; electromagnet <= 4; electromagnet++)
    {
        activateElectromagnetV2(electromagnet, 35);
        delay(1000);
        deactivateAllMagnets();
    }
    BleChess.setState("► Electromagnets Check Completed ");

    // ── 5. Battery connection check ───────────────────────────────────────────
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Battery Check ");
    BleChess.setState("► Battery Check ");
    int batteryStatus = BleChess.getBatteryStatus();
    if (batteryStatus == -1 && testFlag == -1)
    {
        /* Battery disconnected in board-triggered test; warn but continue. */
        Serial.println("✕ Battery is NOT connected");
        BleChess.sendTestModeError("✕ Battery is NOT connected");
        soundHandler(8);
    }

    // ── 6. Sensor stability with no pieces ────────────────────────────────────
    /* Wait up to 30 seconds for all 100 sensors to read HIGH (empty).
     * This catches stuck sensors or pieces accidentally left on the board. */
    long startTime = millis();
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Testing Sensors Stabilty when off ");
    BleChess.setState("► Sensors Stabilty when off ");
    do
    {
        sensorsOn = 0;
        readRawSensors(sensorMatrix);
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (sensorMatrix[i][j] == 0)
                {
                    sensorsOn++;
                }
            }
        }
    } while (sensorsOn != 0 && millis() - startTime < 30000);

    if (sensorsOn != 0)
    {
        /* Build a list of the stuck sensor coordinates for the error message. */
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (sensorMatrix[i][j] == 0)
                {
                    error = error + "," + i + j;
                }
            }
        }
        errorMessage("✕ Sensors that failed: " + error);
    }
    else
    {
        Serial.println("✓ Stabilty of sensors when off passed.");
    }
    error = "";

    // ── 7. Electromagnet-sensor cross-check ───────────────────────────────────
    /* For each electromagnet, move the carriage to a test position, activate
     * the magnet, and verify the expected sensor reads LOW while all others
     * remain HIGH.  postocheck advances diagonally (3,3 → 4,4 → 5,5 → 6,6)
     * to stay within the quadrant each electromagnet covers. */
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Electromagnet Check ");
    BleChess.setState("► Electromagnet Check ");

    int postocheck = 3;
    for (int electromagnet = 1; electromagnet <= 4; electromagnet++)
    {
        Serial.println("Checking in sensor: " + String(postocheck) + "," + String(postocheck));
        rawMovement(postocheck, postocheck, electromagnet, posX, posY);
        activateElectromagnetV2(electromagnet, 35);
        unsigned long timeElectromagneting = millis();

        /* Wait for the target sensor to trigger (up to 5 s). */
        do
        {
            readRawSensors(sensorMatrix);
            if (millis() - timeElectromagneting > 5000)
            {
                errorMessage("✕ electromagnet " + String(electromagnet) + " not turn on");
            }
        } while (sensorMatrix[postocheck][postocheck] == true);
        Serial.println("✓ electromagnet " + String(electromagnet) + " On");

        /* Verify no other electromagnet positions accidentally trigger the sensor. */
        for (int i = 1; i <= 4; i++)
        {
            if (i != electromagnet)
            {
                rawMovement(postocheck, postocheck, i, posX, posY);
                readRawSensors(sensorMatrix);
                if (sensorMatrix[postocheck][postocheck] == false)
                {
                    errorMessage("✕ electromagnet " + String(i) +
                                 " is on when main electromagnet " +
                                 String(electromagnet) + " is on");
                }
                else
                {
                    Serial.println("✓ electromagnet " + String(i) + " Off");
                }
            }
        }
        Serial.println("");
        deactivateAllMagnets();
        postocheck++;
    }

    // ── 8. Full snake-pattern sensor scan ─────────────────────────────────────
    /* Traverse every non-corner position in a boustrophedon (snake) pattern:
     * even columns iterate i=0→9, odd columns iterate i=9→0.
     * For each position:
     *   a) Move carriage there with the appropriate electromagnet.
     *   b) Measure sensor bounce rate over 50 ms (must be < 0.1% of reads).
     *   c) Verify the specific sensor reads LOW.
     *   d) Verify all other sensors read HIGH. */
    Serial.println("────────────────────────────────────────────────────");
    Serial.print("► Testing All the Sensors Health: ");
    BleChess.setState("► Testing All the Sensors Health: ");
    int checadas = 0;
    int sensorsOn2 = 0;
    int timesBouncing = 0;
    bool sensorMatrixAux[10][10] = {0};
    int electromagnet = 1;
    int previousElectromagnet = 0;

    for (int j = 0; j <= 9; j++)
    {
        if (j % 2 == 0) /* Even column: sweep left to right (i = 0 → 9) */
        {
            for (int i = 0; i <= 9; i++)
            {
                /* Skip the four corners – they are not reachable by the mechanism. */
                if ((i == 0 && j == 0) || (i == 9 && j == 0) ||
                    (i == 0 && j == 9) || (i == 9 && j == 9))
                {
                    continue;
                }

                Serial.printf("\n ───────────── Sensor X: %d Y: %d\n", i, j);

                /* Select the electromagnet whose quadrant contains this position.
                 * Border rows/columns use dedicated graveyard electromagnets. */
                electromagnet = (j == 0) ? 2 : (j == 9) ? 4
                                           : (i == 0)   ? 1
                                           : (i == 9)   ? 3
                                                        : 1;

                if (electromagnet != previousElectromagnet)
                {
                    deactivateAllMagnets();
                    activateElectromagnetV2(electromagnet, 35);
                    delay(delayElectromagnet);
                    previousElectromagnet = electromagnet;
                }

                rawMovement(i, j, electromagnet, posX, posY);

                /* Measure bounce rate: sample for 50 ms, count transitions. */
                readRawSensors(sensorMatrix);
                for (int i = 0; i < 10; i++)
                    for (int j = 0; j < 10; j++)
                        sensorMatrixAux[i][j] = sensorMatrix[i][j];

                long timeBouncing = millis();
                while (millis() - timeBouncing < 50)
                {
                    readRawSensors(sensorMatrix);
                    for (int i = 0; i < 10; i++)
                    {
                        for (int j = 0; j < 10; j++)
                        {
                            if (sensorMatrix[i][j] != sensorMatrixAux[i][j])
                            {
                                sensorMatrixAux[i][j] = sensorMatrix[i][j];
                                timesBouncing++;
                            }
                        }
                    }
                    checadas++;
                }

                Serial.println("✓ Bouncing: " + String(timesBouncing) +
                               " times out of: " + String(checadas));
                /* Reject if > 0.1% of samples were unstable. */
                if (timesBouncing > checadas * 0.001)
                {
                    errorMessage("✕ Sensor bouncing: " + String(i) + String(j));
                }
                checadas = 0;
                timesBouncing = 0;

                /* Specific sensor must now read LOW (piece detected). */
                if (sensorMatrix[i][j] == false)
                {
                    Serial.println("✓ Sensor " + String(i) + String(j) + " is on");
                }
                else
                {
                    errorMessage("✕ Sensor " + String(i) + String(j) + " is off");
                }

                /* All OTHER sensors must read HIGH. */
                for (int k = 0; k <= 9; k++)
                {
                    for (int l = 0; l <= 9; l++)
                    {
                        if (sensorMatrix[l][k] == false && (k != j || l != i))
                        {
                            sensorsOn2++;
                            error = error + ", " + l + k;
                        }
                    }
                }

                if (sensorsOn2 == 0)
                {
                    Serial.println("✓ All the other Sensors are off");
                }
                else
                {
                    Serial.println("✕ More Sensors where found on when sensor " +
                                   String(i) + String(j) + " is on");
                    BleChess.sendTestModeError(
                        "✕ More Sensors where found on when sensor " +
                        String(i) + String(j) + " is on: " + error);
                    /* Print the full sensor matrix to Serial for debugging. */
                    for (int k = 0; k <= 9; k++)
                    {
                        for (int l = 0; l <= 9; l++)
                        {
                            Serial.print(sensorMatrix[l][k]);
                            Serial.print(" ");
                        }
                        Serial.println("");
                    }
                    int timer = millis();
                    while (true)
                    {
                        deactivateAllMagnets();
                        cute._tone(NOTE_G3, 50, 100);
                        if (millis() - timer > 5000)
                        {
                            Serial.println("✕ More Sensors where found on when sensor " +
                                           String(i) + String(j) + " is on");
                            BleChess.sendTestModeError(
                                "✕ More Sensors where found on when sensor " +
                                String(i) + String(j) + " is on: " + error);
                            for (int k = 0; k <= 9; k++)
                            {
                                for (int l = 0; l <= 9; l++)
                                {
                                    Serial.print(sensorMatrix[l][k]);
                                    Serial.print(" ");
                                }
                                Serial.println("");
                            }
                            timer = millis();
                        }
                    }
                }
                error = "";
                sensorsOn2 = 0;
            }
        }
        else /* Odd column: sweep right to left (i = 9 → 0) */
        {
            for (int i = 9; i >= 0; i--)
            {
                /* Skip corners. */
                if ((i == 0 && j == 0) || (i == 9 && j == 0) ||
                    (i == 0 && j == 9) || (i == 9 && j == 9))
                {
                    continue;
                }

                Serial.printf("\n ───────────── Sensor X: %d Y: %d\n", i, j);

                electromagnet = (j == 0) ? 2 : (j == 9) ? 4
                                           : (i == 0)   ? 1
                                           : (i == 9)   ? 3
                                                        : 1;

                if (electromagnet != previousElectromagnet)
                {
                    deactivateAllMagnets();
                    activateElectromagnetV2(electromagnet, 35);
                    delay(delayElectromagnet);
                    previousElectromagnet = electromagnet;
                }

                rawMovement(i, j, electromagnet, posX, posY);

                /* Bounce measurement. */
                readRawSensors(sensorMatrix);
                for (int i = 0; i < 10; i++)
                    for (int j = 0; j < 10; j++)
                        sensorMatrixAux[i][j] = sensorMatrix[i][j];

                long timeBouncing = millis();
                while (millis() - timeBouncing < 50)
                {
                    readRawSensors(sensorMatrix);
                    for (int i = 0; i < 10; i++)
                    {
                        for (int j = 0; j < 10; j++)
                        {
                            if (sensorMatrix[i][j] != sensorMatrixAux[i][j])
                            {
                                sensorMatrixAux[i][j] = sensorMatrix[i][j];
                                timesBouncing++;
                            }
                        }
                    }
                    checadas++;
                }

                Serial.println("✓ Bouncing: " + String(timesBouncing) +
                               " times out of: " + String(checadas));
                if (timesBouncing > checadas * 0.001)
                {
                    errorMessage("✕ Sensor bouncing: " + String(i) + String(j));
                }
                checadas = 0;
                timesBouncing = 0;

                if (sensorMatrix[i][j] == false)
                {
                    Serial.println("✓ Sensor " + String(i) + String(j) + " is on");
                }
                else
                {
                    errorMessage("✕ Sensor " + String(i) + String(j) + " is off");
                }

                for (int k = 0; k <= 9; k++)
                {
                    for (int l = 0; l <= 9; l++)
                    {
                        if (sensorMatrix[l][k] == false && (k != j || l != i))
                        {
                            sensorsOn2++;
                            error = error + ", " + l + k;
                        }
                    }
                }

                if (sensorsOn2 == 0)
                {
                    Serial.println("✓ All the other Sensors are off");
                }
                else
                {
                    Serial.println("✕ More Sensors where found on when sensor " +
                                   String(i) + String(j) + " is on");
                    BleChess.sendTestModeError(
                        "✕ More Sensors where found on when sensor " +
                        String(i) + String(j) + " is on: " + error);
                    for (int k = 0; k <= 9; k++)
                    {
                        for (int l = 0; l <= 9; l++)
                        {
                            Serial.print(sensorMatrix[l][k]);
                            Serial.print(" ");
                        }
                        Serial.println("");
                    }
                    int timer = millis();
                    while (true)
                    {
                        deactivateAllMagnets();
                        cute._tone(NOTE_G3, 50, 100);
                        if (millis() - timer > 5000)
                        {
                            Serial.println("✕ More Sensors where found on when sensor " +
                                           String(i) + String(j) + " is on");
                            BleChess.sendTestModeError(
                                "✕ More Sensors where found on when sensor " +
                                String(i) + String(j) + " is on: " + error);
                            for (int k = 0; k <= 9; k++)
                            {
                                for (int l = 0; l <= 9; l++)
                                {
                                    Serial.print(sensorMatrix[l][k]);
                                    Serial.print(" ");
                                }
                                Serial.println("");
                            }
                            timer = millis();
                        }
                    }
                }
                error = "";
                sensorsOn2 = 0;
            }
        }
    }

    // ── 9. Cable-length / repeatability test ──────────────────────────────────
    /* Move to a reference position, record the calibrated coordinates, then
     * move away and back and measure the positional error.  If Y error > 1.5 mm
     * the cable tension is likely misconfigured. */
    Serial.println("────────────────────────────────────────────────────");
    Serial.print("► Testing Cable Lenght: ");
    BleChess.setState("► Testing Cable Lenght: ");
    float posX1 = 0;
    float posY1 = 0;
    sensorsCalibration(5, 5, 1, posX1, posY1);
    Serial.printf("X1: %f Y1: %f\n", posX1, posY1);
    Serial.println("Tryng to loose steps: ");
    rawMovement(0, 198, -2, posX, posY); /* Intentional long traverse to stress belts */
    sensorsCalibration(5, 4, 0, posX, posY);
    Serial.printf("X2: %f Y2: %f\n", posX, posY);

    float cableMissingX = posX - posX1;
    float cableMissingY = posY - posY1;
    Serial.printf("Error X: %f Y: %f\n", cableMissingX, cableMissingY);

    if ((posY - posY1 <= -1.5 || posY - posY1 >= 1.5))
    {
        errorMessage("✕ Cable Lenght Failed" + String(cableMissingX) +
                     " " + String(cableMissingY));
    }
    else
    {
        Serial.println("✓ Cable Lenght is correct.");
    }

    // ── 10. Post-test dispatch ────────────────────────────────────────────────
    if (testFlag == -1)
    {
        /* Board-triggered test: enter a 10-minute sculptureMode endurance run
         * so mechanical robustness can be verified under continuous operation. */
        Serial.println("────────────────────────────────────────────────────");
        Serial.print("► Starting Full Sculpture Mode 10 minutes Routine: ");
        BleChess.setState("► Testing Cable Lenght: ");
        testModeSculpture = true;
        BleChess.setMode(1);
        testFlag = -2;
        return; /* Return to loop() which will dispatch to sculptureMode(). */
    }
    else
    {
        /* App-triggered test: report result and stay in notification loop. */
        if (BleChess.getBatteryStatus() == -1)
        {
            while (true)
            {
                Serial.println("✓ FULL TEST PASSED BUT BATTERY NOT DETECTED");
                BleChess.sendTestModeError("✓ FULL TEST PASSED BUT BATTERY NOT DETECTED");
                soundHandler(7);
                soundHandler(8);
                delay(1500);
            }
        }

        while (true)
        {
            Serial.println("✓ FULL TEST PASSED");
            BleChess.sendTestModeError("✓ FULL TEST PASSED");
            cute._tone(NOTE_C7, 50, 100);
            delay(1500);
        }
    }
}

// =============================================================================
// pauseMode – idle while paused (mode 3)
// =============================================================================

/**
 * @brief Block until the BLE mode changes away from 3.
 *
 * The board is fully paused: no motors move, no sensors are processed beyond
 * the background bleTask() polling.  The function returns as soon as the app
 * sends a different mode number.
 */
void pauseMode()
{
    Serial.println("-----------------Pause Mode MAIN");
    BleChess.setMode(3);
    BleChess.setState("Pause");

    while (true)
    {
        int newData = BleChess.getModeChess();
        if (newData != 3)
        {
            break;
        }
        delay(100);
    }
}

// =============================================================================
// sculptureMode – autonomous PGN replay (mode 1)
// =============================================================================

/**
 * @brief Thin wrapper around sculptureMain() (src/chess/pgn.cpp).
 *
 * sculptureMain() runs the complete autonomous game-replay state machine.
 * When it returns (e.g. because the app changed the mode), this function
 * polls until the mode is no longer 1 before returning to loop().
 */
void sculptureMode()
{
    sculptureMain();

    /* After sculptureMain() exits, drain any residual mode-1 state so loop()
     * doesn't immediately call us again if the mode hasn't propagated yet. */
    while (true)
    {
        int newData = BleChess.getModeChess();
        if (newData != 1)
        {
            break;
        }
        delay(100);
    }
}
