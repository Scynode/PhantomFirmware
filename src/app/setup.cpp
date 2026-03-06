/**
 * @file setup.cpp
 * @brief Arduino entry points (setup / loop) and top-level global definitions.
 *
 * This file owns every global that was previously at the top of main.cpp and
 * provides the two Arduino lifecycle functions the framework requires:
 *
 *   setup()  – runs once at boot; initialises serial, pins, BLE (via FreeRTOS
 *               task on core 0), sensors, drivers and performs the initial
 *               mechanical calibration.
 *
 *   loop()   – called repeatedly by the Arduino scheduler; dispatches to the
 *               correct mode handler based on the current BLE mode number.
 *
 * ─── Dual-core architecture ──────────────────────────────────────────────────
 *
 *   Core 0 runs bleTask(), which:
 *     • initialises the NimBLE stack (Bluetooth::init())
 *     • continuously calls Bluetooth::batterySim() to update the simulated
 *       battery level
 *     • continuously calls updateSensors() to refresh the sensorMatrix global
 *       from the hall-effect mux chain
 *
 *   Core 1 runs the Arduino loop() dispatcher:
 *     Mode 1 → sculptureMode()   autonomous PGN replay
 *     Mode 2 → playMode()        interactive two-player chess
 *     Mode 3 → pauseMode()       wait for mode change
 *     Mode 4 → testMode()        full hardware self-test
 *     Mode 5 → sound tutorial    play tones requested by app
 *     Mode 6 → lock position     move mechanism to maintenance position
 *     default→ errorMessage()    unexpected mode – halt with alert
 *
 * ─── Boot-counter (testFlag) logic ───────────────────────────────────────────
 *
 *   Each boot increments testFlag in NVS before any other initialisation.
 *   After a clean boot completes setup(), testFlag is reset to 0.
 *   If the board is power-cycled rapidly without reaching the reset:
 *     2 – 7  rapid cycles → trigger testMode() automatically
 *     8 – 14 rapid cycles → factory reset + reboot
 *
 * ─── OTA recovery ────────────────────────────────────────────────────────────
 *
 *   If a previous OTA update was interrupted ("updatingStatus" ≥ 0 in NVS),
 *   setup() enters an infinite loop broadcasting "Updating" over BLE so the
 *   app can detect the stale state and re-flash.
 *
 * ─── Hardware version check ──────────────────────────────────────────────────
 *
 *   Units shipped with hardwareV == -1 or 1 receive a one-time factory reset
 *   and are upgraded to hardwareV == 2.
 */

// ── Standard / Arduino includes ──────────────────────────────────────────────
#include <esp_system.h>
#include <config.h>
#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include "BLE.h"
#include "scultpureMode.h"
#include <CuteBuzzerSounds.h>
#include "soc/rtc_wdt.h"
#include "SPIFFS.h"
#include "FS.h"
#include <vector>

// ── App-layer includes ────────────────────────────────────────────────────────
#include "app_globals.h"

using namespace std;

// ── Global definitions ────────────────────────────────────────────────────────

/** The single BLE manager instance.  Declared extern in config.h so all
 *  subsystem files can call Bluetooth:: static methods without re-instantiation. */
Bluetooth BleChess;

/** NVS (Non-Volatile Storage) handle used throughout the firmware for
 *  persisting settings, playlists, calibration thresholds, etc. */
Preferences preferences;

/** Binary semaphore used to synchronise setup() with bleTask().
 *  setup() blocks on this semaphore; bleTask() gives it once BLE init is done,
 *  guaranteeing that the BLE stack is fully ready before the main loop starts. */
SemaphoreHandle_t xSemaphore;

/** True while the board is running the full sculpture-mode endurance test
 *  triggered by a board-initiated testMode.  sculptureMain() uses this to
 *  shorten its inner timing loops during the test. */
bool testModeSculpture = false;

/** Accumulates time (ms) that the motors were actively moving.  Used by
 *  Bluetooth::batterySim() to compute a more accurate battery discharge
 *  estimate (moving draws more current than holding). */
unsigned long timeMoving = 0;

/** Legacy 8×8 matrix scratch buffer; kept for compatibility. */
char matrix[8][8];

/** Live 10×10 boolean sensor map updated by bleTask() on core 0.
 *  false = piece present (hall-effect sensor active).
 *  true  = square empty. */
bool sensorMatrix[10][10];

/** Current mode, kept in sync with BleChess.getModeChess() at the top of
 *  each loop() iteration. */
int mode = 1;

/** NVS-persisted boot counter; see file-level doc for the full logic. */
int testFlag = 0;

/** Snapshot of sensorMatrix taken after each accepted move.  detectChangePlus()
 *  diffs incoming readings against this to identify delta events. */
bool previousChessState[10][10] = {0};

/** Column coordinate of the last piece lift detected by detectChangePlus(). */
int iniChangeX = 0;
/** Row coordinate of the last piece lift detected by detectChangePlus(). */
int iniChangeY = 0;
/** Column coordinate of the last piece placement detected by detectChangePlus(). */
int finChangeX = 0;
/** Row coordinate of the last piece placement detected by detectChangePlus(). */
int finChangeY = 0;

/** Running count of captured pieces since last full reset (debug aid). */
int totalDeadPieces = 0;
/** Sensor pass/fail counter used inside testMode(). */
int pieceSensorCount = 0;
/** Dead-piece sensor counter used inside testMode(). */
int deadPieceSensorCount = 0;
/** Number of captures that existed before the current automated BLE move;
 *  used in automaticMechanicMovement() to decide whether a capture occurred. */
int prevDeadPiecesCount = 0;

// ── Forward declarations for mode functions (defined in other app/ files) ─────
void sculptureMode();
void playMode();
void testMode();
void pauseMode();
String detectChangePlus(char[10][10], int &);
void soundEndGame();
void stringToMatrix(String, char[10][10]);
void automaticMechanicMovement(String, char[10][10]);

// =============================================================================
// bleTask – runs on core 0
// =============================================================================

/**
 * @brief FreeRTOS task that owns BLE initialisation and the sensor/battery
 *        polling loop.
 *
 * Pinned to CPU core 0 so that the NimBLE stack (which is also core-0
 * affine) and the tight sensor-polling loop do not compete with the motion
 * algorithms on core 1.
 *
 * After init the task enters an infinite loop:
 *   1. If a valid battery reading exists, advance the battery simulation by
 *      one time-step (batterySim tracks elapsed time internally).
 *   2. Read all 100 hall-effect sensors through the multiplexer chain and
 *      store the result in the shared sensorMatrix global.
 *   3. Yield to the FreeRTOS scheduler for one tick so higher-priority tasks
 *      (e.g. the BLE stack's own tasks) can run.
 */
void bleTask(void *parameter)
{
    /* Initialise NimBLE, create GATT service/characteristics, start advertising.
     * This call also loads all settings from NVS into the Bluetooth:: statics. */
    BleChess.init();

    /* Signal setup() that BLE is ready so the main thread can continue. */
    xSemaphoreGive(xSemaphore);

    while (true)
    {
        /* Update simulated battery level (charge/discharge model). */
        if (BleChess.getBatteryStatus() != -1)
        {
            BleChess.batterySim();
        }

        /* Refresh the live sensor matrix from the hall-effect mux chain. */
        updateSensors();

        /* Yield one FreeRTOS tick (~1 ms) to allow BLE stack tasks to run. */
        vTaskDelay(1);
    }
}

// =============================================================================
// setup – Arduino boot entry point
// =============================================================================

/**
 * @brief One-time hardware and software initialisation.
 *
 * Sequence:
 *   1. Start serial ports.
 *   2. Read / increment the boot counter (testFlag) in NVS.
 *   3. Check hardware version; factory-reset and reboot if on version ≤ 1.
 *   4. Spawn bleTask on core 0; wait for BLE init semaphore.
 *   5. Configure all GPIO pins.
 *   6. Initialise the sensor mux tables (sensorsDir) and stepper drivers.
 *   7. Reset the boot counter to 0 (clean boot).
 *   8. Dispatch to testMode, factory reset, OTA recovery, or normal startup
 *      depending on testFlag and NVS state.
 *   9. Perform initial mechanical calibration.
 */
void setup()
{
    Serial.begin(115200);
    /* Serial1 is used as the TMC2209 UART bus. */
    SERIAL_PORT2.begin(115200);

    /* Increment boot counter before anything else so power cycles are counted
     * even if the board crashes before reaching the reset at the end of setup(). */
    preferences.begin("myApp", false);
    testFlag = preferences.getInt("testFlag", 0);
    preferences.putInt("testFlag", ++testFlag);
    preferences.end();

    /* Hardware version gate – units shipped with version ≤ 1 need a one-time
     * factory reset to update their NVS schema to version 2. */
    preferences.begin("factorySetup", true);
    int hardwareVersion = preferences.getInt("hardwareV", -1);
    preferences.end();
    if (hardwareVersion == -1 || hardwareVersion == 1)
    {
        /* Write new hardware version, then reset defaults and reboot. */
        preferences.begin("factorySetup", false);
        preferences.clear();
        preferences.putInt("hardwareV", 2); // Bump to current hardware schema
        preferences.end();
        BleChess.factoryReset();
        esp_restart();
    }

    /* Create the semaphore that bleTask() signals when BLE init is done. */
    xSemaphore = xSemaphoreCreateBinary();

    /* Launch bleTask on core 0 with a 10 kB stack.
     * Priority 1 means it will be preempted by higher-priority BLE stack tasks
     * but runs at higher priority than idle. */
    xTaskCreatePinnedToCore(
        bleTask,   /* Function to run                          */
        "bleTask", /* Debug name                               */
        10000,     /* Stack size in bytes                      */
        NULL,      /* No parameter passed to the task          */
        1,         /* Priority (1 = low, yields to BLE stack)  */
        NULL,      /* Task handle (not needed here)            */
        0          /* Pin to core 0                            */
    );

    // ── GPIO pin configuration ────────────────────────────────────────────────

    /* Buzzer: configure as digital output then hand to CuteBuzzerSounds and
     * ledcSetup so we can play tones via hardware PWM on channel 5. */
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    cute.init(BUZZER_PIN);
    ledcSetup(5, 1000, 12);      /* channel 5, max 1 kHz, 12-bit resolution */
    ledcAttachPin(BUZZER_PIN, 5);

    /* Electromagnets: configure all four magnet drive pins as outputs.
     * Disconnect any leftover PWM channels and pull HIGH (off) initially.
     * The magnets are active-LOW via the driver circuitry. */
    pinMode(magnet1, OUTPUT);
    pinMode(magnet2, OUTPUT);
    pinMode(magnet3, OUTPUT);
    pinMode(magnet4, OUTPUT);
    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};
    for (int i = 0; i < 4; ++i)
    {
        ledcDetachPin(magnetPins[i]); /* Ensure no stale PWM channel is bound */
        digitalWrite(magnetPins[i], HIGH); /* HIGH = off                      */
    }

    /* Sensor multiplexer select lines: all outputs. */
    pinMode(mux8_0, OUTPUT);
    pinMode(mux8_1, OUTPUT);
    pinMode(mux8_2, OUTPUT);   /* Also doubles as battery mux S0 */
    pinMode(mux16_0, OUTPUT);  /* Also doubles as battery mux S1 */
    pinMode(mux16_1, OUTPUT);  /* Also doubles as battery mux S2 */
    pinMode(mux16_2, OUTPUT);

    /* Sensor data lines: inputs (open-drain hall-effect outputs pulled up
     * by the sensor board). */
    pinMode(mux16Out_1, INPUT);
    pinMode(mux16Out_2, INPUT);
    pinMode(mux16Out_3, INPUT);
    pinMode(mux16Out_4, INPUT);

    /* Stepper motor step/direction/enable pins. */
    pinMode(MOTOR_0_STEP_PIN, OUTPUT);
    pinMode(MOTOR_0_DIR_PIN, OUTPUT);
    pinMode(MOTOR_1_STEP_PIN, OUTPUT);
    pinMode(MOTOR_1_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); /* LOW = drivers enabled */

    // ── Sensor and driver initialisation ─────────────────────────────────────

    /* Pre-compute the mux address lookup table used by readRawSensors(). */
    sensorsDir();

    /* Configure the TMC2209 UART drivers (current, microstepping, etc.). */
    configDrivers();

    /* Reset boot counter NOW – if we reach this point, the boot was clean. */
    preferences.begin("myApp", false);
    preferences.putInt("testFlag", 0);
    int updatingStatus = preferences.getInt("updatingStatus", -1);
    preferences.end();

    /* Wait for bleTask() to finish BLE initialisation before proceeding. */
    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    // ── Boot-count dispatch ───────────────────────────────────────────────────

    if (testFlag >= 2 && testFlag < 8)
    {
        /* Rapid power cycles 2–7: enter hardware self-test mode. */
        soundHandler(1);      /* "entering test mode" chime */
        testFlag = -1;        /* -1 means test was board-triggered */
        BleChess.setMode(4);  /* Mode 4 = testMode */
    }
    else if (testFlag >= 8 && testFlag < 15)
    {
        /* Rapid power cycles 8–14: factory reset and reboot. */
        soundHandler(7);
        soundHandler(7);
        soundHandler(7);
        Serial.println("Factory Reset Mode");
        BleChess.factoryReset();
        esp_restart(); /* NOTE: does NOT gracefully de-initialise peripherals */
    }
    else
    {
        /* Normal boot path. */

        /* If a previous OTA update left "updatingStatus" ≥ 0 in NVS, the
         * new firmware never completed its first clean boot.  Broadcast the
         * "Updating" state indefinitely so the app can detect and recover. */
        if (updatingStatus >= 0)
        {
            BleChess.setState("Updating");
            int timer = millis();
            while (true)
            {
                int timerNow = millis();
                if ((timerNow - timer) > 2000)
                {
                    timer = timerNow;
                    BleChess.setState("Updating");
                }
            }
        }

        /* If the stored mode is not one of the three normal operating modes,
         * reset it to sculpture mode (1) so the board starts in a known state. */
        int modeSetup = BleChess.getModeChess();
        if (modeSetup != 1 && modeSetup != 2 && modeSetup != 3)
        {
            BleChess.setMode(1);
        }

        /* Play startup chime. */
        soundHandler(0);

        /* Perform initial mechanical calibration (homing). */
        mechanicalCalibration(BleChess.getCalibType());
    }

    Serial.println("batteryStatus: " + String(BleChess.getBatteryStatus()));
    Serial.println("Hardware Version: " + String(hardwareVersion));
    Serial.println("Firmware Version: 30 09 2024");
}

// =============================================================================
// Utility functions
// =============================================================================

/**
 * @brief Euclidean distance between two 2-D points.
 * @param x1, y1  First point coordinates (mm).
 * @param x2, y2  Second point coordinates (mm).
 * @return Distance in the same units as the inputs.
 */
float euclideanDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

/**
 * @brief Find the centre of the 50 mm grid square nearest to (x, y).
 *
 * Iterates over all 64 board squares (plus graveyard positions) and picks the
 * one whose centre minimises the Euclidean distance to the query point.
 *
 * The board coordinate system has its origin at the centre:
 *   • X increases left-to-right, ranging from –175 mm to +175 mm.
 *   • Y increases top-to-bottom, ranging from +175 mm to –175 mm.
 * Square centres are spaced 50 mm apart in both axes.
 *
 * @param[in]  x, y          Query point (mm, board coordinates).
 * @param[out] centerX       X coordinate of the nearest square centre.
 * @param[out] centerY       Y coordinate of the nearest square centre.
 * @param[out] kprima        1-based column index of the nearest square.
 * @param[out] lprima        1-based row index of the nearest square.
 */
void nearestCenter(float x, float y,
                   float &centerX, float &centerY,
                   float &kprima, float &lprima)
{
    int k = 1;
    int l = 1;
    /* Initialise to an impossibly-far point so the first candidate always wins. */
    centerX = 9999;
    centerY = 9999;

    /* Scan Y from +175 (top) to –175 (bottom) in 50 mm steps. */
    for (float i = 175; i >= -175; i -= 50)
    {
        /* Scan X from –175 (left) to +175 (right) in 50 mm steps. */
        for (float j = -175; j <= 175; j += 50)
        {
            if (euclideanDistance(x, y, j, i) < euclideanDistance(x, y, centerX, centerY))
            {
                kprima = l;
                lprima = k;
                centerX = j;
                centerY = i;
            }
            l++;
        }
        l = 1;
        k++;
    }
}

// =============================================================================
// loop – Arduino main loop (mode dispatcher)
// =============================================================================

/**
 * @brief Main Arduino loop – reads the current BLE mode and dispatches to the
 *        appropriate handler.
 *
 * Mode table
 * ──────────
 *   1  sculptureMode – autonomous PGN game replay
 *   2  playMode      – interactive two-player game with BLE opponent
 *   3  pauseMode     – idle; wait for mode change from the app
 *   4  testMode      – full hardware self-test sequence
 *   5  sound tutorial– play tones requested by the app (UUID_MAKE_SOUND)
 *   6  lock position – move the carriage to a physical maintenance position
 *   ≠1–6  errorMessage – unexpected mode; halt with repeated alert
 */
void loop()
{
    int newData = BleChess.getModeChess();
    mode = newData;
    switch (newData)
    {
    case 1:
        /* Autonomous sculpture / PGN replay mode. */
        sculptureMode();
        break;

    case 2:
        /* Interactive two-player chess (human vs. BLE engine). */
        playMode();
        break;

    case 3:
        /* Paused – do nothing until the app changes the mode. */
        pauseMode();
        break;

    case 4:
        /* Full hardware self-test. */
        testMode();
        break;

    case 5:
        /* Sound tutorial: play any tone the app requests via UUID_MAKE_SOUND. */
        while (BleChess.getModeChess() == 5)
        {
            int makeSound = BleChess.getSoundTutorial();
            if (makeSound != -1)
            {
                soundHandler(makeSound);
            }
        }
        break;

    case 6:
        /* Lock position mode: calibrate, then drive the carriage to the
         * physical maintenance/shipping position so the user can insert screws. */
        mechanicalCalibration(BleChess.getCalibType());
        BleChess.setState("Moving to Lock Position");
        {
            float tempPosX, tempPosY;
            /* Coordinates correspond to the physical lock position on the PCB. */
            rawMovement(-26.6, 198.5, -2, tempPosX, tempPosY);
        }
        BleChess.setState("In Lock Position");
        /* Broadcast error message every 10 seconds so the user knows why the
         * board is not responding. */
        {
            unsigned long timer = millis();
            BleChess.sendTestModeError(
                "MECHANISM IN LOCK POSITION, PUT SCREWS AND RESTART THE BOARD");
            while (true)
            {
                if (millis() - timer > 10000)
                {
                    BleChess.sendTestModeError(
                        "MECHANISM IN LOCK POSITION, PUT SCREWS AND RESTART THE BOARD");
                    timer = millis();
                }
                cute._tone(NOTE_C7, 50, 100);
                delay(2000);
            }
        }
        break;

    default:
        /* Should never happen in normal operation.  Halt with audible alert. */
        {
            unsigned long timer = millis();
            while (true)
            {
                cute._tone(NOTE_G3, 50, 100);
                if (millis() - timer > 10000)
                {
                    BleChess.sendTestModeError(
                        "ERROR: MA-LOOP-312. Please send error to the "
                        "Phantom Team and restart the product.");
                    timer = millis();
                }
            }
        }
        break;
    }
}
