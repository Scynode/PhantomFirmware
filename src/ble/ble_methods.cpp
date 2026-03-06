/**
 * @file ble_methods.cpp
 * @brief Bluetooth:: static method implementations.
 *
 * This file implements all public static methods of the Bluetooth class
 * (declared in BLE.h) that are not related to battery management (those live
 * in ble_battery.cpp) and not related to initialisation (ble_server.cpp).
 *
 * These methods are the primary interface between the rest of the firmware
 * (main.cpp, scultpureMode.cpp) and the BLE subsystem.  They read from or
 * write to the global variables defined in ble_state.cpp.
 *
 * ─── Naming conventions ────────────────────────────────────────────────────
 *   get*()   – return a global value; may consume (clear) it after returning
 *   set*()   – update a global value and push it to the app via notify()
 *   reset*() – zero / clear a transient buffer
 *   verif*() – compare current vs. previous state and return a delta
 */

#include "BLE.h"
#include <config.h>
#include <Arduino.h>
#include <Preferences.h>
#include "ble_state.h"

// ---------------------------------------------------------------------------
// Movement command polling
// ---------------------------------------------------------------------------

/**
 * Returns the latest move command received via GENERAL_UUID_RECEIVE_MOVEMENT
 * if it differs from the last consumed command; otherwise returns "".
 * The caller must process the returned string before the app sends another move.
 */
String Bluetooth::verifNewCommandBluetooth()
{
    String resultMessage = "";
    if (currentBluetoothMessage != newBluetoothMessage)
    {
        resultMessage          = newBluetoothMessage;
        currentBluetoothMessage = newBluetoothMessage;
    }
    return resultMessage;
}

/**
 * Clears both the current and pending movement command buffers.
 * Call this after fully processing a move to prevent stale commands.
 */
void Bluetooth::resetVerifNewCommandBluetooth()
{
    currentBluetoothMessage = "";
    newBluetoothMessage     = "";
}

// ---------------------------------------------------------------------------
// Status / state push to app
// ---------------------------------------------------------------------------

/**
 * Pushes a physical-move status string to the app via GENERAL_UUID_STATUS_BOARD.
 * Format matches what the app expects, e.g. "e2e4" for a detected board move.
 */
void Bluetooth::setStatus(String status)
{
    generalCharacteristic_status_board->setValue(status);
    generalCharacteristic_status_board->notify();
    Serial.println("Physical move to App: " + status);
}

/**
 * Sends a compressed board-state matrix to the app via GENERAL_UUID_SEND_MATRIX.
 * @param error    Error code / label string prepended to the payload.
 * @param sensors  10×10 boolean sensor state (true = magnet / piece detected).
 * @param matrixToSend 10×10 char matrix encoding piece types / colours.
 *
 * Wire format: "error,matrixChars,sensorBits"
 *   matrixChars – 100-character run of char values (row-major)
 *   sensorBits  – 100-character run of '0'/'1' sensor values
 */
void Bluetooth::sendMatrixToApp(String error, bool sensors[10][10], char matrixToSend[10][10])
{
    String strSensors = "";
    String strMatriz  = "";
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            strSensors += sensors[i][j];
            strMatriz  += matrixToSend[i][j];
        }
    }
    generalCharacteristic_send_matrix->setValue(error + "," + strMatriz + "," + strSensors);
    generalCharacteristic_send_matrix->notify();
}

/**
 * Pushes a test-mode error string to the app via GENERAL_UUID_SEND_TESTMODE_ERROR.
 */
void Bluetooth::sendTestModeError(String error)
{
    generalCharacteristic_send_testMode_Error->setValue(error);
    generalCharacteristic_send_testMode_Error->notify();
}

// ---------------------------------------------------------------------------
// Voice command
// ---------------------------------------------------------------------------

/** Returns the latest voice command string written by the app. */
String Bluetooth::getVoiceCommand()
{
    return globalVoice;
}

/** Clears the voice command buffer after the firmware has processed it. */
void Bluetooth::resetVoiceCommand()
{
    globalVoice = "";
}

// ---------------------------------------------------------------------------
// Matrix init game
// ---------------------------------------------------------------------------

/** Returns the board-matrix string the app sent to seed a new game. */
String Bluetooth::getMatrixInitGame()
{
    return globalMatrixInitGame;
}

/** Clears the matrix init buffer after the firmware has consumed it. */
void Bluetooth::resetMatrixInitGame()
{
    globalMatrixInitGame = "";
}

// ---------------------------------------------------------------------------
// Move validity
// ---------------------------------------------------------------------------

/**
 * Returns the last move-check result written by the app via GENERAL_UUID_CHECK_MOVE.
 * "1" = legal move (execute it); "2" = illegal move (undo / reject it).
 */
String Bluetooth::getCheckMove()
{
    return globalCheckMovement;
}

/**
 * Overrides the check-move result from the firmware side (e.g. to pre-populate
 * before the app has replied).
 */
void Bluetooth::setCheckMove(String result)
{
    globalCheckMovement = result;
}

// ---------------------------------------------------------------------------
// Play info
// ---------------------------------------------------------------------------

/**
 * Returns the game-context payload most recently sent by the app via UUID_PLAY_INFO
 * (e.g. player names, game title, current FEN).
 */
String Bluetooth::getPlayInfo()
{
    return globalPlayInfo;
}

/**
 * Clears the in-bound play-info buffer and pushes a new value to the app.
 * Used when the firmware needs to echo or update play info.
 */
void Bluetooth::setPlayInfo(String _playInfo)
{
    globalPlayInfo = "";
    generalCharacteristic_play_info->setValue(_playInfo);
}

// ---------------------------------------------------------------------------
// Game mode
// ---------------------------------------------------------------------------

/** Returns the current game mode index (persisted in NVS "modeGameplay"). */
int Bluetooth::getModeChess()
{
    return modeChess;
}

/**
 * Updates the game mode from the firmware side and syncs it to the app and NVS.
 * Also sends a BLE notification so the app's UI reflects the change immediately.
 */
void Bluetooth::setMode(int _mode)
{
    modeChess = _mode;
    preferences.begin("myApp", false);
    preferences.putInt("modeGameplay", modeChess);
    preferences.end();
    generalCharacteristic_select_mode->setValue(String(modeChess));
    generalCharacteristic_select_mode->notify();
}

// ---------------------------------------------------------------------------
// Board state string
// ---------------------------------------------------------------------------

/**
 * Pushes a human-readable state label to the app via UUID_SETSTATE.
 * Examples: "Initializating", "Playing", "Calibrating", "Error".
 */
void Bluetooth::setState(String stateLocal)
{
    state = stateLocal;
    generalCharacteristic_setState->setValue(state);
    generalCharacteristic_setState->notify();
}

// ---------------------------------------------------------------------------
// Playlist / game selection
// ---------------------------------------------------------------------------

/**
 * Returns the game index to play, and optionally advances the playlist pointer.
 *
 * @param mode  0 = query only (do not advance index)
 *              1 = advance to next game according to sculptureRepeat mode
 *
 * Repeat modes (sculptureRepeat):
 *   0 = stop at last game (sets pause and resets index to 0)
 *   1 = keep replaying the same game (index unchanged)
 *   2 = loop: wrap index back to 0 when the end is reached
 *
 * Returns -1 if the playlist is empty.
 */
int Bluetooth::gameToPlay(int mode)
{
    int currentIndexLocal = 0;
    int lastGametoPlay    = -1;
    int playlistSize      = 0;
    int gameToPlayResult  = 0;

    if (playlist.length() == 0)
    {
        return -1; // No playlist configured
    }

    // Parse dash-separated index list into a heap-allocated integer array
    int *playlistIndices = new int[200];
    for (int i = 0; i < (int)playlist.length(); i++)
    {
        if (playlist.charAt(i) == '-')
        {
            playlistIndices[playlistSize++] = playlist.substring(currentIndexLocal, i).toInt();
            currentIndexLocal = i + 1;
        }
    }
    lastGametoPlay = playlist.substring(currentIndexLocal).toInt();
    playlistIndices[playlistSize++] = lastGametoPlay;

    // Guard against a stale index that is now out of range
    if (currentIndex.toInt() >= playlistSize)
    {
        Serial.println("Current Index out of range, setting to 0");
        currentIndex = "0";
        preferences.begin("myApp", false);
        preferences.putString("lastFile", currentIndex);
        preferences.end();
    }

    gameToPlayResult = playlistIndices[currentIndex.toInt()];

    delete[] playlistIndices;
    playlistIndices = nullptr;

    if (mode == 1)
    {
        // Advance the playlist index according to the repeat mode
        if (sculptureRepeat == 0) // Stop at end
        {
            if (currentIndex.toInt() == playlistSize - 1)
            {
                Serial.print("NEXT GAME TO PLAY: STOP PLAYING, INDEX: ");
                currentIndex = "0";
                globalPause  = 1;
                generalCharacteristic_pause->setValue("1");
                generalCharacteristic_pause->notify();
            }
            else
            {
                Serial.print("NEXT GAME TO PLAY: NORMAL COUNTER, INDEX: ");
                currentIndex = String(currentIndex.toInt() + 1);
            }
        }
        else if (sculptureRepeat == 1) // Repeat same game
        {
            Serial.print("NEXT GAME TO PLAY: REPEAT SAME GAME, INDEX: ");
            // When at the last game, the index intentionally stays unchanged
            // (repeat the same game).  The self-assignment below is preserved
            // from the original code to make the no-op explicit.
            if (currentIndex.toInt() == playlistSize - 1)
            {
                // no-op: keep currentIndex as-is to replay the current game
                (void)currentIndex;
            }
        }
        else if (sculptureRepeat == 2) // Loop whole playlist
        {
            if (currentIndex.toInt() == playlistSize - 1)
            {
                Serial.print("NEXT GAME TO PLAY: REPEAT WHOLE PLAYLIST, INDEX: ");
                currentIndex = "0";
            }
            else
            {
                Serial.print("NEXT GAME TO PLAY: NORMAL COUNTER, INDEX: ");
                currentIndex = String(currentIndex.toInt() + 1);
            }
        }

        Serial.println(currentIndex);

        preferences.begin("myApp", false);
        preferences.putString("lastFile", currentIndex);
        preferences.end();

        generalCharacteristic_currentIndex->setValue(currentIndex);
        generalCharacteristic_currentIndex->notify();
    }

    return gameToPlayResult;
}

// ---------------------------------------------------------------------------
// Hardware settings (read-only getters used by the motion / audio subsystems)
// ---------------------------------------------------------------------------

/**
 * Returns the configured motor speed in mm/s.
 * "1" → 50 mm/s  |  "2" → 100 mm/s  |  "3" → 200 mm/s  |  default → 50 mm/s
 */
double Bluetooth::getMechanismSpeed()
{
    if (mechanismSpeed == "1") return 50.0;
    if (mechanismSpeed == "2") return 100.0;
    if (mechanismSpeed == "3") return 200.0;
    return 50.0; // Safe default
}

/** Returns the sound volume level as an integer (parsed from the string value). */
int Bluetooth::getSoundLevel()
{
    return soundLevel.toInt();
}

/**
 * Returns and clears the transient sound-check trigger.
 * Returns "1" or "2" if a sound effect was requested, "0" otherwise.
 * Only "1" and "2" are valid triggers; anything else is normalised to "0".
 */
String Bluetooth::getSoundCheck()
{
    Serial.println("GET Sound Check from BLE: " + soundCheck);
    if (soundCheck != "1" && soundCheck != "2")
    {
        soundCheck = "0";
    }
    String soundCheckLocal = soundCheck;
    soundCheck = ""; // Consume the trigger
    return soundCheckLocal;
}

/**
 * Returns the tutorial sound index and resets the pending value to -1.
 * Returns -1 if no tutorial sound is pending.
 */
int Bluetooth::getSoundTutorial()
{
    int soundTutorialLocal = soundTutorial.toInt();
    Serial.println("Sound tutorial from BLE: " + soundTutorialLocal);
    soundTutorial = "-1"; // Consume the trigger
    return soundTutorialLocal;
}

/**
 * Returns how many games to play before the gantry returns to the home position.
 */
int Bluetooth::getpatternsToHome()
{
    return patternsToHome.toInt();
}

/**
 * Returns 1 if the gantry should visit the centre square between piece moves,
 * 0 if it should go directly to the destination.
 */
int Bluetooth::getJumpToCenter()
{
    return jumpToCenter.toInt();
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------

/** Returns 1 if the sculpture is currently paused, 0 if running. */
int Bluetooth::getPauseInfo()
{
    return globalPause;
}

/**
 * Sets the pause state and notifies the app so its UI updates immediately.
 * @param globalPauseLocal  1 = pause, 0 = resume
 */
void Bluetooth::setPauseInfo(int globalPauseLocal)
{
    globalPause = globalPauseLocal;
    generalCharacteristic_pause->setValue(String(globalPause));
    generalCharacteristic_pause->notify();
}

/**
 * Returns the repeat mode:
 *   0 = stop playback at the last game
 *   1 = repeat the current game indefinitely
 *   2 = loop back to the first game when the playlist ends
 */
int Bluetooth::getRepeat()
{
    Serial.println("Sculpture Repeat from BLE: " + String(sculptureRepeat));
    return sculptureRepeat;
}

/**
 * Returns the inter-move delay in seconds for auto-play mode.
 * 0 means use the hard-coded default timing.
 */
int Bluetooth::getTimeToMove()
{
    return timeToMove;
}

/**
 * Returns 1 if the board should advance to the next move automatically,
 * 0 if it should wait for a manual step from the app.
 */
int Bluetooth::getAutoPlayback()
{
    return automaticPlayback;
}

/**
 * Returns the pending single-move request integer and resets it to -1.
 * Returns -1 if no single-move request is pending.
 */
int Bluetooth::getSingleMove()
{
    int localSingleMove = globalSingleMove.toInt();
    globalSingleMove    = "-1"; // Consume the request
    return localSingleMove;
}

// ---------------------------------------------------------------------------
// Physical hardware configuration getters
// ---------------------------------------------------------------------------

/**
 * Returns the piece positional offset in mm, clamped to [0, 5].
 * Controls how far past the target square centre the magnet pushes a piece.
 */
float Bluetooth::getOffsetPieces()
{
    if (offsetPieces.toInt() < 0) offsetPieces = "0";
    if (offsetPieces.toInt() > 5) offsetPieces = "5";
    return offsetPieces.toFloat();
}

/**
 * Returns the board rotation in degrees (0, 90, 180, or 270).
 * When in test mode or chess mode 4, always returns 0 (no rotation).
 */
int Bluetooth::getBoardRotation()
{
    if (modeChess == 4 || testModeSculpture)
    {
        return 0;
    }
    return boardRotation.toInt();
}

/**
 * Returns the calibration type, clamped to [0, 2].
 * 0 = default stall-guard, 1 = alternate, 2 = manual.
 */
int Bluetooth::getCalibType()
{
    if (calibType < 0 || calibType > 2)
    {
        calibType = 0;
    }
    return calibType;
}

// ---------------------------------------------------------------------------
// Takeback
// ---------------------------------------------------------------------------

/**
 * Returns the takeback payload written by the app and clears the buffer.
 * Format: "move1[ move2],FEN" – see ble_callbacks.h for details.
 * Returns "" if no takeback is pending.
 */
String Bluetooth::getTakeback()
{
    String takebackLocal = globalTakeback;
    globalTakeback       = ""; // Consume the request
    return takebackLocal;
}

// ---------------------------------------------------------------------------
// Factory reset
// ---------------------------------------------------------------------------

/**
 * Resets ALL NVS settings to factory defaults and clears the playlist.
 * Called from generalCallbacks_factory_reset::onWrite() which then restarts.
 *
 * Default values after reset:
 *   modeGameplay    = 1          sculptureRepeat = 2
 *   timeToMove      = 0          autoPlayback    = 1
 *   mechanismSpeed  = "2"        soundLevel      = "2"
 *   jumpToCenter    = "0"        patternsToHome  = "6"
 *   lastFile        = "150"      maxBattery      = 3200
 *   offsetPieces    = "3"        rotation        = 0
 *   calibType       = 0          betterCalib     = 1
 *   updatingStatus  = -1         (OTA not in progress)
 *   phantomName     = random 0–9999
 */
void Bluetooth::factoryReset()
{
    preferences.begin("myApp", false);
    preferences.clear();
    preferences.putInt("testFlag",       0);
    preferences.putInt("sculptureRepeat", 2);
    preferences.putInt("timeToMove",     0);
    preferences.putInt("autoPlayback",   1);
    preferences.putInt("modeGameplay",   1);
    preferences.putString("mechanismSpeed", "2");
    preferences.putString("soundLevel",     "2");
    preferences.putString("jumpToCenter",   "0");
    preferences.putString("patternsToHome", "6");
    preferences.putString("lastFile",       "150");
    preferences.putFloat("maxBattery",  3200);
    preferences.putString("offsetPieces",   "3");
    preferences.putInt("updatingStatus", -1);
    preferences.putInt("rotation",       0);
    preferences.putInt("betterCalib",    1);
    preferences.putInt("calibType",      0);
    preferences.putInt("stallOld",       40);
    preferences.putInt("stallOlder",     40);
    preferences.putInt("stallOldD2",     40);
    preferences.putInt("stallOlderD2",   40);
    randomSeed(analogRead(4));
    preferences.putInt("phantomName", random(9999));
    preferences.end();

    resetPlaylist();
}
