/**
 * @file ble_callbacks.cpp
 * @brief Implementations of all BLE characteristic callback classes.
 *
 * This file provides the onWrite() / onRead() bodies for every callback class
 * declared in ble_callbacks.h, with the exception of FilesCallbacks_receiveOta
 * (implemented in ble_ota.cpp) and bleServerCallback (defined inline in
 * ble_server.cpp because it is only used there).
 *
 * General pattern for settings characteristics:
 *   onWrite – store the new value in the corresponding global variable and, if
 *             the setting is persistent, write it to NVS via preferences.
 *   onRead  – push the current value back to the connected app via setValue()
 *             + notify() so the app always shows the live board state.
 *
 * Characteristics that trigger a hardware restart (board_rotation, calib_type)
 * call delay(1000) before esp_restart() to allow the BLE notification to reach
 * the app before the radio shuts down.
 */

#include "BLE.h"
#include <config.h>
#include <Preferences.h>
#include <Arduino.h>
#include "ble_state.h"
#include "ble_callbacks.h"

// ---------------------------------------------------------------------------
// Version (UUID_VERSION)
// The app reads this on connect to decide whether an OTA update is available.
// ---------------------------------------------------------------------------
void generalCallbacks_version::onWrite(BLECharacteristic *characteristic) {}

void generalCallbacks_version::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(versionUpdate);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Game mode (GENERAL_UUID_SELECT_MODE)
// The app writes the mode index; the board persists it and switches gameplay.
// ---------------------------------------------------------------------------
void generalCallbacks_select_mode::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    modeChess = atoi(rxData.c_str());
    preferences.begin("myApp", false);
    preferences.putInt("modeGameplay", modeChess);
    preferences.end();
}

void generalCallbacks_select_mode::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Board state (UUID_SETSTATE)
// Board pushes state strings to the app; the app only observes this value.
// ---------------------------------------------------------------------------
void generalCallbacks_set_state::onWrite(BLECharacteristic *characteristic) {}

void generalCallbacks_set_state::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Mechanism speed (UUID_MECHANISM_SPEED)
// "1"=50 mm/s  "2"=100 mm/s  "3"=200 mm/s  (anything else defaults to 50)
// Persisted in NVS so the speed survives a reboot.
// ---------------------------------------------------------------------------
void generalCallbacks_mechanism_speed::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    mechanismSpeed = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putString("mechanismSpeed", mechanismSpeed);
    preferences.end();
}

void generalCallbacks_mechanism_speed::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(mechanismSpeed);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Sound volume level (UUID_SOUND_LEVEL)
// Integer sound volume; persisted so the user's preference is remembered.
// ---------------------------------------------------------------------------
void generalCallbacks_sound_level::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    soundLevel = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putString("soundLevel", soundLevel);
    preferences.end();
}

void generalCallbacks_sound_level::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(soundLevel);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Transient sound trigger (UUID_MAKE_SOUND)
// "1" = play effect A, "2" = play effect B.  The value is consumed once by
// Bluetooth::getSoundCheck() and then cleared to empty.  NOT persisted.
// ---------------------------------------------------------------------------
void generalCallbacks_sound_check::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    soundCheck = rxData.c_str();
}

void generalCallbacks_sound_check::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Games per home-return (UUID_SCULPTURE_TO_HOME)
// After this many complete games the gantry drives back to the home position.
// Persisted so the museum operator's calibration survives a reboot.
// ---------------------------------------------------------------------------
void generalCallbacks_sculpture_to_home::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    patternsToHome = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putString("patternsToHome", patternsToHome);
    preferences.end();
}

void generalCallbacks_sculpture_to_home::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(patternsToHome);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Jump-to-centre flag (UUID_JUMP_TO_CENTER)
// "1" = gantry moves to the centre square between piece movements (smoother
// presentation); "0" = skip the centre detour for faster play.  Persisted.
// ---------------------------------------------------------------------------
void generalCallbacks_jump_to_center::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    jumpToCenter = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putString("jumpToCenter", jumpToCenter);
    preferences.end();
}

void generalCallbacks_jump_to_center::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(jumpToCenter);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Pause / resume (UUID_PAUSE)
// App writes 1 to pause, 0 to resume.  The main loop polls getPauseInfo().
// NOT persisted – the board always starts in running mode after a reboot.
// ---------------------------------------------------------------------------
void generalCallbacks_pause::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    globalPause = std::stoi(rxData);
    Serial.println("Pause onWrite: " + String(globalPause));
}

void generalCallbacks_pause::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Current playlist index (UUID_CURRENTINDEX)
// Persisted to NVS key "lastFile" so the sculpture resumes from the correct
// game after a power cycle.
// ---------------------------------------------------------------------------
void generalCallbacks_currentIndex::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    currentIndex = rxData.c_str();
    Serial.println("Next data onWrite: " + currentIndex);
    preferences.begin("myApp", false);
    preferences.putString("lastFile", currentIndex);
    preferences.end();
}

void generalCallbacks_currentIndex::onRead(BLECharacteristic *characteristic)
{
    Serial.println("Reading data from BLE: " + currentIndex);
    characteristic->setValue(currentIndex);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Repeat mode (UUID_REPEAT)
// 0 = stop playback at the last game in the playlist
// 1 = keep replaying the same game indefinitely
// 2 = loop back to the first game when the playlist is exhausted
// Persisted so the mode survives a reboot.
// ---------------------------------------------------------------------------
void generalCallbacks_sculpture_repeat::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    sculptureRepeat = std::stoi(rxData);
    Serial.printf("Sculpture Repeat onWrite: %d\n", sculptureRepeat);
    preferences.begin("myApp", false);
    preferences.putInt("sculptureRepeat", sculptureRepeat);
    preferences.end();
}

void generalCallbacks_sculpture_repeat::onRead(BLECharacteristic *characteristic)
{
    Serial.printf("Read Sculpture Repeat from BLE: %d\n", sculptureRepeat);
    String sculptureRepeatStr = String(sculptureRepeat);
    characteristic->setValue(sculptureRepeatStr);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Seconds between auto-play moves (UUID_TIMETOMOVE)
// 0 = use the hard-coded default move timing.  Persisted.
// ---------------------------------------------------------------------------
void generalCallbacks_sculpture_timeToMove::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    timeToMove = std::stoi(rxData);
    Serial.printf("Time to Move onWrite: %d\n", timeToMove);
    preferences.begin("myApp", false);
    preferences.putInt("timeToMove", timeToMove);
    preferences.end();
}

void generalCallbacks_sculpture_timeToMove::onRead(BLECharacteristic *characteristic)
{
    Serial.printf("Read Time to Move from BLE: %d\n", timeToMove);
    String timeToMoveStr = String(timeToMove);
    characteristic->setValue(timeToMoveStr);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Automatic playback (UUID_AUTOMATIC_PLAYBACK)
// 1 = board advances to the next move automatically after the delay expires
// 0 = board waits for a manual step signal from the app
// Persisted so the setting survives a reboot.
// ---------------------------------------------------------------------------
void generalCallbacks_automatic_playback::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    automaticPlayback = std::stoi(rxData);
    Serial.println("Automatic Playback onWrite: " + String(automaticPlayback));
    preferences.begin("myApp", false);
    preferences.putInt("autoPlayback", automaticPlayback);
    preferences.end();
}

void generalCallbacks_automatic_playback::onRead(BLECharacteristic *characteristic)
{
    Serial.println("Read Automatic Playback from BLE: " + String(automaticPlayback));
    String autoPlaybackStr = String(automaticPlayback);
    characteristic->setValue(autoPlaybackStr);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Single-move request (UUID_SINGLE_MOVE)
// App writes an integer index; board processes it once via getSingleMove()
// which resets globalSingleMove to "-1".  NOT persisted.
// ---------------------------------------------------------------------------
void generalCallbacks_single_move::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    globalSingleMove = rxData.c_str();
}

void generalCallbacks_single_move::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Piece offset (UUID_OFFSETPIECES)
// Float value 0–5 mm; controls how far pieces are pushed past the target
// square centre before being released.  Persisted.
// ---------------------------------------------------------------------------
void generalCallbacks_offsetPieces::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    offsetPieces = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putString("offsetPieces", offsetPieces);
    preferences.end();
}

void generalCallbacks_offsetPieces::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(offsetPieces);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Board rotation (UUID_BOARD_ROTATION)
// Accepted values: "0", "90", "180", "270" degrees.
// Persisted, then the board restarts to apply the new orientation.  The 1 s
// delay ensures the BLE notification reaches the app before the radio dies.
// ---------------------------------------------------------------------------
void generalCallbacks_board_rotation::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    boardRotation = rxData.c_str();
    preferences.begin("myApp", false);
    preferences.putInt("rotation", boardRotation.toInt());
    preferences.end();
    delay(1000);
    esp_restart();
}

void generalCallbacks_board_rotation::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(boardRotation);
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Calibration type (UUID_CALIB_TYPE)
// 0 = default stall-guard calibration
// 1 = alternate calibration algorithm
// 2 = manual calibration
// Also resets stall-detection history in NVS before restarting.
// ---------------------------------------------------------------------------
void generalCallbacks_calib_type::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    calibType = std::stoi(rxData);
    Serial.print("Calib Type onWrite: ");
    Serial.print(calibType);
    preferences.begin("myApp", false);
    preferences.putInt("calibType", calibType);
    // Reset stall-detection history so the new calibration starts fresh
    preferences.putInt("stallOld",     40);
    preferences.putInt("stallOlder",   40);
    preferences.putInt("stallOldD2",   40);
    preferences.putInt("stallOlderD2", 40);
    preferences.putInt("betterCalib",  1);
    preferences.end();
    delay(1000);
    esp_restart();
}

void generalCallbacks_calib_type::onRead(BLECharacteristic *characteristic)
{
    Serial.print("Read Calib Type from BLE: ");
    Serial.println(calibType);
    characteristic->setValue(String(calibType));
    characteristic->notify();
}

// ---------------------------------------------------------------------------
// Battery info (GENERAL_UUID_BATTERYINFO)
// Read-only; returns the batteryStatustoApp string that batterySim() builds:
//   "percent,wallStatus,charging,doneCharging"
// ---------------------------------------------------------------------------
void generalCallbacks_batteryInfo::onWrite(BLECharacteristic *characteristic) {}

void generalCallbacks_batteryInfo::onRead(BLECharacteristic *characteristic)
{
    Serial.println("BatteryStatus: " + String(batteryStatus_));
    Serial.println("BatteryToBLE: " + batteryStatustoApp);
    generalCharacteristic_batteryInfo->setValue(batteryStatustoApp);
    generalCharacteristic_batteryInfo->notify();
}

// ---------------------------------------------------------------------------
// Factory reset (UUID_FACTORY_RESET)
// Any write triggers a full NVS wipe (Bluetooth::factoryReset()) followed by
// an immediate restart.  The board comes up with default settings.
// ---------------------------------------------------------------------------
void generalCallbacks_factory_reset::onWrite(BLECharacteristic *characteristic)
{
    Bluetooth::factoryReset();
    delay(1000);
    esp_restart();
}

// ---------------------------------------------------------------------------
// Receive chess movement (GENERAL_UUID_RECEIVE_MOVEMENT)
// The app writes moves in "M f7-f5" format (start square dash end square).
// Moves longer than 25 characters are rejected with an error notification.
// The main loop polls via Bluetooth::verifNewCommandBluetooth().
// ---------------------------------------------------------------------------
void generalCallbacks_receive_movement::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String bluetoothName = rxData.c_str();
    if (bluetoothName.length() > 25)
    {
        String dataBLE = "error=  -1";
        generalCharacteristic_errorMsg->setValue(dataBLE);
        generalCharacteristic_errorMsg->notify();
        return;
    }
    newBluetoothMessage = bluetoothName;
}

void generalCallbacks_receive_movement::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Board status (GENERAL_UUID_STATUS_BOARD)
// Physical piece movements detected by the board are pushed to the app via
// Bluetooth::setStatus().  The app only observes; writes are ignored.
// ---------------------------------------------------------------------------
void generalCallbacks_status_board::onWrite(BLECharacteristic *characteristic) {}

void generalCallbacks_status_board::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Check-move result (GENERAL_UUID_CHECK_MOVE)
// After receiving a movement via GENERAL_UUID_RECEIVE_MOVEMENT the app
// validates it against the current game state and writes the result here:
//   "1" = legal move – the board should execute it
//   "2" = illegal move – the board should reject / undo it
// Clears the error-message characteristic as a side effect.
// ---------------------------------------------------------------------------
void generalCallbacks_check_move::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String bluetoothCheckMov = rxData.c_str();
    String dataBLE = "error=  -1";
    generalCharacteristic_errorMsg->setValue(dataBLE);
    generalCharacteristic_errorMsg->notify();
    Serial.print("Check Move Result: ");
    Serial.println(bluetoothCheckMov);
    globalCheckMovement = bluetoothCheckMov;
}

void generalCallbacks_check_move::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Play info (UUID_PLAY_INFO)
// Arbitrary game-context payload (e.g. game title, player names, current FEN)
// sent by the app.  Consumed once by Bluetooth::getPlayInfo() then cleared.
// ---------------------------------------------------------------------------
void generalCallbacks_play_info::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String playinfoStr = rxData.c_str();
    Serial.print("Play Info: ");
    Serial.println(playinfoStr);
    globalPlayInfo = playinfoStr;
}

void generalCallbacks_play_info::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Takeback (UUID_TAKEBACK)
// Wire format: "move1[ move2],FEN"
//   move1 / move2 – algebraic squares (e.g. "e4") for piece(s) to move back
//   FEN            – new board FEN after the undo is applied
// The main loop consumes this via Bluetooth::getTakeback().
// ---------------------------------------------------------------------------
void generalCallbacks_takeback::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String takebackStr = rxData.c_str();
    Serial.print("Received Takeback: ");
    Serial.println(takebackStr);
    globalTakeback = takebackStr;

    // Parse and log the takeback for debugging
    int commaIndex = takebackStr.indexOf(',');
    if (commaIndex != -1)
    {
        String takebackMoves = takebackStr.substring(0, commaIndex);
        String newFen = takebackStr.substring(commaIndex + 1);

        int spaceIndex = takebackMoves.indexOf(' ');
        if (spaceIndex != -1)
        {
            // Two moves to undo
            String firstMove  = takebackMoves.substring(0, spaceIndex);
            String secondMove = takebackMoves.substring(spaceIndex + 1);
            Serial.print("First Move: ");
            Serial.println(firstMove);
            Serial.print("Second Move: ");
            Serial.println(secondMove);
        }
        else
        {
            // Single move to undo
            Serial.print("Single Move: ");
            Serial.println(takebackMoves);
        }

        Serial.print("New FEN: ");
        Serial.println(newFen);
    }
    else
    {
        Serial.println("Error: Invalid takeback format - missing comma");
    }
}

// ---------------------------------------------------------------------------
// Voice command (UUID_VOICE)
// Arbitrary voice-command string from the app; consumed once by
// Bluetooth::getVoiceCommand() which resets globalVoice to "".
// ---------------------------------------------------------------------------
void generalCallbacks_voice::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String voice = rxData.c_str();
    globalVoice = voice;
}

void generalCallbacks_voice::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Matrix init game (UUID_MATRIX_INIT_GAME)
// Board matrix snapshot from the app used to seed the starting position of a
// new game.  Consumed once by Bluetooth::getMatrixInitGame() then cleared.
// ---------------------------------------------------------------------------
void generalCallbacks_matrix_init_game::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String matrixInitGame = rxData.c_str();
    globalMatrixInitGame = matrixInitGame;
    Serial.println("Matrix init game: ");
    Serial.println(globalMatrixInitGame);
}

void generalCallbacks_matrix_init_game::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Playlist (UUID_PLAYLIST)
// Wire format: "count,index1-index2-..." where count is the number of indices.
// Large playlists are fragmented: the app wraps the payload in
// "START<data>…<data>END" across multiple BLE writes.
// The Fragment helper in the class reassembles these into a single string.
// ---------------------------------------------------------------------------
void generalCallbacks_playlist::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    fragment.processData(rxData);
    if (fragment.isComplete())
    {
        std::string completeData = fragment.getBuffer();
        processData(completeData.c_str());
        fragment.clear();
    }
}

void generalCallbacks_playlist::onRead(BLECharacteristic *characteristic)
{
    Serial.printf("Playlist read: %s\n", playlist.c_str());
    // Playlist may be large; send it back in 100-byte fragments
    fragment.sendFragmentedData(characteristic, playlist.c_str());
}

void generalCallbacks_playlist::processData(String data)
{
    int split = data.indexOf(",");
    int count = data.substring(0, split).toInt();
    playlist  = data.substring(split + 1);

    preferences.begin("PL_setting");
    preferences.putString("playlist",      playlist);
    preferences.putInt("playlistCount",    count);
    preferences.end();

    Serial.println("Playlist received: ");
    Serial.println(playlist.c_str());
}

// ---------------------------------------------------------------------------
// Playlist database (UUID_PLAYLIST_DB)
// Wire format (fragmented): "<metadata>&&&<pgn>"
//   metadata – event|||date|||white|||black|||result fields
//   pgn      – full PGN move text
// Each write inserts one new record.  Indices are auto-incremented from
// NVS key "pgnIndex" (starting at 201) to avoid collisions with built-in games.
// ---------------------------------------------------------------------------
void generalCallbacks_playlist_db::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    fragment.processData(rxData);
    if (fragment.isComplete())
    {
        std::string completeData = fragment.getBuffer();
        processPgnData(completeData);
        fragment.clear();
    }
}

void generalCallbacks_playlist_db::onRead(BLECharacteristic *characteristic)
{
    characteristic->setValue(playlistDB);
    characteristic->notify();
}

void generalCallbacks_playlist_db::processPgnData(const std::string &completeData)
{
    // Split payload at "&&&" into metadata and PGN sections
    size_t pos = completeData.find("&&&");
    String metadata = completeData.substr(0, pos).c_str();
    String pgn      = completeData.substr(pos + 3).c_str();

    preferences.begin("PL_setting");
    int data  = preferences.getInt("pgnIndex", 201); // next available index
    int count = preferences.getInt("pgnCount",   0);
    preferences.putInt("pgnIndex", data + 1);
    preferences.putInt("pgnCount", count + 1);
    std::string indexStr = std::to_string(data);
    const char *indexKey = indexStr.c_str();
    preferences.end();

    preferences.begin("PL_metadata");
    preferences.putString(indexKey, metadata);
    preferences.end();

    preferences.begin("PL_pgn");
    preferences.putString(indexKey, pgn);
    preferences.end();

    // Append the new index to the in-memory and persisted playlistDB string
    if (playlistDB == "")
        playlistDB = String(data);
    else
        playlistDB = playlistDB + "-" + String(data);

    preferences.begin("PL_setting");
    preferences.putString("playlistDB", playlistDB);
    preferences.end();
}

// ---------------------------------------------------------------------------
// Playlist sync (UUID_PLAYLIST_SYNC)
// App writes a numeric index string to request one PGN record's metadata.
// Board replies with "<index>|||<metadata>" via indicate().
// App writes "finish" when it has received all records it needs.
// ---------------------------------------------------------------------------
void generalCallbacks_playlist_sync::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String data = rxData.c_str();

    if (data == "finish")
    {
        characteristic->setValue(data);
        characteristic->indicate();
        return;
    }

    Serial.println("");
    Serial.printf("Syncing playlist, data: %s\n", data.c_str());

    preferences.begin("PL_metadata");
    String metadata = preferences.getString(data.c_str(), "NOINIT");
    preferences.end();

    characteristic->setValue(data + "|||" + metadata);
    characteristic->indicate();
}

void generalCallbacks_playlist_sync::onRead(BLECharacteristic *characteristic) {}

// ---------------------------------------------------------------------------
// Playlist delete (UUID_PLAYLIST_DEL)
// Wire format: "count,updatedList,indexToDelete"
//   count         – new total record count after deletion
//   updatedList   – the new dash-separated index list
//   indexToDelete – NVS key of the record to remove from PL_metadata / PL_pgn
// Writing without a comma (no delimiter) triggers a full playlist reset.
// ---------------------------------------------------------------------------
void generalCallbacks_playlist_del::onWrite(BLECharacteristic *characteristic)
{
    std::string rxData = characteristic->getValue();
    String data = rxData.c_str();

    int split = data.indexOf(",");
    if (split == -1)
    {
        // No comma = reset the entire playlist to factory defaults
        resetPlaylist();
        Serial.println("Playlist reset.");
        return;
    }

    // Parse the three comma-separated fields
    String parts[3] = {"", "", ""};
    int i = 0;
    while (data.length() > 0 && i < 3)
    {
        split = data.indexOf(",");
        if (split == -1)
        {
            parts[i++] = data;
            break;
        }
        else
        {
            parts[i++] = data.substring(0, split);
            data        = data.substring(split + 1);
        }
    }

    int    count = parts[0].toInt();
    String list  = parts[1];
    String index = parts[2];

    updatePlaylistDB(count, list, index);
    Serial.printf("Playlist DB deleted, list: %s\n", list.c_str());
}

void generalCallbacks_playlist_del::onRead(BLECharacteristic *characteristic) {}

void generalCallbacks_playlist_del::updatePlaylistDB(
    const int count, const String &list, const String index)
{
    preferences.begin("PL_setting");
    preferences.putString("playlistDB", list);
    preferences.putInt("pgnCount",      count);
    preferences.end();

    preferences.begin("PL_metadata");
    preferences.remove(index.c_str());
    preferences.end();

    preferences.begin("PL_pgn");
    preferences.remove(index.c_str());
    preferences.end();

    playlistDB = list;
}
