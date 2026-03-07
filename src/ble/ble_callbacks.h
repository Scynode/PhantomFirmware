/**
 * @file ble_callbacks.h
 * @brief Class declarations for all BLE characteristic callback classes.
 *
 * Each class derived from BLECharacteristicCallbacks here handles onWrite()
 * and/or onRead() for one BLE characteristic in the General Config service
 * (SERVICE_UUID5).  Full method bodies live in ble_callbacks.cpp; only the
 * class skeleton is declared here so that ble_server.cpp can instantiate them
 * with `new`.
 *
 * The FilesCallbacks_receiveOta class is also declared here (its methods are
 * implemented in ble_ota.cpp) because ble_server.cpp must instantiate it via
 * `new FilesCallbacks_receiveOta()`.
 *
 * Characteristic purpose quick reference (see BLE.h for the UUID macros):
 *
 *  UUID_VERSION                 – firmware version string, read-only to app
 *  GENERAL_UUID_SELECT_MODE     – active game mode index (persisted to NVS)
 *  UUID_SETSTATE                – board state string pushed to app
 *  UUID_MECHANISM_SPEED         – "1"=50 mm/s  "2"=100 mm/s  "3"=200 mm/s
 *  UUID_SOUND_LEVEL             – integer volume 0-N (persisted)
 *  UUID_MAKE_SOUND              – transient trigger: "1" or "2" for a sound effect
 *  UUID_SCULPTURE_TO_HOME       – games-per-home-return count (persisted)
 *  UUID_JUMP_TO_CENTER          – "1"=jump to centre between moves (persisted)
 *  UUID_PAUSE                   – 1=paused / 0=running
 *  UUID_CURRENTINDEX            – current playlist position (persisted to NVS)
 *  UUID_SINGLE_MOVE             – single-move request from app ("-1"=none)
 *  UUID_REPEAT                  – 0=stop, 1=repeat same game, 2=repeat playlist
 *  UUID_TIMETOMOVE              – seconds between auto-play moves (persisted)
 *  UUID_AUTOMATIC_PLAYBACK      – 1=auto-advance, 0=manual step (persisted)
 *  UUID_OFFSETPIECES            – piece offset mm 0-5 (persisted)
 *  UUID_BOARD_ROTATION          – 0/90/180/270 degrees; TRIGGERS RESTART on write
 *  UUID_CALIB_TYPE              – calibration type 0/1/2; TRIGGERS RESTART on write
 *  GENERAL_UUID_BATTERYINFO     – battery status string "pct,wall,chg,done" (notify)
 *  UUID_FACTORY_RESET           – any write triggers factory reset + restart
 *  GENERAL_UUID_RECEIVE_MOVEMENT – chess move "M f7-f5" format from app
 *  GENERAL_UUID_STATUS_BOARD    – board-detected moves pushed to app (notify)
 *  GENERAL_UUID_CHECK_MOVE      – app writes "1"=valid or "2"=invalid move result
 *  UUID_PLAY_INFO               – game context payload (JSON / structured string)
 *  UUID_TAKEBACK                – "move1[ move2],FEN" – undo request from app
 *  UUID_VOICE                   – voice command string from app
 *  UUID_MATRIX_INIT_GAME        – board matrix from app to initialise a game
 *  UUID_PLAYLIST                – dash-separated game index list (fragmented)
 *  UUID_PLAYLIST_DB             – PGN + metadata insertion (fragmented)
 *  UUID_PLAYLIST_SYNC           – request a single PGN metadata entry by index
 *  UUID_PLAYLIST_DEL            – delete/replace a playlist entry
 *  FILE_UUID_RECEIVE_OTA        – binary OTA firmware packets (see ble_ota.cpp)
 */

#pragma once

#include "BLE.h"       // NimBLE types, BLECharacteristic, Fragment, String
#include "ble_state.h" // extern global variable declarations
#ifdef ESP32
#include "esp_ota_ops.h" // esp_ota_handle_t, esp_ota_begin(), etc.
#endif

// ---------------------------------------------------------------------------
// Version
// ---------------------------------------------------------------------------
class generalCallbacks_version : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Game mode selection (GENERAL_UUID_SELECT_MODE)
// ---------------------------------------------------------------------------
class generalCallbacks_select_mode : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// State push (UUID_SETSTATE) – board pushes state strings to the app
// ---------------------------------------------------------------------------
class generalCallbacks_set_state : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Mechanism (motor) speed (UUID_MECHANISM_SPEED)
// "1"=50 mm/s  "2"=100 mm/s  "3"=200 mm/s
// ---------------------------------------------------------------------------
class generalCallbacks_mechanism_speed : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Sound volume level (UUID_SOUND_LEVEL)
// Integer string; stored in NVS and applied to the audio subsystem
// ---------------------------------------------------------------------------
class generalCallbacks_sound_level : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Transient sound trigger (UUID_MAKE_SOUND)
// "1" = play effect A, "2" = play effect B; consumed once by getSoundCheck()
// ---------------------------------------------------------------------------
class generalCallbacks_sound_check : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Games per home-return (UUID_SCULPTURE_TO_HOME)
// After this many games the gantry drives back to the home position
// ---------------------------------------------------------------------------
class generalCallbacks_sculpture_to_home : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Jump-to-centre flag (UUID_JUMP_TO_CENTER)
// "1" = gantry visits the centre square between piece movements
// ---------------------------------------------------------------------------
class generalCallbacks_jump_to_center : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Pause / resume (UUID_PAUSE)
// 1 = sculpture paused, 0 = sculpture running
// ---------------------------------------------------------------------------
class generalCallbacks_pause : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Current playlist index (UUID_CURRENTINDEX)
// Persisted to NVS "lastFile" so playback resumes after a reboot
// ---------------------------------------------------------------------------
class generalCallbacks_currentIndex : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Repeat mode (UUID_REPEAT)
// 0=stop at last game, 1=repeat current game, 2=loop whole playlist
// ---------------------------------------------------------------------------
class generalCallbacks_sculpture_repeat : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Seconds between auto-play moves (UUID_TIMETOMOVE)
// 0 = use hard-coded default timing
// ---------------------------------------------------------------------------
class generalCallbacks_sculpture_timeToMove : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Automatic playback flag (UUID_AUTOMATIC_PLAYBACK)
// 1 = board advances to the next move automatically; 0 = wait for manual step
// ---------------------------------------------------------------------------
class generalCallbacks_automatic_playback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Single-move request (UUID_SINGLE_MOVE)
// App writes an integer; "-1" means no request pending
// ---------------------------------------------------------------------------
class generalCallbacks_single_move : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Piece offset (UUID_OFFSETPIECES)
// Float 0–5 mm; controls how far pieces are pushed beyond their target square
// ---------------------------------------------------------------------------
class generalCallbacks_offsetPieces : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Board rotation (UUID_BOARD_ROTATION)
// Accepted values: "0", "90", "180", "270".  Write triggers esp_restart().
// ---------------------------------------------------------------------------
class generalCallbacks_board_rotation : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Calibration type (UUID_CALIB_TYPE)
// 0 = default, 1 = alternate, 2 = manual.  Write triggers esp_restart().
// ---------------------------------------------------------------------------
class generalCallbacks_calib_type : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Battery information (GENERAL_UUID_BATTERYINFO)
// Read-only; returns "percent,wallStatus,charging,doneCharging"
// ---------------------------------------------------------------------------
class generalCallbacks_batteryInfo : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Factory reset (UUID_FACTORY_RESET)
// Any write triggers Bluetooth::factoryReset() followed by esp_restart()
// ---------------------------------------------------------------------------
class generalCallbacks_factory_reset : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Receive chess movement (GENERAL_UUID_RECEIVE_MOVEMENT)
// App writes moves in "M f7-f5" format (start square dash end square)
// ---------------------------------------------------------------------------
class generalCallbacks_receive_movement : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Board status (GENERAL_UUID_STATUS_BOARD)
// Board-detected physical moves are pushed to the app via notify
// ---------------------------------------------------------------------------
class generalCallbacks_status_board : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Check-move result (GENERAL_UUID_CHECK_MOVE)
// App writes "1" (valid move) or "2" (invalid move) after validating a move
// ---------------------------------------------------------------------------
class generalCallbacks_check_move : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Play info (UUID_PLAY_INFO)
// Arbitrary game-context payload (JSON or structured string) from the app
// ---------------------------------------------------------------------------
class generalCallbacks_play_info : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Takeback (UUID_TAKEBACK)
// Format: "move1[ move2],FEN"
//   move1 / move2 – algebraic squares (e.g. "e4") for piece(s) to move back
//   FEN            – new board position after the undo
// ---------------------------------------------------------------------------
class generalCallbacks_takeback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Voice command (UUID_VOICE)
// Arbitrary voice-command string from app; consumed once by getVoiceCommand()
// ---------------------------------------------------------------------------
class generalCallbacks_voice : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Matrix init game (UUID_MATRIX_INIT_GAME)
// Board matrix snapshot sent by the app to seed a new game's starting position
// ---------------------------------------------------------------------------
class generalCallbacks_matrix_init_game : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Playlist (UUID_PLAYLIST)
// Wire format: "count,index1-index2-..." arriving in START/chunk/END fragments
// The Fragment helper (declared in BLE.h) reassembles multi-packet payloads.
// ---------------------------------------------------------------------------
class generalCallbacks_playlist : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;

private:
    // Parse and persist a fully-reassembled playlist string
    void processData(String data);

    Fragment fragment; // per-connection reassembly buffer
};

// ---------------------------------------------------------------------------
// Playlist database (UUID_PLAYLIST_DB)
// Wire format (fragmented): "<metadata>&&&<pgn>"
// Each write inserts a new PGN record into NVS namespaces PL_metadata / PL_pgn
// ---------------------------------------------------------------------------
class generalCallbacks_playlist_db : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;

private:
    void processPgnData(const std::string &completeData);

    Fragment fragment;
};

// ---------------------------------------------------------------------------
// Playlist sync (UUID_PLAYLIST_SYNC)
// App writes a numeric index; board replies with "<index>|||<metadata>"
// App writes "finish" to signal end of sync; board confirms with an indicate.
// ---------------------------------------------------------------------------
class generalCallbacks_playlist_sync : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;
};

// ---------------------------------------------------------------------------
// Playlist delete (UUID_PLAYLIST_DEL)
// Wire format: "count,list,index"  – updates the playlist and removes the
// specified PGN record from NVS.  Writing without a comma resets the playlist.
// ---------------------------------------------------------------------------
class generalCallbacks_playlist_del : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;

private:
    void updatePlaylistDB(const int count, const String &list, const String index);
};

// ---------------------------------------------------------------------------
// OTA firmware update (FILE_UUID_RECEIVE_OTA)
// Full implementation is in ble_ota.cpp.  Declared here so ble_server.cpp
// can instantiate it with `new FilesCallbacks_receiveOta()`.
//
// Protocol overview:
//   1. First write:  4-byte little-endian uint32 = total firmware image size.
//      If this is the board's first OTA (updatingStatus != 0 in NVS) the board
//      saves updatingStatus=0 and immediately reboots into OTA-only mode.
//   2. Subsequent writes: raw binary firmware chunks (any size up to MTU).
//   3. When receivedSize >= totalSize the board calls esp_ota_end(),
//      esp_ota_set_boot_partition(), saves updatingStatus=-1, then reboots.
//
// Notification responses:
//   {0x01, 0x00} – chunk accepted, continue sending
//   {0x02, 0x00} – OTA complete, reboot imminent
//   {0xFF, code} – error; code values:
//       1 = invalid first packet (< 4 bytes)
//       2 = no OTA partition found
//       3 = esp_ota_begin() failed
//       4 = esp_ota_write() failed
//       5 = esp_ota_end() failed
//       6 = esp_ota_set_boot_partition() failed
// ---------------------------------------------------------------------------
class FilesCallbacks_receiveOta : public BLECharacteristicCallbacks
{
public:
    void onWrite(BLECharacteristic *characteristic) override;
    void onRead(BLECharacteristic *characteristic) override;

private:
    void initializeOTA();
    void writeOTAData();
    void finalizeOTA();
    void notifyError(uint8_t errorCode, const char *message);
    void notifySuccess();
    void notifyFinish();
    void notify(uint8_t status, uint8_t code);

    BLECharacteristic *localCharacteristic = nullptr;

    bool updateFlag    = false; // true once the first valid packet has been processed
    bool partitionReady = false; // true once esp_ota_begin() succeeds
    uint32_t totalSize    = 0;  // expected firmware image size (from first packet)
    uint32_t receivedSize = 0;  // bytes written so far
    const esp_partition_t *update_partition = nullptr;
    esp_ota_handle_t otaHandler = 0;
};
