/**
 * @file ble_state.h
 * @brief Extern declarations for all BLE global state variables.
 *
 * This header is the single source of truth for the names and types of every
 * global variable shared across the BLE subsystem.  Each variable is DEFINED
 * exactly once in ble_state.cpp; every other translation unit that needs one
 * of them includes this file to get the extern declaration.
 *
 * Include order for BLE module files:
 *   1. BLE.h          – class/UUID/characteristic-pointer declarations, Arduino types
 *   2. config.h       – project pin / mode constants, extern Preferences & flags
 *   3. ble_state.h    – this file
 */

#pragma once

#include "BLE.h"    // NimBLE types, String, BLECharacteristic, Fragment class
#include <config.h> // project-wide constants and extern Preferences / testModeSculpture

// ---------------------------------------------------------------------------
// Battery tracking
// ---------------------------------------------------------------------------
extern float batteryStatus_;       // Current estimated battery level (raw ADC units)
extern float batteryInit;          // Battery reading taken at startup (or power-event)
extern int   maxBattery;           // Maximum observed battery reading (calibrated full)
extern int   previousWallStatus;   // Wall-power status from the previous batterySim() call

extern unsigned long initialTimeCharging; // millis() when charger was connected
extern unsigned long initialTime;         // millis() at last power-event reference point

// ---------------------------------------------------------------------------
// BLE connection
// ---------------------------------------------------------------------------
// NOTE: globalConnect is *declared* as extern in BLE.h and *defined* in
// ble_state.cpp.  It must NOT be declared extern a second time here to avoid
// a duplicate extern warning; BLE.h already covers that declaration.

// ---------------------------------------------------------------------------
// Firmware version (sent to the app on UUID_VERSION read)
// ---------------------------------------------------------------------------
extern String versionUpdate;

// ---------------------------------------------------------------------------
// Game / mode state
// ---------------------------------------------------------------------------
extern int    modeChess;      // Current game mode index (persisted to NVS "modeGameplay")
extern String state;          // Human-readable board state string (e.g. "Initializating")

// ---------------------------------------------------------------------------
// Playlist management
// ---------------------------------------------------------------------------
extern String playlist;    // Dash-separated game-index list, e.g. "1-5-12"
extern String playlistDB;  // Dash-separated PGN storage indices, e.g. "201-202"

// ---------------------------------------------------------------------------
// Adjustable settings (all persisted in NVS "myApp" namespace)
// ---------------------------------------------------------------------------
extern String mechanismSpeed;   // "1"=50 mm/s  "2"=100 mm/s  "3"=200 mm/s
extern String soundLevel;       // Integer volume level as string
extern String soundCheck;       // Transient: "1"=sound A, "2"=sound B, ""=none
extern String soundTutorial;    // Transient tutorial sound index as string; "-1"=none
extern String patternsToHome;   // Number of games before returning to home position
extern String jumpToCenter;     // "1"=jump gantry to centre between moves, "0"=skip
extern int    globalPause;      // 1 = paused, 0 = running

extern int sculptureRepeat;    // 0=stop at end, 1=repeat same game, 2=repeat playlist
extern int timeToMove;         // Seconds between moves in auto-play mode
extern int automaticPlayback;  // 1=auto-advance to next move, 0=manual step

extern String currentIndex;       // Current position within the playlist (as string int)
extern String globalSingleMove;   // Single-move request from app ("-1" = none pending)
extern String offsetPieces;       // Piece offset in mm, clamped 0–5
extern String boardRotation;      // Board orientation: "0", "90", "180", or "270" degrees
extern int    calibType;          // Calibration algorithm: 0, 1, or 2
extern String batteryStatustoApp; // Formatted battery string sent to app: "pct,wall,chg,done"

// ---------------------------------------------------------------------------
// BLE command / event buffers
// ---------------------------------------------------------------------------
extern int    MAX_CHARACTERS_BTNAME;      // Maximum BLE name length (20)
extern String currentBluetoothMessage;   // Last consumed movement command
extern String newBluetoothMessage;        // Latest movement command from app

// GENERAL_UUID_CHECK_MOVE: app replies "1"=valid or "2"=invalid after verifying a move
extern String globalCheckMovement;

// UUID_PLAY_INFO: arbitrary JSON/string payload describing current game context
extern String globalPlayInfo;

// UUID_VOICE: voice command string received from app
extern String globalVoice;

// UUID_MATRIX_INIT_GAME: board matrix snapshot sent by app to seed a new game
extern String globalMatrixInitGame;

// UUID_TAKEBACK: "move1[ move2],FEN" – piece(s) to physically move back + new FEN
extern String globalTakeback;

// Scratch buffer used for PGN data assembly
extern String pgnStr;

// ---------------------------------------------------------------------------
// BLE Characteristic pointers
// (created during Bluetooth::init() and stored globally so all modules can
//  call setValue()/notify() without re-querying the service object)
// ---------------------------------------------------------------------------

// -- Config / settings characteristics --
extern BLECharacteristic *generalCharacteristic_version;        // UUID_VERSION
extern BLECharacteristic *generalCharacteristic_select_mode;    // GENERAL_UUID_SELECT_MODE
extern BLECharacteristic *generalCharacteristic_setState;       // UUID_SETSTATE
extern BLECharacteristic *generalCharacteristic_mechanism_speed; // UUID_MECHANISM_SPEED
extern BLECharacteristic *generalCharacteristic_sound_level;    // UUID_SOUND_LEVEL
extern BLECharacteristic *generalCharacteristic_sound_check;    // UUID_MAKE_SOUND
extern BLECharacteristic *generalCharacteristic_sculpture_to_home; // UUID_SCULPTURE_TO_HOME
extern BLECharacteristic *generalCharacteristic_jump_to_center; // UUID_JUMP_TO_CENTER

// -- Playback control characteristics --
extern BLECharacteristic *generalCharacteristic_pause;              // UUID_PAUSE
extern BLECharacteristic *generalCharacteristic_currentIndex;       // UUID_CURRENTINDEX
extern BLECharacteristic *generalCharacteristic_single_move;        // UUID_SINGLE_MOVE
extern BLECharacteristic *generalCharacteristic_sculpture_repeat;   // UUID_REPEAT
extern BLECharacteristic *generalCharacteristic_sculpture_timeToMove; // UUID_TIMETOMOVE
extern BLECharacteristic *generalCharacteristic_automatic_playback; // UUID_AUTOMATIC_PLAYBACK

// -- Hardware / calibration characteristics --
extern BLECharacteristic *generalCharacteristic_offsetPieces;   // UUID_OFFSETPIECES
extern BLECharacteristic *generalCharacteristic_board_rotation; // UUID_BOARD_ROTATION (triggers restart)
extern BLECharacteristic *generalCharacteristic_calib_type;     // UUID_CALIB_TYPE (triggers restart)
extern BLECharacteristic *generalCharacteristic_batteryInfo;    // GENERAL_UUID_BATTERYINFO
extern BLECharacteristic *generalCharacteristic_factory_reset;  // UUID_FACTORY_RESET

// -- Chess / game characteristics --
extern BLECharacteristic *generalCharacteristic_errorMsg;          // GENERAL_UUID_ERRORMSG
extern BLECharacteristic *generalCharacteristic_receive_movement;  // GENERAL_UUID_RECEIVE_MOVEMENT
extern BLECharacteristic *generalCharacteristic_status_board;      // GENERAL_UUID_STATUS_BOARD
extern BLECharacteristic *generalCharacteristic_send_matrix;       // GENERAL_UUID_SEND_MATRIX
extern BLECharacteristic *generalCharacteristic_send_testMode_Error; // GENERAL_UUID_SEND_TESTMODE_ERROR
extern BLECharacteristic *generalCharacteristic_check_move;        // GENERAL_UUID_CHECK_MOVE
extern BLECharacteristic *generalCharacteristic_play_info;         // UUID_PLAY_INFO
extern BLECharacteristic *generalCharacteristic_takeback;          // UUID_TAKEBACK

// -- Voice / matrix characteristics --
extern BLECharacteristic *generalCharacteristic_voice;            // UUID_VOICE
extern BLECharacteristic *generalCharacteristic_matrix_init_game; // UUID_MATRIX_INIT_GAME

// -- OTA update characteristic --
extern BLECharacteristic *fileCharacteristic_receiveOta; // FILE_UUID_RECEIVE_OTA

// -- Playlist characteristics --
extern BLECharacteristic *generalCharacteristic_playlist;      // UUID_PLAYLIST
extern BLECharacteristic *generalCharacteristic_playlist_db;   // UUID_PLAYLIST_DB
extern BLECharacteristic *generalCharacteristic_playlist_sync; // UUID_PLAYLIST_SYNC
extern BLECharacteristic *generalCharacteristic_playlist_del;  // UUID_PLAYLIST_DEL

// ---------------------------------------------------------------------------
// Free-function forward declaration (defined in ble_fragment.cpp)
// ---------------------------------------------------------------------------
// Resets all playlist preferences to factory defaults and clears runtime vars.
void resetPlaylist();
