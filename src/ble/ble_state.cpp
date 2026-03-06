/**
 * @file ble_state.cpp
 * @brief Definitions of all BLE global state variables.
 *
 * Every global variable used across the BLE subsystem is DEFINED here exactly
 * once.  All other translation units access them through the extern declarations
 * in ble_state.h.
 *
 * Persistent settings (mechanismSpeed, soundLevel, etc.) are loaded from NVS
 * during Bluetooth::init() (ble_server.cpp) and updated in-place whenever the
 * app writes a new value via a BLE characteristic callback.
 */

#include "BLE.h"
#include <config.h>
#include <Preferences.h>
#include "ble_state.h"

// ---------------------------------------------------------------------------
// Battery tracking
// ---------------------------------------------------------------------------
float batteryStatus_ = -1;  // Updated every loop by Bluetooth::batterySim()
float batteryInit     = -1;  // Snapshot at the last power-state transition
int   maxBattery      = -1;  // Loaded from NVS "maxBattery" during init()
int   previousWallStatus = -1; // Tracks last known wall-power state for edge detection

unsigned long initialTimeCharging = -1; // millis() when charger was plugged in
unsigned long initialTime         = -1; // millis() at last reference point for discharge calc

// ---------------------------------------------------------------------------
// BLE connection state
// (declared extern in BLE.h so the rest of the firmware can read it)
// ---------------------------------------------------------------------------
String globalConnect = "0"; // "1" when a BLE client is connected, "0" when not

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
String versionUpdate = "0.1.6"; // Sent to app on UUID_VERSION read

// ---------------------------------------------------------------------------
// Game / mode state
// ---------------------------------------------------------------------------
int    modeChess = 0;               // 0=unset, 1..N = specific game mode
String state     = "Initializating"; // Displayed in the app's status area

// ---------------------------------------------------------------------------
// Playlist management
// ---------------------------------------------------------------------------
String playlist   = "";  // Dash-separated game indices stored in NVS "PL_setting"/"playlist"
String playlistDB = "";  // Dash-separated PGN record indices, e.g. "201-202-203"

// ---------------------------------------------------------------------------
// Adjustable settings (persisted in NVS namespace "myApp")
// ---------------------------------------------------------------------------

// Mechanism speed: "1"=50 mm/s, "2"=100 mm/s, "3"=200 mm/s
String mechanismSpeed = "NOINIT";

// Sound volume level (integer string, e.g. "5")
String soundLevel = "NOINIT";

// Transient: set to "1" or "2" by app to trigger a specific sound effect; cleared after read
String soundCheck = "";

// Transient: tutorial sound index as string; "-1" means none pending
String soundTutorial = "-1";

// How many complete games to play before the gantry returns to the home position
String patternsToHome = "NOINIT";

// Whether the gantry jumps to the centre square between piece movements ("1"=yes, "0"=no)
String jumpToCenter = "NOINIT";

// Pause flag: 1 = sculpture paused, 0 = running
int globalPause = 0;

// Repeat mode: 0=stop at last game, 1=repeat current game, 2=loop whole playlist
int sculptureRepeat = -1;

// Seconds to wait between automated moves; 0 = use default timing
int timeToMove = -1;

// Automatic playback: 1=advance to next move automatically, 0=wait for manual step
int automaticPlayback = -1;

// Current position in the playlist (persisted in NVS "lastFile")
String currentIndex = "0";

// Single-move request from the app; "-1" means none pending
String globalSingleMove = "-1";

// Piece positional offset in mm (float, clamped 0–5 at read time)
String offsetPieces = "NOINIT";

// Board rotation in degrees: "0", "90", "180", "270" – triggers esp_restart() on change
String boardRotation = "NOINIT";

// Calibration algorithm selector (0, 1, or 2); triggers esp_restart() on change
int calibType = 0;

// Composite battery string sent to app: "percent,wallStatus,charging,doneCharging"
String batteryStatustoApp = "-";

// ---------------------------------------------------------------------------
// BLE command / event buffers
// ---------------------------------------------------------------------------
int    MAX_CHARACTERS_BTNAME    = 20;  // Max length of movement commands accepted via BLE
String currentBluetoothMessage  = "";  // Most recently consumed movement command
String newBluetoothMessage      = "";  // Latest raw value from GENERAL_UUID_RECEIVE_MOVEMENT

// Result of move validation: "1"=valid, "2"=invalid (written by app via GENERAL_UUID_CHECK_MOVE)
String globalCheckMovement = "0";

// Payload from UUID_PLAY_INFO (e.g. game metadata JSON); cleared after consumption
String globalPlayInfo = "";

// Voice command received from UUID_VOICE; cleared after consumption by Bluetooth::getVoiceCommand()
String globalVoice = "";

// Board matrix snapshot from UUID_MATRIX_INIT_GAME; cleared after consumption
String globalMatrixInitGame = "";

// Takeback data from UUID_TAKEBACK: "move1[ move2],FEN"; cleared after consumption
String globalTakeback = "";

// Scratch buffer for in-progress PGN string assembly
String pgnStr = "";

// ---------------------------------------------------------------------------
// BLE Characteristic pointers
// (assigned in Bluetooth::init(); nullptr until then)
// ---------------------------------------------------------------------------

BLECharacteristic *generalCharacteristic_version         = nullptr;
BLECharacteristic *generalCharacteristic_select_mode     = nullptr;
BLECharacteristic *generalCharacteristic_setState        = nullptr;
BLECharacteristic *generalCharacteristic_mechanism_speed = nullptr;
BLECharacteristic *generalCharacteristic_sound_level     = nullptr;
BLECharacteristic *generalCharacteristic_sound_check     = nullptr;
BLECharacteristic *generalCharacteristic_sculpture_to_home = nullptr;
BLECharacteristic *generalCharacteristic_jump_to_center  = nullptr;

BLECharacteristic *generalCharacteristic_pause               = nullptr;
BLECharacteristic *generalCharacteristic_currentIndex        = nullptr;
BLECharacteristic *generalCharacteristic_single_move         = nullptr;
BLECharacteristic *generalCharacteristic_sculpture_repeat    = nullptr;
BLECharacteristic *generalCharacteristic_sculpture_timeToMove = nullptr;
BLECharacteristic *generalCharacteristic_automatic_playback  = nullptr;

BLECharacteristic *generalCharacteristic_offsetPieces   = nullptr;
BLECharacteristic *generalCharacteristic_board_rotation = nullptr;
BLECharacteristic *generalCharacteristic_calib_type     = nullptr;
BLECharacteristic *generalCharacteristic_batteryInfo    = nullptr;
BLECharacteristic *generalCharacteristic_factory_reset  = nullptr;

BLECharacteristic *generalCharacteristic_errorMsg           = nullptr;
BLECharacteristic *generalCharacteristic_receive_movement   = nullptr;
BLECharacteristic *generalCharacteristic_status_board       = nullptr;
BLECharacteristic *generalCharacteristic_send_matrix        = nullptr;
BLECharacteristic *generalCharacteristic_send_testMode_Error = nullptr;
BLECharacteristic *generalCharacteristic_check_move         = nullptr;
BLECharacteristic *generalCharacteristic_play_info          = nullptr;
BLECharacteristic *generalCharacteristic_takeback           = nullptr;

BLECharacteristic *generalCharacteristic_voice            = nullptr;
BLECharacteristic *generalCharacteristic_matrix_init_game = nullptr;

BLECharacteristic *fileCharacteristic_receiveOta = nullptr; // OTA update characteristic

BLECharacteristic *generalCharacteristic_playlist      = nullptr;
BLECharacteristic *generalCharacteristic_playlist_db   = nullptr;
BLECharacteristic *generalCharacteristic_playlist_sync = nullptr;
BLECharacteristic *generalCharacteristic_playlist_del  = nullptr;
