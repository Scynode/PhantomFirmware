/**
 * @file ble_server.cpp
 * @brief BLE server initialisation, connection callbacks, and Bluetooth::init().
 *
 * This file owns:
 *   1. bleServerCallback  – handles client connect / disconnect events.
 *   2. Bluetooth::Bluetooth() – trivial constructor.
 *   3. Bluetooth::init()  – the main setup routine that:
 *        a) reads battery state and calibrates the initial charge estimate,
 *        b) loads all persistent settings from NVS,
 *        c) creates the NimBLE device, service, and all characteristics,
 *        d) attaches callback instances to each characteristic,
 *        e) starts advertising.
 *
 * ─── BLE Service layout ────────────────────────────────────────────────────
 *
 *   Service: SERVICE_UUID5 "fd31a840-22e7-11eb-adc1-0242ac120002"
 *
 *   All characteristics belong to this single "General Config" service.
 *   Refer to BLE.h for the UUID macros and ble_callbacks.h for the callback
 *   class documentation of each characteristic.
 *
 * ─── Connection parameters ─────────────────────────────────────────────────
 *
 *   min interval : 0x10 × 1.25 ms = 20 ms
 *   max interval : 0x20 × 1.25 ms = 40 ms
 *   latency      : 0  (no slave latency – keeps responsiveness high)
 *   timeout      : 800 × 10 ms = 8 s (supervision timeout)
 */

#include "BLE.h"
#include <config.h>
#include <Arduino.h>
#include <Preferences.h>
#include "ble_state.h"
#include "ble_callbacks.h"

// ---------------------------------------------------------------------------
// bleServerCallback – BLE connection lifecycle events
// ---------------------------------------------------------------------------
class bleServerCallback : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) override
    {
        globalConnect = "1";
        Serial.println("-----Bluetooth Device Connected");
    }

    void onDisconnect(BLEServer *pServer) override
    {
        globalConnect = "0";
        Serial.println("-----Bluetooth Device Disconnected");
    }
};

// ---------------------------------------------------------------------------
// Bluetooth constructor
// ---------------------------------------------------------------------------
Bluetooth::Bluetooth() {}

// ---------------------------------------------------------------------------
// Bluetooth::init()
// Called once from setup() in main.cpp after all other peripherals are ready.
// ---------------------------------------------------------------------------
void Bluetooth::init()
{
    // ── 1. Battery initialisation ─────────────────────────────────────────
    // Read the raw ADC battery voltage and configure the simulation baseline.
    batteryInit = batteryCheck(0); // mode 0 = read raw VBAT ADC average
    pinMode(4, INPUT);
    int doneCharging = batteryCheck(2); // mode 2 = done charging?
    int wallStatus   = batteryCheck(3); // mode 3 = wall power connected?

    initialTime       = millis();
    previousWallStatus = wallStatus;

    if (batteryInit == -1) // Battery not connected or read error
    {
        Serial.println("--------------------NO BATTERY");
        batteryStatustoApp = "-1";
        batteryStatus_     = -1;
    }
    else
    {
        // When on wall power the raw ADC over-reads slightly; apply a 5 %
        // correction so the simulation starts from the true resting voltage.
        batteryInit = (wallStatus) ? batteryInit * 0.95f : batteryInit;

        // If wall power is present, done-charging, and the voltage is in the
        // expected full-charge window, save it as the new calibrated maximum.
        if (wallStatus && doneCharging && batteryInit > 2800 && batteryInit < 3500)
        {
            preferences.begin("myApp", false);
            preferences.putFloat("maxBattery", batteryInit);
            preferences.end();
            Serial.println("-----Changing MaxBattery: " + String(batteryInit));
        }

        if (batteryInit == 4095) // ADC saturated = read error
        {
            Serial.println("Battery Read Error");
            // Fall back to a known good value depending on charger state
            batteryInit = (doneCharging) ? maxBattery : 2895;
        }

        if (wallStatus)
        {
            initialTimeCharging = millis();
        }

        preferences.begin("myApp", true);
        maxBattery = preferences.getFloat("maxBattery", 3200);
        preferences.end();

        Serial.println("--------------------Max Battery: "     + String(maxBattery));
        Serial.println("--------------------Initial Battery: " + String(batteryInit));
        batteryStatus_ = batteryInit;
    }

    Serial.printf("Battery Status: %f doneCharging: %d wallStatus: %d\n",
                  batteryStatus_, doneCharging, wallStatus);

    // ── 2. Load persistent settings from NVS ─────────────────────────────
    preferences.begin("myApp", true);
    String randomPhantom  = "Phantom " + String(preferences.getInt("phantomName", -1));
    modeChess             = preferences.getInt("modeGameplay", -1);
    sculptureRepeat       = preferences.getInt("sculptureRepeat", 2);
    timeToMove            = preferences.getInt("timeToMove", 0);
    automaticPlayback     = preferences.getInt("autoPlayback", 1);
    mechanismSpeed        = preferences.getString("mechanismSpeed", "empty");
    soundLevel            = preferences.getString("soundLevel",     "empty");
    patternsToHome        = preferences.getString("patternsToHome", "emtpy");
    jumpToCenter          = preferences.getString("jumpToCenter",   "empty");
    currentIndex          = preferences.getString("lastFile",       "0");
    offsetPieces          = preferences.getString("offsetPieces",   "empty");
    boardRotation         = preferences.getInt("rotation",  0);
    calibType             = preferences.getInt("calibType", 0);
    preferences.end();

    preferences.begin("PL_setting");
    playlist   = preferences.getString("playlist",   "1-2-3");
    playlistDB = preferences.getString("playlistDB", "");
    preferences.end();

    // ── 3. Create BLE device and service ─────────────────────────────────
    BLEDevice::init(randomPhantom.c_str());
    BLEServer  *pServer             = BLEDevice::createServer();
    BLEService *pServiceGeneralConfig = pServer->createService(BLEUUID(SERVICE_UUID5));
    pServer->setCallbacks(new bleServerCallback());

    // ── 4. Create characteristics ─────────────────────────────────────────

    // Firmware version (read + write; write ignored by the callback)
    generalCharacteristic_version = pServiceGeneralConfig->createCharacteristic(
        UUID_VERSION,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Game mode selection
    generalCharacteristic_select_mode = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SELECT_MODE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Board state string pushed to app
    generalCharacteristic_setState = pServiceGeneralConfig->createCharacteristic(
        UUID_SETSTATE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Motor speed: "1"=50 mm/s  "2"=100 mm/s  "3"=200 mm/s
    generalCharacteristic_mechanism_speed = pServiceGeneralConfig->createCharacteristic(
        UUID_MECHANISM_SPEED,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Sound volume level
    generalCharacteristic_sound_level = pServiceGeneralConfig->createCharacteristic(
        UUID_SOUND_LEVEL,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Games before the gantry returns to home
    generalCharacteristic_sculpture_to_home = pServiceGeneralConfig->createCharacteristic(
        UUID_SCULPTURE_TO_HOME,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Jump-to-centre toggle
    generalCharacteristic_jump_to_center = pServiceGeneralConfig->createCharacteristic(
        UUID_JUMP_TO_CENTER,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Pause / resume flag
    generalCharacteristic_pause = pServiceGeneralConfig->createCharacteristic(
        UUID_PAUSE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Repeat mode (0/1/2)
    generalCharacteristic_sculpture_repeat = pServiceGeneralConfig->createCharacteristic(
        UUID_REPEAT,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Seconds between auto-play moves
    generalCharacteristic_sculpture_timeToMove = pServiceGeneralConfig->createCharacteristic(
        UUID_TIMETOMOVE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Auto-advance flag
    generalCharacteristic_automatic_playback = pServiceGeneralConfig->createCharacteristic(
        UUID_AUTOMATIC_PLAYBACK,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Current playlist position
    generalCharacteristic_currentIndex = pServiceGeneralConfig->createCharacteristic(
        UUID_CURRENTINDEX,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Piece positional offset (mm)
    generalCharacteristic_offsetPieces = pServiceGeneralConfig->createCharacteristic(
        UUID_OFFSETPIECES,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Board rotation (triggers restart)
    generalCharacteristic_board_rotation = pServiceGeneralConfig->createCharacteristic(
        UUID_BOARD_ROTATION,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Calibration type (triggers restart)
    generalCharacteristic_calib_type = pServiceGeneralConfig->createCharacteristic(
        UUID_CALIB_TYPE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Battery status (read + notify)
    generalCharacteristic_batteryInfo = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_BATTERYINFO,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Factory reset trigger (write + notify)
    generalCharacteristic_factory_reset = pServiceGeneralConfig->createCharacteristic(
        UUID_FACTORY_RESET,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Move validity result from app ("1"=valid, "2"=invalid)
    generalCharacteristic_check_move = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_CHECK_MOVE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Error message pushed to app (notify-only from board side)
    generalCharacteristic_errorMsg = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_ERRORMSG,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Chess move input from app ("M f7-f5" format)
    generalCharacteristic_receive_movement = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_RECEIVE_MOVEMENT,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Physical move detected by the board (notify to app)
    generalCharacteristic_status_board = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_STATUS_BOARD,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Board matrix snapshot pushed to app
    generalCharacteristic_send_matrix = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SEND_MATRIX,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Test-mode error string pushed to app
    generalCharacteristic_send_testMode_Error = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SEND_TESTMODE_ERROR,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Single-move request from app
    generalCharacteristic_single_move = pServiceGeneralConfig->createCharacteristic(
        UUID_SINGLE_MOVE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Arbitrary game-context payload
    generalCharacteristic_play_info = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAY_INFO,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Undo request: "move1[ move2],FEN"
    generalCharacteristic_takeback = pServiceGeneralConfig->createCharacteristic(
        UUID_TAKEBACK,
        NIMBLE_PROPERTY::WRITE);

    // OTA firmware delivery
    fileCharacteristic_receiveOta = pServiceGeneralConfig->createCharacteristic(
        FILE_UUID_RECEIVE_OTA,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Transient sound trigger
    generalCharacteristic_sound_check = pServiceGeneralConfig->createCharacteristic(
        UUID_MAKE_SOUND,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Voice command from app
    generalCharacteristic_voice = pServiceGeneralConfig->createCharacteristic(
        UUID_VOICE,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Board matrix for game initialisation
    generalCharacteristic_matrix_init_game = pServiceGeneralConfig->createCharacteristic(
        UUID_MATRIX_INIT_GAME,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    // Playlist (fragmented read/write)
    generalCharacteristic_playlist = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // Playlist database insertion (fragmented)
    generalCharacteristic_playlist_db = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_DB,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Per-record metadata sync
    generalCharacteristic_playlist_sync = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_SYNC,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

    // Playlist record deletion
    generalCharacteristic_playlist_del = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_DEL,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // ── 5. Attach callback instances ──────────────────────────────────────
    generalCharacteristic_version->setCallbacks(new generalCallbacks_version());
    generalCharacteristic_select_mode->setCallbacks(new generalCallbacks_select_mode());
    generalCharacteristic_setState->setCallbacks(new generalCallbacks_set_state());
    generalCharacteristic_mechanism_speed->setCallbacks(new generalCallbacks_mechanism_speed());
    generalCharacteristic_sound_level->setCallbacks(new generalCallbacks_sound_level());
    generalCharacteristic_sculpture_to_home->setCallbacks(new generalCallbacks_sculpture_to_home());
    generalCharacteristic_jump_to_center->setCallbacks(new generalCallbacks_jump_to_center());
    generalCharacteristic_pause->setCallbacks(new generalCallbacks_pause());
    generalCharacteristic_sculpture_repeat->setCallbacks(new generalCallbacks_sculpture_repeat());
    generalCharacteristic_sculpture_timeToMove->setCallbacks(new generalCallbacks_sculpture_timeToMove());
    generalCharacteristic_automatic_playback->setCallbacks(new generalCallbacks_automatic_playback());
    generalCharacteristic_currentIndex->setCallbacks(new generalCallbacks_currentIndex());
    generalCharacteristic_offsetPieces->setCallbacks(new generalCallbacks_offsetPieces());
    generalCharacteristic_board_rotation->setCallbacks(new generalCallbacks_board_rotation());
    generalCharacteristic_calib_type->setCallbacks(new generalCallbacks_calib_type());
    generalCharacteristic_batteryInfo->setCallbacks(new generalCallbacks_batteryInfo());
    generalCharacteristic_factory_reset->setCallbacks(new generalCallbacks_factory_reset());

    generalCharacteristic_receive_movement->setCallbacks(new generalCallbacks_receive_movement());
    generalCharacteristic_status_board->setCallbacks(new generalCallbacks_status_board());
    generalCharacteristic_check_move->setCallbacks(new generalCallbacks_check_move());
    generalCharacteristic_single_move->setCallbacks(new generalCallbacks_single_move());
    generalCharacteristic_play_info->setCallbacks(new generalCallbacks_play_info());
    generalCharacteristic_takeback->setCallbacks(new generalCallbacks_takeback());
    generalCharacteristic_sound_check->setCallbacks(new generalCallbacks_sound_check());
    generalCharacteristic_voice->setCallbacks(new generalCallbacks_voice());
    generalCharacteristic_matrix_init_game->setCallbacks(new generalCallbacks_matrix_init_game());
    fileCharacteristic_receiveOta->setCallbacks(new FilesCallbacks_receiveOta());

    generalCharacteristic_playlist->setCallbacks(new generalCallbacks_playlist());
    generalCharacteristic_playlist_db->setCallbacks(new generalCallbacks_playlist_db());
    generalCharacteristic_playlist_sync->setCallbacks(new generalCallbacks_playlist_sync());
    generalCharacteristic_playlist_del->setCallbacks(new generalCallbacks_playlist_del());

    // ── 6. Start service and begin advertising ────────────────────────────
    pServiceGeneralConfig->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID5);
    pAdvertising->setScanResponse(true);
    // These preferred-interval hints help iOS resolve the "connection issues" quirk
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);

    // Negotiate tighter connection parameters for lower latency
    uint16_t latency = 0;
    uint16_t max_int = 0x20; // 0x20 × 1.25 ms = 40 ms
    uint16_t min_int = 0x10; // 0x10 × 1.25 ms = 20 ms
    uint16_t timeout = 800;  // 800 × 10 ms = 8 s supervision timeout
    pServer->updateConnParams(1, min_int, max_int, latency, timeout);

    BLEDevice::startAdvertising();
}
