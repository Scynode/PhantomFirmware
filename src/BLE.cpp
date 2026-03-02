#include "BLE.h"
#include "esp_ota_ops.h"
#include <config.h>
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"

float batteryStatus_ = -1;
float batteryInit = -1;
int maxBattery = -1;
int previousWallStatus = -1;

unsigned long initialTimeCharging = -1;
unsigned long initialTime = -1;
String globalConnect = "0";

// int playlistSize = 0;
// int lastGametoPlay = -1;
//--------------------------------------------------
String versionUpdate = "0.1.6";
int modeChess = 0;
String state = "Initializating";
String playlist = "";
String playlistDB = "";
String mechanismSpeed = "NOINIT";
String soundLevel = "NOINIT";
String soundCheck = "";
String soundTutorial = "-1";
String patternsToHome = "NOINIT";
String jumpToCenter = "NOINIT";
int globalPause = 0;

int sculptureRepeat = -1;
int timeToMove = -1;
int automaticPlayback = -1;

String currentIndex = "0";
String globalSingleMove = "-1";
String offsetPieces = "NOINIT";
String boardRotation = "NOINIT";
int calibType = 0;
String batteryStatustoApp = "-";
//--------------------------------------------------

int MAX_CHARACTERS_BTNAME = 20;
String currentBluetoothMessage = "";
String newBluetoothMessage = "";
String globalCheckMovement = "0";
String globalPlayInfo = "";
String globalVoice = "";
String globalMatrixInitGame = "";

String globalTakeback = "";

String pgnStr = "";

void resetPlaylist();

BLECharacteristic *generalCharacteristic_version;
BLECharacteristic *generalCharacteristic_select_mode;
BLECharacteristic *generalCharacteristic_setState;
BLECharacteristic *generalCharacteristic_mechanism_speed;
BLECharacteristic *generalCharacteristic_sound_level;
BLECharacteristic *generalCharacteristic_sound_check;
BLECharacteristic *generalCharacteristic_sculpture_to_home;
BLECharacteristic *generalCharacteristic_jump_to_center;

BLECharacteristic *generalCharacteristic_pause;
BLECharacteristic *generalCharacteristic_currentIndex;
BLECharacteristic *generalCharacteristic_single_move;
BLECharacteristic *generalCharacteristic_sculpture_repeat;
BLECharacteristic *generalCharacteristic_sculpture_timeToMove;
BLECharacteristic *generalCharacteristic_automatic_playback;

BLECharacteristic *generalCharacteristic_offsetPieces;
BLECharacteristic *generalCharacteristic_board_rotation;
BLECharacteristic *generalCharacteristic_calib_type;
BLECharacteristic *generalCharacteristic_batteryInfo;
BLECharacteristic *generalCharacteristic_factory_reset;

BLECharacteristic *generalCharacteristic_errorMsg;
BLECharacteristic *generalCharacteristic_receive_movement;
BLECharacteristic *generalCharacteristic_status_board;
BLECharacteristic *generalCharacteristic_send_matrix;
BLECharacteristic *generalCharacteristic_send_testMode_Error;
BLECharacteristic *generalCharacteristic_check_move;
BLECharacteristic *generalCharacteristic_play_info;
BLECharacteristic *generalCharacteristic_takeback;

BLECharacteristic *generalCharacteristic_voice;
BLECharacteristic *generalCharacteristic_matrix_init_game;
BLECharacteristic *fileCharacteristic_receiveOta; // OTA update characteristic
// BLECharacteristic *fileCharacteristic_notification_matrix;

BLECharacteristic *generalCharacteristic_playlist;
BLECharacteristic *generalCharacteristic_playlist_db;
BLECharacteristic *generalCharacteristic_playlist_sync;
BLECharacteristic *generalCharacteristic_playlist_del;

class bleServerCallback : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        // Bluetooth::setName(vectSensorString);
        globalConnect = "1";
        Serial.println("-----Bluetooth Device Connected");
    }

    void onDisconnect(BLEServer *pServer)
    {
        globalConnect = "0";
        Serial.println("-----Blueooth Device Disconnected");
    }
};

class generalCallbacks_version : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Version from BLE: " + versionUpdate);
        characteristic->setValue(versionUpdate);
        characteristic->notify();
    }
};

class generalCallbacks_select_mode : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        modeChess = atoi(rxData.c_str()); // Use atoi to convert to integer
        // Serial.println("Mode from BLE: " + String(modeChess));
        preferences.begin("myApp", false);
        preferences.putInt("modeGameplay", modeChess);
        preferences.end();
    }
    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_set_state : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
    }
    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_mechanism_speed : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        mechanismSpeed = rxData.c_str();
        // Serial.println("Speed onWrite: " + mechanismSpeed);
        preferences.begin("myApp", false);
        preferences.putString("mechanismSpeed", mechanismSpeed);
        preferences.end();
    }
    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Speed from BLE: " + mechanismSpeed);
        characteristic->setValue(mechanismSpeed);
        characteristic->notify();
    }
};

class generalCallbacks_sound_level : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        soundLevel = rxData.c_str();
        // Serial.println("Sound level onWrite: " + soundLevel);
        preferences.begin("myApp", false);
        preferences.putString("soundLevel", soundLevel);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Sound Level from BLE: " + soundLevel);
        characteristic->setValue(soundLevel);
        characteristic->notify();
    }
};

class generalCallbacks_sound_check : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        soundCheck = rxData.c_str();
        // Serial.println("Make Sound onWrite: " + soundCheck);
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_sculpture_to_home : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        patternsToHome = rxData.c_str();
        // Serial.println("Sculpture to home: " + patternsToHome);
        preferences.begin("myApp", false);
        preferences.putString("patternsToHome", patternsToHome);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Sculpture to Home from BLE: " + patternsToHome);
        characteristic->setValue(patternsToHome);
        characteristic->notify();
    }
};

class generalCallbacks_jump_to_center : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        jumpToCenter = rxData.c_str();
        // Serial.print("Jump to center onWrite: " + jumpToCenter);
        preferences.begin("myApp", false);
        preferences.putString("jumpToCenter", jumpToCenter);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Jump to Center from BLE: " + jumpToCenter);
        characteristic->setValue(jumpToCenter);
        characteristic->notify();
    }
};

class generalCallbacks_pause : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        globalPause = std::stoi(rxData);
        Serial.println("Pause onWrite: " + String(globalPause));
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_currentIndex : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        currentIndex = rxData.c_str();
        Serial.println("Next data onWrite: " + currentIndex);
        preferences.begin("myApp", false);
        preferences.putString("lastFile", currentIndex);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.println("Reading data from BLE: " + currentIndex);
        characteristic->setValue(currentIndex);
        characteristic->notify();
    }
};

class generalCallbacks_sculpture_repeat : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        sculptureRepeat = std::stoi(rxData);
        Serial.printf("Sculpture Repeat onWrite: %d\n", sculptureRepeat);
        preferences.begin("myApp", false);
        preferences.putInt("sculptureRepeat", sculptureRepeat);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.printf("Read Sculpture Repeat from BLE: %d\n", sculptureRepeat);
        String sculptureRepeatStr = String(sculptureRepeat);
        characteristic->setValue(sculptureRepeatStr);
        characteristic->notify();
    }
};

class generalCallbacks_sculpture_timeToMove : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        timeToMove = std::stoi(rxData);
        Serial.printf("Time to Move onWrite: %d\n", timeToMove);
        preferences.begin("myApp", false);
        preferences.putInt("timeToMove", timeToMove);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.printf("Read Time to Move from BLE: %d\n", timeToMove);
        String timeToMoveStr = String(timeToMove);
        characteristic->setValue(timeToMoveStr);
        characteristic->notify();
    }
};

class generalCallbacks_automatic_playback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        automaticPlayback = std::stoi(rxData);
        Serial.println("Automatic Playback onWrite: " + String(automaticPlayback));
        preferences.begin("myApp", false);
        preferences.putInt("autoPlayback", automaticPlayback);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.println("Read Automatic Playback from BLE: " + String(automaticPlayback));
        String autoPlaybackStr = String(automaticPlayback);
        characteristic->setValue(autoPlaybackStr);
        characteristic->notify();
    }
};

class generalCallbacks_single_move : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        globalSingleMove = rxData.c_str();
        // Serial.println("Single Move onWrite: " + globalSingleMove);
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_offsetPieces : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        offsetPieces = rxData.c_str();
        // Serial.print("OffsetPieces onWrite: " + offsetPieces);
        preferences.begin("myApp", false);
        preferences.putString("offsetPieces", offsetPieces);
        preferences.end();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Offset Pieces from BLE: " + offsetPieces);
        characteristic->setValue(offsetPieces);
        characteristic->notify();
    }
};

class generalCallbacks_board_rotation : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)

    {
        std::string rxData = characteristic->getValue();
        boardRotation = rxData.c_str();
        // Serial.print("Board Rotation To: " + boardRotation);
        preferences.begin("myApp", false);
        preferences.putInt("rotation", boardRotation.toInt());
        preferences.end();
        delay(1000);
        esp_restart();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        // Serial.println("Read Board Rotation from BLE: " + boardRotation);
        characteristic->setValue(boardRotation);
        characteristic->notify();
    }
};

class generalCallbacks_calib_type : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        calibType = std::stoi(rxData);
        Serial.print("Calib Type onWrite: " + calibType);
        preferences.begin("myApp", false);
        preferences.putInt("calibType", calibType);
        preferences.putInt("stallOld", 40);
        preferences.putInt("stallOlder", 40);
        preferences.putInt("stallOldD2", 40);
        preferences.putInt("stallOlderD2", 40);
        preferences.putInt("betterCalib", 1);
        preferences.end();
        delay(1000);
        esp_restart();
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.println("Read Calib Type from BLE: " + calibType);
        characteristic->setValue(String(calibType));
        characteristic->notify();
    }
};

class generalCallbacks_batteryInfo : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
    }

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.println("BatteryStatus: " + String(batteryStatus_));
        Serial.println("BatteryToBLE: " + batteryStatustoApp);
        generalCharacteristic_batteryInfo->setValue(batteryStatustoApp);
        generalCharacteristic_batteryInfo->notify();
    }
};

class generalCallbacks_factory_reset : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        // Serial.print("Factory Reset");
        Bluetooth::factoryReset();
        delay(1000);
        esp_restart();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class generalCallbacks_receive_movement : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
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

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_status_board : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
    }
    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class FilesCallbacks_receiveOta : public BLECharacteristicCallbacks
{
public:
    void onWrite(BLECharacteristic *characteristic)
    {
        if (localCharacteristic == NULL)
        {
            localCharacteristic = characteristic;
        }

        if (!updateFlag)
        {
            preferences.begin("myApp", false);
            int updatingStatus = preferences.getInt("updatingStatus", -1);
            preferences.end();

            if (updatingStatus == -1) // First time OTA, suspend all the operations and restart to enter only OTA.
            {
                preferences.begin("myApp", false);
                preferences.putInt("updatingStatus", 0);
                preferences.end();
                esp_restart();
            }
            initializeOTA();
        }
        else
        {
            writeOTAData();
        }
    }

    void onRead(BLECharacteristic *characteristic)
    {
        uint8_t send = partitionReady ? 0x10 : 0x00;
        characteristic->setValue(&send, sizeof(send));
    }

private:
    BLECharacteristic *localCharacteristic;

    void initializeOTA()
    {
        Serial.println("Initializing OTA");
        esp_task_wdt_init(15, 0);
        delay(100);

        std::string rxData = localCharacteristic->getValue();
        if (rxData.length() < 4)
        {
            notifyError(1, "Invalid initial packet");
            return;
        }

        totalSize = *reinterpret_cast<const uint32_t *>(rxData.c_str());
        Serial.printf("Total size: %u bytes\n", totalSize);

        update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition == NULL)
        {
            notifyError(2, "No update partition found");
            return;
        }

        esp_err_t error = esp_ota_begin(update_partition, totalSize, &otaHandler);
        if (error != ESP_OK)
        {
            notifyError(3, "esp_ota_begin failed");
            return;
        }

        updateFlag = true;
        receivedSize = 0;

        Serial.println("OTA initialized");
        partitionReady = true;
    }

    void writeOTAData()
    {
        std::string rxData = localCharacteristic->getValue();
        esp_err_t writeResult = esp_ota_write(otaHandler, rxData.c_str(), rxData.length());
        if (writeResult != ESP_OK)
        {
            notifyError(4, "esp_ota_write failed");
            return;
        }

        receivedSize += rxData.length();
        Serial.printf("Received: %u / %u bytes\n", receivedSize, totalSize);

        if (receivedSize >= totalSize)
        {
            finalizeOTA();
        }
        else
        {
            notifySuccess();
        }
    }

    void finalizeOTA()
    {
        esp_err_t endResult = esp_ota_end(otaHandler);
        if (endResult != ESP_OK)
        {
            notifyError(5, "esp_ota_end failed");
            return;
        }

        if (ESP_OK == esp_ota_set_boot_partition(update_partition))
        {
            Serial.println("OTA successful, restarting...");
            preferences.begin("myApp", false);
            preferences.putInt("updatingStatus", -1);
            preferences.end();
            notifyFinish();
            delay(3000);
            esp_restart();
        }
        else
        {
            notifyError(6, "Unable to set boot partition");
        }
    }

    void notifyError(uint8_t errorCode, const char *message)
    {
        Serial.printf("OTA Error: %s (Code: %d)\n", message, errorCode);
        updateFlag = false;
        if (otaHandler)
        {
            esp_ota_end(otaHandler);
            otaHandler = 0;
        }

        if (update_partition != NULL)
        {
            esp_partition_erase_range(update_partition, 0, update_partition->size);
        }

        preferences.begin("myApp", false);
        preferences.putInt("updatingStatus", -1);
        preferences.end();

        notify(0xFF, errorCode);

        delay(3000);
        esp_restart();
    }

    void notifySuccess()
    {
        notify(0x01, 0);
    }

    void notifyFinish()
    {
        notify(0x02, 0);
    }

    void notify(uint8_t status, uint8_t code)
    {
        uint8_t response[2] = {status, code};
        localCharacteristic->setValue(response, 2);
        localCharacteristic->notify();
    }

    bool updateFlag = false;
    bool partitionReady = false;
    uint32_t totalSize = 0;
    uint32_t receivedSize = 0;
    const esp_partition_t *update_partition = NULL;
    esp_ota_handle_t otaHandler = 0;
};

class generalCallbacks_check_move : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
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

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_play_info : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String playinfoStr = rxData.c_str();
        Serial.print("Play Info: ");
        Serial.println(playinfoStr);
        globalPlayInfo = playinfoStr;
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_takeback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String takebackStr = rxData.c_str();
        Serial.print("Received Takeback: ");
        Serial.println(takebackStr);
        globalTakeback = takebackStr;

        // Split by comma first
        int commaIndex = takebackStr.indexOf(',');
        if (commaIndex != -1)
        {
            String takebackMoves = takebackStr.substring(0, commaIndex);
            String newFen = takebackStr.substring(commaIndex + 1);

            // Split moves by space
            int spaceIndex = takebackMoves.indexOf(' ');
            if (spaceIndex != -1)
            {
                // Two moves
                String firstMove = takebackMoves.substring(0, spaceIndex);
                String secondMove = takebackMoves.substring(spaceIndex + 1);
                Serial.print("First Move: ");
                Serial.println(firstMove);
                Serial.print("Second Move: ");
                Serial.println(secondMove);
            }
            else
            {
                // Single move
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
};

class generalCallbacks_voice : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String voice = rxData.c_str();
        globalVoice = voice;
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

class generalCallbacks_matrix_init_game : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String matrixInitGame = rxData.c_str();
        globalMatrixInitGame = matrixInitGame;
        Serial.println("Matrix init game: ");
        Serial.println(globalMatrixInitGame);
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

// count,list
class generalCallbacks_playlist : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
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

    void onRead(BLECharacteristic *characteristic)
    {
        Serial.printf("Playlist read: %s\n", playlist.c_str());
        fragment.sendFragmentedData(characteristic, playlist.c_str());
    }

private:
    void processData(String data)
    {
        int split = data.indexOf(",");
        int count = data.substring(0, split).toInt();
        playlist = data.substring(split + 1);

        preferences.begin("PL_setting");
        preferences.putString("playlist", playlist);
        preferences.putInt("playlistCount", count);
        preferences.end();

        Serial.println("Playlist received: ");
        Serial.println(playlist.c_str());
    }

    Fragment fragment;
};

// event|||date|||white|||black|||result
class generalCallbacks_playlist_db : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
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

    void onRead(BLECharacteristic *characteristic)
    {
        characteristic->setValue(playlistDB);
        characteristic->notify();
    }

private:
    void processPgnData(const std::string &completeData)
    { // separate
        size_t pos = completeData.find("&&&");
        String metadata = completeData.substr(0, pos).c_str();
        String pgn = completeData.substr(pos + 3).c_str();

        preferences.begin("PL_setting");
        // pgnIndex save the next data
        int data = preferences.getInt("pgnIndex", 201);
        int count = preferences.getInt("pgnCount", 0);
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

        if (playlistDB == "")
        {
            playlistDB = String(data);
        }
        else
        {
            playlistDB = playlistDB + "-" + String(data);
        }
        preferences.begin("PL_setting");
        preferences.putString("playlistDB", playlistDB);
        preferences.end();
    }

    Fragment fragment;
};

// data|||event|||date|||white|||black|||result
class generalCallbacks_playlist_sync : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
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

    void onRead(BLECharacteristic *characteristic)
    {
    }
};

// del from playlistDB
// count,list,index
class generalCallbacks_playlist_del : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String data = rxData.c_str();

        int split = data.indexOf(",");
        if (split == -1)
        {
            resetPlaylist();
            Serial.println("Playlist reset.");
            return;
        }

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
                data = data.substring(split + 1);
            }
        }

        int count = parts[0].toInt();
        String list = parts[1];
        String index = parts[2];

        updatePlaylistDB(count, list, index);
        Serial.printf("Playlist DB deleted, list: %s\n", list.c_str());
    }

    void onRead(BLECharacteristic *characteristic)
    {
    }

private:
    void updatePlaylistDB(const int count, const String &list, const String index)
    {
        preferences.begin("PL_setting");
        preferences.putString("playlistDB", list);
        preferences.putInt("pgnCount", count);
        preferences.end();

        preferences.begin("PL_metadata");
        preferences.remove(index.c_str());
        preferences.end();

        preferences.begin("PL_pgn");
        preferences.remove(index.c_str());
        preferences.end();

        playlistDB = list;
    }
};

Bluetooth::Bluetooth()
{
}

void Bluetooth::batterySim()
{
    float dischargeRateOnHold = 0.00001;
    float dischargeRateMoving = 0.00002;
    float chargeRate = 0.00005;
    int charging = Bluetooth::batteryCheck(1);
    int doneCharging = Bluetooth::batteryCheck(2);
    int wallStatus = Bluetooth::batteryCheck(3);

    // Serial.println("============================SimBattery: " + String(batteryStatus_));

    if (previousWallStatus && !wallStatus) // Disconnected from power
    {
        initialTime = millis();
        timeMoving = 0;
        previousWallStatus = wallStatus;
        batteryInit = batteryStatus_;
        // Serial.println("--------------------Disconnected from power");
    }
    else if (!previousWallStatus && wallStatus) // Connected to power
    {
        initialTimeCharging = millis();
        previousWallStatus = wallStatus;
        batteryInit = batteryStatus_;
        // Serial.println("--------------------Connected to power");
    }

    unsigned long totalTimeOn = millis() - initialTime;
    if (timeMoving > totalTimeOn) // If it was moving when disconnected from power, discard that time so it does not accumulate
    {
        //  Serial.println("======================Correction triggered");
        timeMoving = 0;
    }
    unsigned long timeOnHold = totalTimeOn - timeMoving;
    unsigned long timeCharging = millis() - initialTimeCharging;

    if (wallStatus) // Connected to power
    {
        if (charging)
        {
            // Serial.println("--------------------Charging time: " + String(timeCharging));
            batteryStatus_ = batteryInit + timeCharging * chargeRate;
        }
        if (doneCharging)
        {
            batteryStatus_ = maxBattery;
            batteryInit = maxBattery;
            //  Serial.println("--------------------Done Charging: " + String(batteryStatus_));
        }
    }
    else // Not connected to power
    {
        // Serial.printf("--------------------Total time: %lu Time Moving: %lu time OnHold: %lu \n", totalTimeOn, timeMoving, timeOnHold);
        batteryStatus_ = batteryInit - timeOnHold * dischargeRateOnHold - timeMoving * dischargeRateMoving;
    }

    // Serial.println("============================SimBatteryPOST: " + String(batteryStatus_));

    int batteryPercent = map(batteryStatus_, 2500, maxBattery, 0, 100);

    batteryStatustoApp = String(batteryPercent) + "," + String(wallStatus) + "," + String(charging) + "," + String(doneCharging);

    // Serial.println("Battery Info to BLE: " + batteryStatustoApp);
    // Serial.println();
}

void Bluetooth::init()
{
    // Initialize battery before anything else
    batteryInit = batteryCheck(0);
    pinMode(4, INPUT);
    int doneCharging = batteryCheck(2);
    int wallStatus = batteryCheck(3);

    initialTime = millis();
    previousWallStatus = wallStatus;

    if (batteryInit == -1) // NO BATTERY
    {
        Serial.println("--------------------NO BATTERY");
        batteryStatustoApp = "-1";
        batteryStatus_ = -1;
    }
    else
    {
        batteryInit = (wallStatus) ? batteryInit * 0.95 : batteryInit; // at 3061 this is 95%

        if (wallStatus && doneCharging && batteryInit > 2800 && batteryInit < 3500)
        {
            preferences.begin("myApp", false);
            preferences.putFloat("maxBattery", batteryInit);
            preferences.end();
            Serial.println("-----Changing MaxBattery: " + String(batteryInit));
        }
        if (batteryInit == 4095) // Read error
        {
            Serial.println("Battery Read Error");
            batteryInit = (doneCharging) ? maxBattery : 2895;
        }
        if (wallStatus)
        {
            initialTimeCharging = millis();
        }

        preferences.begin("myApp", true);
        maxBattery = preferences.getFloat("maxBattery", 3200);
        preferences.end();
        Serial.println("--------------------Max Battery: " + String(maxBattery));
        Serial.println("--------------------Initial Battery: " + String(batteryInit));
        batteryStatus_ = batteryInit;
    }
    Serial.printf("Battery Status: %f doneCharging: %d wallStatus: %d\n", batteryStatus_, doneCharging, wallStatus);

    // Load all initial settings from memory.
    preferences.begin("myApp", true);
    String randomPhantom = "Phantom " + String(preferences.getInt("phantomName", -1));
    modeChess = preferences.getInt("modeGameplay", -1);
    sculptureRepeat = preferences.getInt("sculptureRepeat", 2);
    timeToMove = preferences.getInt("timeToMove", 0);
    automaticPlayback = preferences.getInt("autoPlayback", 1);
    mechanismSpeed = preferences.getString("mechanismSpeed", "empty");
    soundLevel = preferences.getString("soundLevel", "empty");
    patternsToHome = preferences.getString("patternsToHome", "emtpy");
    jumpToCenter = preferences.getString("jumpToCenter", "empty");
    currentIndex = preferences.getString("lastFile", "0");
    offsetPieces = preferences.getString("offsetPieces", "empty");
    boardRotation = preferences.getInt("rotation", 0);
    calibType = preferences.getInt("calibType", 0);
    preferences.end();

    preferences.begin("PL_setting");
    playlist = preferences.getString("playlist", "1-2-3");
    playlistDB = preferences.getString("playlistDB", "");
    preferences.end();

    BLEDevice::init(randomPhantom.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pServiceGeneralConfig = pServer->createService(BLEUUID(SERVICE_UUID5));
    pServer->setCallbacks(new bleServerCallback());

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////

    generalCharacteristic_version = pServiceGeneralConfig->createCharacteristic(
        UUID_VERSION,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_select_mode = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SELECT_MODE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_setState = pServiceGeneralConfig->createCharacteristic(
        UUID_SETSTATE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_mechanism_speed = pServiceGeneralConfig->createCharacteristic(
        UUID_MECHANISM_SPEED,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_sound_level = pServiceGeneralConfig->createCharacteristic(
        UUID_SOUND_LEVEL,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_sculpture_to_home = pServiceGeneralConfig->createCharacteristic(
        UUID_SCULPTURE_TO_HOME,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_jump_to_center = pServiceGeneralConfig->createCharacteristic(
        UUID_JUMP_TO_CENTER,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_pause = pServiceGeneralConfig->createCharacteristic(
        UUID_PAUSE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_sculpture_repeat = pServiceGeneralConfig->createCharacteristic(
        UUID_REPEAT,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_sculpture_timeToMove = pServiceGeneralConfig->createCharacteristic(
        UUID_TIMETOMOVE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_automatic_playback = pServiceGeneralConfig->createCharacteristic(
        UUID_AUTOMATIC_PLAYBACK,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_currentIndex = pServiceGeneralConfig->createCharacteristic(
        UUID_CURRENTINDEX,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_offsetPieces = pServiceGeneralConfig->createCharacteristic(
        UUID_OFFSETPIECES,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_board_rotation = pServiceGeneralConfig->createCharacteristic(
        UUID_BOARD_ROTATION,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_calib_type = pServiceGeneralConfig->createCharacteristic(
        UUID_CALIB_TYPE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_batteryInfo = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_BATTERYINFO,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_factory_reset = pServiceGeneralConfig->createCharacteristic(
        UUID_FACTORY_RESET,
        NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    generalCharacteristic_check_move = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_CHECK_MOVE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_errorMsg = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_ERRORMSG,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_receive_movement = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_RECEIVE_MOVEMENT,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_status_board = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_STATUS_BOARD,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_send_matrix = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SEND_MATRIX,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_send_testMode_Error = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SEND_TESTMODE_ERROR,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_single_move = pServiceGeneralConfig->createCharacteristic(
        UUID_SINGLE_MOVE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_play_info = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAY_INFO,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_takeback = pServiceGeneralConfig->createCharacteristic(
        UUID_TAKEBACK,
        NIMBLE_PROPERTY::WRITE);

    fileCharacteristic_receiveOta = pServiceGeneralConfig->createCharacteristic(
        FILE_UUID_RECEIVE_OTA,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_sound_check = pServiceGeneralConfig->createCharacteristic(
        UUID_MAKE_SOUND,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_voice = pServiceGeneralConfig->createCharacteristic(
        UUID_VOICE,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_matrix_init_game = pServiceGeneralConfig->createCharacteristic(
        UUID_MATRIX_INIT_GAME,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE);

    generalCharacteristic_playlist = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_playlist_db = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_DB,
        NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY);

    generalCharacteristic_playlist_sync = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_SYNC,
        NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

    generalCharacteristic_playlist_del = pServiceGeneralConfig->createCharacteristic(
        UUID_PLAYLIST_DEL,
        NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

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

    //  Start Services
    pServiceGeneralConfig->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID5);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);

    uint16_t latency = 0;
    uint16_t max_int = 0x20; // max_int = 0x20 * 1.25ms = 32 * 1.25ms = 40ms
    uint16_t min_int = 0x10; // min_int = 0x10 * 1.25ms = 16 * 1.25ms = 20ms
    uint16_t timeout = 800;  // timeout = 800 * 10ms = 8000ms (8 seconds)
    pServer->updateConnParams(1, min_int, max_int, latency, timeout);
    BLEDevice::startAdvertising();
}

String Bluetooth::verifNewCommandBluetooth()
{
    String resultMessage = "";
    if (currentBluetoothMessage != newBluetoothMessage)
    {
        resultMessage = newBluetoothMessage;
        currentBluetoothMessage = newBluetoothMessage;
    }
    else
    {
        resultMessage = "";
    }
    return resultMessage;
}

void Bluetooth::resetVerifNewCommandBluetooth()
{
    currentBluetoothMessage = "";
    newBluetoothMessage = "";
}

void Bluetooth::setStatus(String status)
{
    generalCharacteristic_status_board->setValue(status); // Updated for the NimBLE library
    generalCharacteristic_status_board->notify();
    Serial.println("Physical move to App: " + status);
}

void Bluetooth::sendMatrixToApp(String error, bool sensors[10][10], char matrixToSend[10][10])
{
    String strSensors = "";
    String strMatriz = "";
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            strSensors += sensors[i][j];
            strMatriz += matrixToSend[i][j];
        }
    }
    generalCharacteristic_send_matrix->setValue(error + "," + strMatriz + "," + strSensors); // Updated for the NimBLE library
    generalCharacteristic_send_matrix->notify();
}

void Bluetooth::sendTestModeError(String error)
{
    generalCharacteristic_send_testMode_Error->setValue(error); // Updated for the NimBLE library
    generalCharacteristic_send_testMode_Error->notify();
    // Serial.println("To Ble: " + error);
}

String Bluetooth::getVoiceCommand()
{
    return globalVoice;
}

void Bluetooth::resetVoiceCommand()
{
    globalVoice = "";
}

String Bluetooth::getMatrixInitGame()
{
    return globalMatrixInitGame;
}

void Bluetooth::resetMatrixInitGame()
{
    globalMatrixInitGame = "";
}

String Bluetooth::getCheckMove()
{
    return globalCheckMovement;
}

String Bluetooth::getPlayInfo() // To send the initial value of the play info
{
    return globalPlayInfo;
}

void Bluetooth::setPlayInfo(String _playInfo)
{
    globalPlayInfo = "";
    generalCharacteristic_play_info->setValue(_playInfo);
}

void Bluetooth::setCheckMove(String result)
{
    globalCheckMovement = result;
    // generalCharacteristic_check_move->setValue(result);
}
//-----------------------------------------------------------------------------------------
int Bluetooth::getModeChess()
{
    // Serial.println("Mode Chess from BLE: " + String(modeChess));
    return modeChess;
}

void Bluetooth::setMode(int _mode)
{
    // Serial.println("Set Mode from Board: " + String(_mode));
    modeChess = _mode;
    preferences.begin("myApp", false);
    preferences.putInt("modeGameplay", modeChess);
    preferences.end();
    // Serial.println("Set Mode from Board to BLE: " + String(modeChess));
    generalCharacteristic_select_mode->setValue(String(modeChess));
    generalCharacteristic_select_mode->notify();
}

void Bluetooth::setState(String stateLocal)
{
    state = stateLocal;
    // Serial.println("Set State from Board to BLE: " + state);
    generalCharacteristic_setState->setValue(state);
    generalCharacteristic_setState->notify();
}

int Bluetooth::gameToPlay(int mode)
{
    int currentIndexLocal = 0;
    int lastGametoPlay = -1;
    int playlistSize = 0;
    int gameToPlay = 0;

    if (playlist.length() > 0) // If there is a playlist and an index
    {
        int *playlistIndices = new int[200];
        for (int i = 0; i < playlist.length(); i++) // Parse game indices from the string into an integer array
        {
            if (playlist.charAt(i) == '-')
            {
                playlistIndices[playlistSize++] = playlist.substring(currentIndexLocal, i).toInt();
                currentIndexLocal = i + 1;
            }
        }
        lastGametoPlay = playlist.substring(currentIndexLocal).toInt();
        playlistIndices[playlistSize++] = lastGametoPlay; // Add the last index
                                                          // playlistIndices[playlistSize++] = playlistData.substring(currentIndexLocal).toInt();; // Add the last index
                                                          // compare the current index to the playlist size; if out of range, reset to the beginning
        if (currentIndex.toInt() >= playlistSize)
        {
            Serial.println("Current Index out of range, setting to 0");
            currentIndex = "0";
            preferences.begin("myApp", false);
            preferences.putString("lastFile", currentIndex);
            preferences.end();
        }

        gameToPlay = playlistIndices[currentIndex.toInt()];
        /*         Serial.print("Playlist from BLE: ");
                for (int i = 0; i < playlistSize; i++)
                {
                    Serial.print(playlistIndices[i]);
                    Serial.print(" ");
                }
                Serial.println(); */
        delete[] playlistIndices;
        playlistIndices = nullptr;

        // Serial.println("Current Index: " + currentIndex);
        // Serial.println("Playlist String: " + playlist);
        // Serial.println("Game to Play from BLE: " + String(gameToPlay));
    }
    else // If there is no playlist or no index, stop and set paused state
    {
        return -1;
    }

    if (mode == 1)
    {
        // This is the natural index increment. Notes: if sculptureRepeat is 0 (normal mode), it stops playing when reaching the last game; if sculptureRepeat is 1, it plays the same game every time; if sculptureRepeat is 2, it replays the whole playlist
        if (sculptureRepeat == 0)
        {
            if (currentIndex.toInt() == playlistSize - 1)
            {
                Serial.print("NEXT GAME TO PLAY: STOP PLAYING, INDEX: ");
                currentIndex = "0";
                globalPause = 1;
                generalCharacteristic_pause->setValue("1");
                generalCharacteristic_pause->notify();
            }
            else
            {
                Serial.print("NEXT GAME TO PLAY: NORMAL COUNTER, INDEX: ");
                currentIndex = String(currentIndex.toInt() + 1);
            }
        }
        else if (sculptureRepeat == 1)
        {
            Serial.print("NEXT GAME TO PLAY: REPEAT SAME GAME, INDEX: ");
            if (currentIndex.toInt() == playlistSize - 1)
            {
                currentIndex = currentIndex;
            }
        }
        else if (sculptureRepeat == 2)
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
    return gameToPlay;
}

double Bluetooth::getMechanismSpeed()
{

    if (mechanismSpeed == "1")
    {
        return 50;
    }
    else if (mechanismSpeed == "2")
    {
        return 100;
    }
    else if (mechanismSpeed == "3")
    {
        return 200;
    }
    else
    {
        return 50;
    }
}

int Bluetooth::getSoundLevel()
{
    // Serial.println("Sound Level from BLE: " + soundLevel);
    return soundLevel.toInt();
}

String Bluetooth::getSoundCheck()
{
    Serial.println("GET Sound Check from BLE: " + soundCheck);
    if (soundCheck != "1" && soundCheck != "2")
    {
        soundCheck = "0";
    }
    String soundCheckLocal = soundCheck;
    soundCheck = "";
    return soundCheckLocal;
}

int Bluetooth::getSoundTutorial()
{
    int soundTuorialLocal = soundTutorial.toInt();
    Serial.println("Sound tutorial from BLE: " + soundTuorialLocal);
    soundTutorial = "-1";
    return soundTuorialLocal;
}

int Bluetooth::getpatternsToHome()
{
    // Serial.println("Patterns to Home from BLE: " + patternsToHome);
    return patternsToHome.toInt();
}

int Bluetooth::getJumpToCenter()
{
    // Serial.println("Jump to Center from BLE: " + jumpToCenter);
    return jumpToCenter.toInt();
}

int Bluetooth::getPauseInfo()
{
    return globalPause;
}

void Bluetooth::setPauseInfo(int globalPauseLocal)
{
    globalPause = globalPauseLocal;
    generalCharacteristic_pause->setValue(String(globalPause));
    generalCharacteristic_pause->notify();
}

int Bluetooth::getRepeat()
{
    Serial.println("Sculpture Repeat from BLE: " + String(sculptureRepeat));
    return sculptureRepeat;
}

int Bluetooth::getTimeToMove()
{
    // Serial.println("Time to Move from BLE: " + String(timeToMove));
    return timeToMove;
}

int Bluetooth::getAutoPlayback()
{
    return automaticPlayback;
}

int Bluetooth::getSingleMove()
{
    int localSingleMove = globalSingleMove.toInt();
    globalSingleMove = "-1";
    return localSingleMove;
}

float Bluetooth::getOffsetPieces()
{
    // clamp offsetPieces to the range 0–5
    if (offsetPieces.toInt() < 0)
    {
        offsetPieces = "0";
    }
    if (offsetPieces.toInt() > 5)
    {
        offsetPieces = "5";
    }
    // Serial.println("Offset Pieces from BLE: " + offsetPieces);
    return offsetPieces.toFloat();
}

int Bluetooth::getBoardRotation()
{
    if (modeChess == 4 || testModeSculpture)
    {
        return 0;
    }

    // Serial.println("Board Rotation from BLE: " + boardRotation);
    return boardRotation.toInt();
}

int Bluetooth::getCalibType()
{
    if (calibType < 0 || calibType > 2)
    {
        calibType = 0;
    }

    return calibType;
}

float Bluetooth::getBatteryStatus()
{
    return batteryStatus_;
}

void Bluetooth::factoryReset()
{
    String initvector_100 = "0-1-2-3-4-5-6-7-8-9-10-11-12-13-14-15-16-17-18-19-20-21-22-23-24-25-26-27-28-29-30-31-32-33-34-35-36-37-38-39-40-41-42-43-44-45-46-47-48-49-50-51-52-53-54-55-56-57-58-59-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-75-76-77-78-79-80-81-82-83-84-85-86-87-88-89-90-91-92-93-94-95-96-97-98-99";
    String initvector_200 = "100-101-102-103-104-105-106-107-108-109-110-111-112-113-114-115-116-117-118-119-120-121-122-123-124-125-126-127-128-129-130-131-132-133-134-135-136-137-138-139-140-141-142-143-144-145-146-147-148-149-150-151-152-153-154-155-156-157-158-159-160-161-162-163-164-165-166-167-168-169-170-171-172-173-174-175-176-177-178-179-180-181-182-183-184-185-186-187-188-189-190-191-192-193-194-195-196-197-198-199";
    preferences.begin("myApp", false);
    preferences.clear();
    preferences.putInt("testFlag", 0);
    preferences.putInt("sculptureRepeat", 2);
    preferences.putInt("timeToMove", 0);
    preferences.putInt("autoPlayback", 1);
    preferences.putInt("modeGameplay", 1);
    preferences.putString("mechanismSpeed", "2");
    preferences.putString("soundLevel", "2");
    preferences.putString("jumpToCenter", "0");
    preferences.putString("patternsToHome", "6");
    preferences.putString("lastFile", "150");
    preferences.putFloat("maxBattery", 3200);
    preferences.putString("offsetPieces", "3");
    preferences.putInt("updatingStatus", -1);
    preferences.putInt("rotation", 0); //  90,  180,  270
    preferences.putInt("betterCalib", 1);
    preferences.putInt("calibType", 0);
    preferences.putInt("stallOld", 40);
    preferences.putInt("stallOlder", 40);
    preferences.putInt("stallOldD2", 40);
    preferences.putInt("stallOlderD2", 40);
    randomSeed(analogRead(4));
    preferences.putInt("phantomName", random(9999));
    preferences.end();

    resetPlaylist();
}
// -----------------------------------------------------------------------------------------
int Bluetooth::batteryCheck(int mode)
{
    int analogValueVBAT = 0;
    int digitalcharge = 0;
    int muxDelay = 5;

    if (mode == 0) // ONLY ANALOGS
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, HIGH);
        delay(muxDelay);

        unsigned long timer = millis();
        int previousDigitalCharge = analogRead(4);

        int cambios = 0;
        while (millis() - timer < 250)
        {
            digitalcharge = analogRead(4);
            if (abs(digitalcharge - previousDigitalCharge) > 2000)
            {
                cambios++;
            }
            previousDigitalCharge = digitalcharge;
        }
        if (cambios > 10) // battery might be diconnected
        {
            Serial.println("=================================Battery Disconnected");
            return -1;
        }
        else
        {
            digitalWrite(batS2, LOW);
            digitalWrite(batS1, HIGH);
            digitalWrite(batS0, HIGH);
            delay(muxDelay);

            timer = millis();
            int counter = 0;
            while (millis() - timer < 250)
            {
                analogValueVBAT = analogValueVBAT + analogRead(4);
                counter++;
            }

            analogValueVBAT = analogValueVBAT / counter;
            Serial.printf("=================================RAW VBAT: %d \n", analogValueVBAT);
            return analogValueVBAT;
        }
    }
    else if (mode == 1) // charging?
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, HIGH);
        delayMicroseconds(1);
        return !digitalRead(4);
    }
    else if (mode == 2) // Done?
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, HIGH);
        digitalWrite(batS0, LOW);
        delayMicroseconds(1);
        return !digitalRead(4);
    }
    else if (mode == 3) // Wall?
    {
        digitalWrite(batS2, LOW);
        digitalWrite(batS1, LOW);
        digitalWrite(batS0, LOW);
        delayMicroseconds(1);
        return digitalRead(4);
    }
    return -1;
}

String Bluetooth::getTakeback()
{
    String takebackLocal = globalTakeback;
    globalTakeback = "";
    return takebackLocal;
}

void resetPlaylist()
{
    preferences.begin("PL_setting");
    preferences.putString("playlist", "1-2-3");
    preferences.putString("playlistDB", "");
    preferences.putInt("pgnIndex", 201);
    preferences.putInt("pgnCount", 0);
    preferences.putInt("playlistCount", 0);
    preferences.end();

    preferences.begin("PL_metadata");
    preferences.clear();
    preferences.end();

    preferences.begin("PL_pgn");
    preferences.clear();
    preferences.end();

    playlist = "";
    playlistDB = "";
}

Fragment::Fragment() : buffer(""), complete(false) {}

void Fragment::processData(const std::string &data)
{
    if (data.find("START") == 0)
    {
        Serial.println("Start of data received.");
        buffer.clear();
        buffer += data.substr(5);
        complete = false;
    }
    else if (data.find("END") != std::string::npos)
    {
        buffer += data.substr(0, data.find("END"));
        Serial.println("End of data received.");
        Serial.println("Complete data: ");
        Serial.println(buffer.c_str());
        complete = true;
    }
    else
    {
        buffer += data;
    }
}

std::string Fragment::getBuffer() const
{
    return buffer;
}

bool Fragment::isComplete() const
{
    return complete;
}

void Fragment::clear()
{
    buffer.clear();
    complete = false;
}

void Fragment::sendFragmentedData(BLECharacteristic *characteristic, const std::string &data)
{
    const size_t chunkSize = 100;
    size_t dataLength = data.length();
    size_t offset = 0;

    characteristic->setValue("START");
    characteristic->notify();
    delay(50);

    while (offset < dataLength)
    {
        size_t end = offset + chunkSize;
        if (end > dataLength)
        {
            end = dataLength;
        }
        std::string chunk = data.substr(offset, end - offset);
        characteristic->setValue(chunk);
        characteristic->notify();
        offset = end;
        delay(50);
    }

    characteristic->setValue("END");
    characteristic->notify();
    delay(50);
}
