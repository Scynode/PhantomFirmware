#pragma once
// #define BLUECOMMENTS
#include <Arduino.h>
#include "NimBLEDevice.h"

extern String globalConnect;

#define SERVICE_UUID5 "fd31a840-22e7-11eb-adc1-0242ac120002" // General service
//====General Config====

//#define GENERAL_UUID_NAME "7b204548-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_BATTERYINFO "7b204548-40c4-11eb-adc1-0242ac120002"
#define GENERAL_UUID_ERRORMSG "7b204d4a-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_RECEIVE_MOVEMENT "c60c786b-bf3f-49d8-bd9e-c268e0519a7b" // Send movement, sent in the following format "M f7-f5": the start coordinate and the end coordinate of the movement
#define GENERAL_UUID_STATUS_BOARD "06034924-77e8-433e-ac4c-27302e5e853f"     // When a piece movement is made on the board, that movement is sent through this characteristic
#define GENERAL_UUID_SEND_MATRIX "1b034927-77e8-433e-ac4c-27302e5e853f"
#define GENERAL_UUID_SEND_TESTMODE_ERROR "1b034928-77e8-433e-ac4c-27302e5e853f"
#define GENERAL_UUID_CHECK_MOVE "9cc3b57e-eee5-4d3e-8c1d-3fbd636d6780"

#define FILE_UUID_RECEIVE_OTA "93601602-bbc2-4e53-95bd-a3ba326bc04b" // The characteristic for OTA updates

#define GENERAL_UUID_SELECT_MODE "c08d3691-e60f-4467-b2d0-4a4b7c72777e"
#define UUID_PLAY_INFO "d7f0b4ea-9b52-11ee-b9d1-0242ac120002"
#define UUID_MECHANISM_SPEED "acb646cc-92ca-11ee-b9d1-0242ac120002"
#define UUID_SOUND_LEVEL "acb64a32-92ca-11ee-b9d1-0242ac120002"
#define UUID_JUMP_TO_CENTER "5e316147-4550-4cf3-8e2b-edc098312a43"
#define UUID_SCULPTURE_TO_HOME "acb64fb4-92ca-11ee-b9d1-0242ac120002"
#define UUID_BOARD_ROTATION "b5a650ea-92ca-11ee-b9d1-0242ac120002"
#define UUID_CALIB_TYPE "c43f07d7-a64a-4776-a35f-6190a53c1c86"

#define UUID_FACTORY_RESET "b583ff00-b77a-42f5-a53f-a9bf4c291d80"
#define UUID_OFFSETPIECES "acb650ea-92ca-11ee-b9d1-0242ac120002"


#define UUID_PAUSE "cc4cbbe0-9b5a-11ee-b9d1-0242ac120002"
#define UUID_SINGLE_MOVE "d9a6b488-1d61-423f-8713-f3b0eedc9904"
#define UUID_CURRENTINDEX "acb6532e-92ca-11ee-b9d1-0242ac120002"
#define UUID_REPEAT "aa4cbbe0-9b5a-11ee-b9d1-0242ac120002"
#define UUID_TIMETOMOVE "bb4cbbe0-9b5a-11ee-b9d1-0242ac120002"
#define UUID_AUTOMATIC_PLAYBACK "dd4cbbe0-9b5a-11ee-b9d1-0242ac120002"


#define UUID_SETSTATE "acb6543c-92ca-11ee-b9d1-0242ac120002" // it is used to handle the states next to mode
#define UUID_VERSION "392d9e66-937a-11ee-b9d1-0242ac120002"
#define UUID_DEVICE_MATRIX "a9ec59b8-1444-4785-aa45-edf4f752bb87"
#define UUID_MAKE_SOUND "59933695-1cf2-4df8-bd2f-529d6d444250"
#define UUID_VOICE "3e42feb6-7c91-4e17-a1ed-31b51840613f"
#define UUID_MATRIX_INIT_GAME "e00b41ea-e840-4793-90d9-6c682cac695d"

#define UUID_PLAYLIST "4f1c9720-939a-11ee-b9d1-0242ac120002"
#define UUID_PLAYLIST_DB "a00125d2-cf9e-494a-b834-6dad6360729c"
#define UUID_PLAYLIST_SYNC "ea41f202-d149-4a1d-80a7-09b4a613be7f"
#define UUID_PLAYLIST_DEL "855fce26-94df-4b3f-b5a8-735a85d220fe"

#define UUID_TAKEBACK "89185e7a-78ef-4bb0-b48f-c0f53f21fc1b"

class Bluetooth
{
private:
public:
    Bluetooth();

    

    static void factoryReset();

    static float getBatteryStatus();
    static int batteryCheck(int mode);
    static void batterySim();
    static String getSoundCheck();
    static int getSoundTutorial();
    static String verifNewCommandBluetooth();
    static void resetVerifNewCommandBluetooth();
    static void setStatus(String);
    static void sendMatrixToApp(String, bool[10][10], char[10][10]);
    static void sendTestModeError(String);
    static int getModeChess();
    static void setMode(int);
    static String getCheckMove();
    static void setCheckMove(String);

    //static int *getPlaylist();

    static void setState(String);
    static String getPlayInfo();
    static void setPlayInfo(String);
    static int getPauseInfo();
    static void setPauseInfo(int);
    static int getRepeat();
    static int getTimeToMove();
    static int getAutoPlayback();
    static int getSingleMove();
    //-------------------------------------------------
    void init();

    static double getMechanismSpeed();
    static int getSoundLevel();
    static int getpatternsToHome();
    static int getJumpToCenter();

   // static void setCurrentIndex(int _currentIndex);
    //static int getCurrentIndex();
    static int gameToPlay(int mode);
    //static void playNextGame(int);

    static float getOffsetPieces();
    static int getBoardRotation();
    static int getCalibType();

    static String getTakeback();
    //-------------------------------------------------

    static String getVoiceCommand();
    static void resetVoiceCommand();
    static String getMatrixInitGame();
    static void resetMatrixInitGame();
};

class Fragment {
public:
    Fragment();
    void processData(const std::string& data);
    std::string getBuffer() const;
    bool isComplete() const;
    void clear();
    void sendFragmentedData(BLECharacteristic *characteristic, const std::string& data);

private:
    std::string buffer;
    bool complete;
};
