// #include <AccelStepper.h>
// #include <CircularBuffer.h>
// #include <MultiStepper.h>
#include <esp_system.h>
#include <config.h>
#include <math.h>
#include <stdio.h>
// #include <EEPROM.h>
// #include <Servo.h>
#include <Wire.h>
#include "BLE.h"
#include "scultpureMode.h"
#include <CuteBuzzerSounds.h>

#include "soc/rtc_wdt.h"
//===========================================
Bluetooth BleChess;
Preferences preferences;
SemaphoreHandle_t xSemaphore;
//===========================================

using namespace std;

#include "SPIFFS.h"
#include "FS.h"
#include <vector>

bool testModeSculpture = false;
unsigned long timeMoving = 0;
char matrix[8][8];
bool sensorMatrix[10][10]; // Used globlally to store the values of the sensors

int mode = 1;
int testFlag = 0;

void sculptureMode();
void playMode();
void testMode();
void pauseMode();

bool previousChessState[10][10] = {0};
int iniChangeX = 0;
int iniChangeY = 0;
int finChangeX = 0;
int finChangeY = 0;

int totalDeadPieces = 0;
int pieceSensorCount = 0;
int deadPieceSensorCount = 0;
int prevDeadPiecesCount = 0;

String detectChangePlus(char[10][10], int &);
void soundEndGame();
void stringToMatrix(String, char[10][10]);
void automaticMechanicMovement(String, char[10][10]);
float euclideanDistance(float, float, float, float);
void nearestCenter(float, float, float &, float &, float &, float &);

void bleTask(void *parameter)
{
    BleChess.init();
    xSemaphoreGive(xSemaphore);
    while (true)
    {
        if (BleChess.getBatteryStatus() != -1)
        {
            BleChess.batterySim();
        }
        updateSensors();
        vTaskDelay(1); // Yields to allow other tasks to run
    }
}

void setup()
{
    Serial.begin(115200);
    SERIAL_PORT2.begin(115200);

    preferences.begin("myApp", false);
    testFlag = preferences.getInt("testFlag", 0);
    preferences.putInt("testFlag", ++testFlag);
    preferences.end();

    preferences.begin("factorySetup", true);
    int hardwareVersion = preferences.getInt("hardwareV", -1);
    preferences.end();
    if (hardwareVersion == -1 || hardwareVersion == 1) // Currently, units already on version 1 should not reach this point.
    {
        preferences.begin("factorySetup", false);
        preferences.clear();
        preferences.putInt("hardwareV", 2); // Change the hardware version here
        preferences.end();
        BleChess.factoryReset();
        esp_restart();
    }

    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(
        bleTask,   // Function to execute
        "bleTask", // Task name
        10000,     // Stack size
        NULL,      // Input parameter
        1,         // Task priority
        NULL,      // Task handle
        0          // Core where the task will run
    );

    //===============================================PINOUT
    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    cute.init(BUZZER_PIN);
    ledcSetup(5, 1000, 12); // channel, max frequency, resolution
    ledcAttachPin(BUZZER_PIN, 5);
    // Buzzer

    // Electromagnets
    pinMode(magnet1, OUTPUT);
    pinMode(magnet2, OUTPUT);
    pinMode(magnet3, OUTPUT);
    pinMode(magnet4, OUTPUT);

    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};
    //  Disconnect all PWM channels from the electromagnet pins
    for (int i = 0; i < 4; ++i)
    {
        ledcDetachPin(magnetPins[i]);      // Disconnect the PWM channel from the electromagnet pin
        digitalWrite(magnetPins[i], HIGH); // Set the pin high (off)
    }
    // Electromagnets

    // Sensors
    pinMode(mux8_0, OUTPUT);
    pinMode(mux8_1, OUTPUT);
    pinMode(mux8_2, OUTPUT);  // Batt S0
    pinMode(mux16_0, OUTPUT); // Batt S1
    pinMode(mux16_1, OUTPUT); // Batt S2
    pinMode(mux16_2, OUTPUT);
    pinMode(mux16Out_1, INPUT);
    pinMode(mux16Out_2, INPUT);
    pinMode(mux16Out_3, INPUT);
    pinMode(mux16Out_4, INPUT);
    // Sensors

    // Motors
    pinMode(MOTOR_0_STEP_PIN, OUTPUT);
    pinMode(MOTOR_0_DIR_PIN, OUTPUT);
    pinMode(MOTOR_1_STEP_PIN, OUTPUT);
    pinMode(MOTOR_1_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    // Motors
    //===============================================PINOUT

    sensorsDir();
    configDrivers();

    preferences.begin("myApp", false);
    preferences.putInt("testFlag", 0);
    int updatingStatus = preferences.getInt("updatingStatus", -1);
    preferences.end();

    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    if (testFlag >= 2 && testFlag < 8)
    {
        soundHandler(1);
        testFlag = -1;
        BleChess.setMode(4);
    }
    else if (testFlag >= 8 && testFlag < 15)
    {
        soundHandler(7);
        soundHandler(7);
        soundHandler(7);
        Serial.println("Factory Reset Mode");
        BleChess.factoryReset();
        esp_restart(); // DOES NOT DEACTIVATE PERIPHERALS
    }
    else
    {
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

        int modeSetup = BleChess.getModeChess();
        if (modeSetup != 1 && modeSetup != 2 && modeSetup != 3)
        {
            BleChess.setMode(1);
        }

        soundHandler(0);
        // delay(3000);

        mechanicalCalibration(BleChess.getCalibType());
    }

    Serial.println("batteryStatus: " + String(BleChess.getBatteryStatus()));
    Serial.println("Hardware Version: " + String(hardwareVersion));
    Serial.println("Firmware Version: 30 09 2024");
}

float euclideanDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void nearestCenter(float x, float y, float &centerX, float &centerY, float &kprima, float &lprima) // Function to find the nearest square to a given x,y position
{
    int k = 1;
    int l = 1;
    centerX = 9999;
    centerY = 9999;

    for (float i = 175; i >= -175; i -= 50)
    {
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

void loop()
{

    int newData = BleChess.getModeChess();
    mode = newData;
    switch (newData)
    {
    case 1:
        sculptureMode();
        break;
    case 2:
        playMode();
        break;
    case 3:
        pauseMode();
        break;
    case 4:
        testMode();
        break;
    case 5:
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

        mechanicalCalibration(BleChess.getCalibType());
        BleChess.setState("Moving to Lock Position");
        float tempPosX, tempPosY;
        rawMovement(-26.6, 198.5, -2, tempPosX, tempPosY);
        BleChess.setState("In Lock Position");
        // SEND ERROR EVERY 5 SECONDS
        {
            unsigned long timer = millis();
            BleChess.sendTestModeError("MECHANISM IN LOCK POSITION, PUT SCREWS AND RESTART THE BOARD");
            while (true)
            {
                if (millis() - timer > 10000)
                {
                    BleChess.sendTestModeError("MECHANISM IN LOCK POSITION, PUT SCREWS AND RESTART THE BOARD");
                    timer = millis();
                }
                cute._tone(NOTE_C7, 50, 100);
                delay(2000);
            }
        }
        break;
    default:
        unsigned long timer = millis();
        while (true)
        {
            cute._tone(NOTE_G3, 50, 100);
            if (millis() - timer > 10000)
            {
                BleChess.sendTestModeError("ERROR: MA-LOOP-312. Please send  error to the Phantom Team and restart the product.");
                timer = millis();
            }
        }
        break;
    }
}

void errorMessage(String error)
{
    deactivateAllMagnets();
    int timer = millis();
    Serial.println(error);
    BleChess.sendTestModeError(error);
    while (true)
    {
        cute._tone(NOTE_G3, 50, 100);
        if (millis() - timer > 5000)
        {
            Serial.println(error);
            BleChess.sendTestModeError(error);
            timer = millis();
        }
    }
}

void testMode()
{
    // boardRotation = 0;
    float posX = 0;
    float posY = 0;
    bool flagElectromagneting = true;
    int allOnes = 0;
    int sensorsOn = 0;
    String error = "";
    int delayElectromagnet = 300;
    unsigned long timeElectromagneting = 0;

    Serial.println("================================Test Mode================================");
    BleChess.setMode(4);
    //------------------------------------checking for drivers------------------------------
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
    //------------------------------------checking for drivers------------------------------
    mechanicalCalibration(BleChess.getCalibType());
    //------------------------------------check playable area------------------------------
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Area Check ");
    BleChess.setState("► Area Check ");
    Serial.println("If you see the end effector moving, and not cracking noises or any other issue, mechanism is ok");
    rawMovement(-180, 180, -3, posX, posY);
    rawMovement(180, 180, -3, posX, posY);
    rawMovement(180, -180, -3, posX, posY);
    rawMovement(-180, -180, -3, posX, posY);
    rawMovement(180, 180, -3, posX, posY);
    rawMovement(-180, 180, -3, posX, posY);
    rawMovement(180, -180, -3, posX, posY);
    Serial.println("► Area Check Completed ");
    //------------------------------------check playable area------------------------------

    //--------------------------------check electromagnets with paper----------------------------------
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Electromagnets Check ");
    BleChess.setState("► Electromagnets Check ");
    Serial.println("Using a piece of paper, check that the electromagnets are working properly");

    for (int electromagnet = 0; electromagnet <= 4; electromagnet++)
    {
        activateElectromagnetV2(electromagnet, 35);
        delay(1000);
        deactivateAllMagnets();
    }
    BleChess.setState("► Electromagnets Check Completed ");
    //--------------------------------check electromagnets with paper----------------------------------

    //-----------------------------------------battery check--------------------------------
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Battery Check ");
    BleChess.setState("► Battery Check ");
    int batteryStatus = BleChess.getBatteryStatus();
    if (batteryStatus == -1 && testFlag == -1)
    {
        Serial.println("✕ Battery is NOT connected");
        BleChess.sendTestModeError("✕ Battery is NOT connected");
        soundHandler(8);
    }

    //---------------------------------All Sensors are OFF && NO PIECES ON THE BOARD---------------------------------
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
    } while (sensorsOn != 0 && millis() - startTime < 30000); //|| millis() - startTime < 15000

    if (sensorsOn != 0)
    {
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
    //---------------------------------All Sensors are OFF && NO PIECES ON THE BOARD---------------------------------

    //-------------------------------------------Electromagnet Check---------------------------------
    Serial.println("────────────────────────────────────────────────────");
    Serial.println("► Electromagnet Check ");
    BleChess.setState("► Electromagnet Check ");

    int postocheck = 3;
    for (int electromagnet = 1; electromagnet <= 4; electromagnet++) // 4,4, 5,4 5,5 5,4
    {
        Serial.println("Checking in sensor: " + String(postocheck) + "," + String(postocheck));
        rawMovement(postocheck, postocheck, electromagnet, posX, posY);
        activateElectromagnetV2(electromagnet, 35);
        unsigned long timeElectromagneting = millis();
        do
        {
            readRawSensors(sensorMatrix);
            if (millis() - timeElectromagneting > 5000)
            {
                errorMessage("✕ electromagnet " + String(electromagnet) + " not turn on");
            }
        } while (sensorMatrix[postocheck][postocheck] == true);
        Serial.println("✓ electromagnet " + String(electromagnet) + " On");

        // Check the other electromagnets at the same position
        for (int i = 1; i <= 4; i++)
        {
            if (i != electromagnet)
            {
                rawMovement(postocheck, postocheck, i, posX, posY);
                readRawSensors(sensorMatrix);
                if (sensorMatrix[postocheck][postocheck] == false)
                {
                    errorMessage("✕ electromagnet " + String(i) + " is on when main electromagnet " + String(electromagnet) + " is on");
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

    //-------------------------------------------Electromagnet Check---------------------------------
    //-------------------------------------------Sensors Check---------------------------------
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
        // If the row is even, iterate from right to left
        if (j % 2 == 0)
        {
            for (int i = 0; i <= 9; i++)
            {
                // corners avoidance
                if ((i == 0 && j == 0) || (i == 9 && j == 0) || (i == 0 && j == 9) || (i == 9 && j == 9)) // skip corners
                {
                    continue;
                }
                // corners avoidance

                Serial.printf("\n ───────────── Sensor X: %d Y: %d\n", i, j);

                // Electromagnet selection
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
                // Eletromagnet selection

                rawMovement(i, j, electromagnet, posX, posY);

                // Sensors bouncing
                readRawSensors(sensorMatrix);
                for (int i = 0; i < 10; i++)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        sensorMatrixAux[i][j] = sensorMatrix[i][j];
                    }
                }
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

                Serial.println("✓ Bouncing: " + String(timesBouncing) + " times out of: " + String(checadas));
                if (timesBouncing > checadas * 0.001)
                {
                    errorMessage("✕ Sensor bouncing: " + String(i) + String(j));
                }
                checadas = 0;
                timesBouncing = 0;
                // Sensors bouncing

                // Specific Sensor Check
                if (sensorMatrix[i][j] == false)
                {
                    Serial.println("✓ Sensor " + String(i) + String(j) + " is on");
                }
                else
                {
                    errorMessage("✕ Sensor " + String(i) + String(j) + " is off");
                }
                // Specific Sensor Check

                // All the other sensors are off
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
                    Serial.println("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on");
                    BleChess.sendTestModeError("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on: " + error);
                    // IMPRIMIR MATRIZBIN
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
                            Serial.println("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on");
                            BleChess.sendTestModeError("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on: " + error);
                            // IMPRIMIR MATRIZBIN
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
                // All the other sensors are off
                sensorsOn2 = 0;
            }
        }
        else // If the row is odd, iterate from left to right
        {
            for (int i = 9; i >= 0; i--)
            {
                // corners avoidance
                if ((i == 0 && j == 0) || (i == 9 && j == 0) || (i == 0 && j == 9) || (i == 9 && j == 9)) // skip corners
                {
                    continue;
                }
                // corners avoidance

                Serial.printf("\n ───────────── Sensor X: %d Y: %d\n", i, j);

                // Electromagnet selection
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
                // Eletromagnet selection

                rawMovement(i, j, electromagnet, posX, posY);

                // Sensors bouncing
                readRawSensors(sensorMatrix);
                for (int i = 0; i < 10; i++)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        sensorMatrixAux[i][j] = sensorMatrix[i][j];
                    }
                }
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

                Serial.println("✓ Bouncing: " + String(timesBouncing) + " times out of: " + String(checadas));
                if (timesBouncing > checadas * 0.001)
                {
                    errorMessage("✕ Sensor bouncing: " + String(i) + String(j));
                }
                checadas = 0;
                timesBouncing = 0;
                // Sensors bouncing

                // Specific Sensor Check
                if (sensorMatrix[i][j] == false)
                {
                    Serial.println("✓ Sensor " + String(i) + String(j) + " is on");
                }
                else
                {
                    errorMessage("✕ Sensor " + String(i) + String(j) + " is off");
                }
                // Specific Sensor Check

                // All the other sensors are off
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
                    Serial.println("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on");
                    BleChess.sendTestModeError("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on: " + error);
                    // IMPRIMIR MATRIZBIN
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
                            Serial.println("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on");
                            BleChess.sendTestModeError("✕ More Sensors where found on when sensor " + String(i) + String(j) + " is on: " + error);
                            // IMPRIMIR MATRIZBIN
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
                // All the other sensors are off
                sensorsOn2 = 0;
            }
        }
    }

    //-----------------------------------------MESH CHECK--------------------------------
    Serial.println("────────────────────────────────────────────────────");
    Serial.print("► Testing Cable Lenght: ");
    BleChess.setState("► Testing Cable Lenght: ");
    float posX1 = 0;
    float posY1 = 0;
    sensorsCalibration(5, 5, 1, posX1, posY1);
    Serial.printf("X1: %f Y1: %f\n", posX1, posY1);
    Serial.println("Tryng to loose steps: ");
    rawMovement(0, 198, -2, posX, posY);
    sensorsCalibration(5, 4, 0, posX, posY);
    Serial.printf("X2: %f Y2: %f\n", posX, posY);

    float cableMissingX = posX - posX1;
    float cableMissingY = posY - posY1;

    Serial.printf("Error X: %f Y: %f\n", cableMissingX, cableMissingY);

    // if total error x or y is greater than 2mm, print error; otherwise print that it passed
    if ((posY - posY1 <= -1.5 || posY - posY1 >= 1.5))
    {
        errorMessage("✕ Cable Lenght Failed" + String(cableMissingX) + " " + String(cableMissingY));
    }
    else
    {
        Serial.println("✓ Cable Lenght is correct.");
    }
    //-----------------------------------------MESH CHECK--------------------------------

    //-----------------------------------------After Test is finish--------------------------------

    if (testFlag == -1) // test mode triggered from the board
    {
        Serial.println("────────────────────────────────────────────────────");
        Serial.print("► Starting Full Sculpture Mode 10 minutes Routine: ");
        BleChess.setState("► Testing Cable Lenght: ");

        testModeSculpture = true;
        BleChess.setMode(1);
        testFlag = -2;
        return;
    }
    else // test mode triggered from the phone
    {
        if (BleChess.getBatteryStatus() == -1)
        {
            Serial.println("✓ FULL TEST PASSED BUT BATTERY NOT DETECTED");
            BleChess.sendTestModeError("✓ FULL TEST PASSED BUT BATTERY NOT DETECTED");
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

void sculptureMode()
{
    // Serial.println("Sculpture Mode");
    sculptureMain();
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

struct GameState
{
    char matrix[10][10];
    enum class Side
    {
        BOARD = 1,
        BLE = 2
    } currentSide;
    int specialMove;
    bool specialMoveActive = false;
    bool isGameOver;
};

class ChessGameManager
{
private:
    GameState state;
    Bluetooth &bleManager;

    bool initializeGame()
    {
        // Initialize matrix and get initial game state
        initMatrixPlus(state.matrix);

        // Wait for valid matrix
        String matrixGame = waitForValidMatrix();
        if (matrixGame.isEmpty())
        {
            Serial.println("Failed to get valid initial matrix");
            return false;
        }
        bleManager.resetMatrixInitGame();

        // Convert and setup board
        setupBoard(matrixGame);
        state.isGameOver = false;
        return true;
    }

    String waitForValidMatrix()
    {
        while (true)
        {
            String matrix = bleManager.getMatrixInitGame();
            if (matrix != "" && matrix != String('\0'))
                return matrix;

            if (!checkGameStatus())
                return "";
            delay(100);
        }
    }

    void printMatrix(char matrix[10][10])
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                Serial.print(matrix[i][j]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    void setupBoard(const String &matrixGame)
    {
        char initMatrix[10][10] = {0};
        stringToMatrix(matrixGame, initMatrix);
        printMatrix(initMatrix);

        bleManager.setState("Setting Up");
        reorderChessboardPlus(3, initMatrix, state.matrix);
        bleManager.resetMatrixInitGame();
        bleManager.resetVerifNewCommandBluetooth();

        compareMatrixVsSensorsPlus(-1, state.matrix);
        detectChessBoard(sensorMatrix);
        bleManager.sendMatrixToApp("CLEAN: Match.", sensorMatrix, state.matrix);
        bleManager.setState("Playing");

        for (int j = 0; j < 10; j++)
            for (int i = 0; i < 10; i++)
                previousChessState[i][j] = sensorMatrix[i][j];
    }

    bool verifyMovement(const String &move)
    {
        if (move.length() < 6)
            return false;

        char x1 = move.charAt(4), y1 = move.charAt(5);
        char x2 = move.charAt(7), y2 = move.charAt(8);

        if (!isValidCoordinate(x1, y1) || !isValidCoordinate(x2, y2))
            return false;

        if (state.specialMoveActive)
        {
            state.specialMoveActive = false;
            state.specialMove = 0;
            return true;
        }

        if (state.specialMove > 0 && !state.specialMoveActive)
            state.specialMoveActive = true;

        bleManager.setCheckMove("0");
        bleManager.setStatus(move);
        while (true)
        {
            String response = bleManager.getCheckMove();
            if (response == "1")
                return true;
            if (response == "2") {
                soundHandler(4);
                return false;
            }
            delay(100);
        }
    }

    bool isValidCoordinate(char x, char y)
    {
        return (x >= 'a' && x <= 'h') && (y >= '1' && y <= '8');
    }

    bool handleMove()
    {
        while (true)
        {
            if (state.currentSide == GameState::Side::BOARD)
            {
                Serial.println("Waiting for board move");
                String move = detectChangePlus(state.matrix, state.specialMove);
                String voiceCmd = bleManager.getVoiceCommand();
                if (voiceCmd.length() > 0)
                {
                    Serial.printf("Voice Command: %s\n", voiceCmd.c_str());
                    automaticMechanicMovement(voiceCmd, state.matrix);
                    state.currentSide = GameState::Side::BLE;
                    bleManager.resetVoiceCommand();
                    move = "1";
                    return true;
                }
                if (move.length() > 0 && verifyMovement(move))
                {
                    soundHandler(5);
                    if (state.specialMoveActive)
                        continue;
                    soundEndGame();
                    state.currentSide = GameState::Side::BLE;
                    return true;
                }
            }
            else if (state.currentSide == GameState::Side::BLE)
            {
                Serial.println("Waiting for BLE move");
                String move = bleManager.verifNewCommandBluetooth();
                if (move.length() > 0)
                {
                    Serial.printf("BLE Move: %s\n", move.c_str());
                    automaticMechanicMovement(move, state.matrix);
                    bleManager.resetVerifNewCommandBluetooth();
                    state.currentSide = GameState::Side::BOARD;
                    soundEndGame();
                    return true;
                }
            }

            if (!checkGameStatus())
                return false;
            delay(1000);
        }
    }

    void updateMatrixFromFen(const String &fen)
    {
        // 1. Save graveyard pieces
        char graveyardTop[10];    // Row 0
        char graveyardBottom[10]; // Row 9
        char graveyardLeft[8];    // Column 0, rows 1-8
        char graveyardRight[8];   // Column 9, rows 1-8

        // Save border pieces
        for (int i = 0; i < 10; i++)
        {
            graveyardTop[i] = state.matrix[i][0];
            graveyardBottom[i] = state.matrix[i][9];
        }
        for (int i = 1; i < 9; i++)
        {
            graveyardLeft[i - 1] = state.matrix[0][i];
            graveyardRight[i - 1] = state.matrix[9][i];
        }

        // 2. Parse FEN and update center
        String fenPosition = fen.substring(0, fen.indexOf(' '));
        int row = 1, col = 1;

        // Clear center 8x8
        for (int i = 1; i <= 8; i++)
        {
            for (int j = 1; j <= 8; j++)
            {
                state.matrix[i][j] = '.';
            }
        }

        // Fill with new position
        for (char c : fenPosition)
        {
            if (c == '/')
            {
                row++;
                col = 1;
            }
            else if (isdigit(c))
            {
                col += c - '0';
            }
            else
            {
                state.matrix[col][row] = c;
                col++;
            }
        }

        // 3. Restore graveyard
        for (int i = 0; i < 10; i++)
        {
            state.matrix[i][0] = graveyardTop[i];
            state.matrix[i][9] = graveyardBottom[i];
        }
        for (int i = 1; i < 9; i++)
        {
            state.matrix[0][i] = graveyardLeft[i - 1];
            state.matrix[9][i] = graveyardRight[i - 1];
        }
    }

    // Eliminate piece from graveyard
    void eliminateFromGrave(char pieceChar)
    {
        // Border positions to check (top, bottom, left, right borders)
        const int borders[4][2] = {
            {0, -1}, // Top row
            {9, -1}, // Bottom row
            {-1, 0}, // Left column
            {-1, 9}  // Right column
        };

        bool found = false;

        // Check top and bottom rows
        for (int i = 0; i < 2 && !found; i++)
        {
            if (borders[i][0] != -1)
            {
                for (int j = 1; j < 9 && !found; j++)
                {
                    if (state.matrix[j][borders[i][0]] == pieceChar)
                    {
                        state.matrix[j][borders[i][0]] = '.';
                        found = true;
                    }
                }
            }
        }

        // Check left and right columns
        for (int i = 2; i < 4 && !found; i++)
        {
            if (borders[i][1] != -1)
            {
                for (int j = 1; j < 9 && !found; j++)
                {
                    if (state.matrix[borders[i][1]][j] == pieceChar)
                    {
                        state.matrix[borders[i][1]][j] = '.';
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            Serial.print("Removed piece from graveyard: ");
            Serial.println(pieceChar);
        }
        else
        {
            Serial.print("Piece not found in graveyard: ");
            Serial.println(pieceChar);
        }
    }

    // Helper function to copy 10x10 matrix
    void copyMatrix(char source[10][10], char dest[10][10])
    {
        // Use memcpy for each row to ensure complete copy
        for (int i = 0; i < 10; i++)
        {
            memcpy(dest[i], source[i], 10 * sizeof(char));
        }
    }

    void handleTakeback()
    {
        char originalMatrix[10][10] = {0};
        copyMatrix(state.matrix, originalMatrix);

        // Wait for takeback message
        String takebackStr = "";
        while (takebackStr == "")
        {
            String str = bleManager.getTakeback();
            if (str != "" && str != String('\0'))
                takebackStr = str;
            delay(100);
        }

        // Split message by commas
        int firstComma = takebackStr.indexOf(',');
        if (firstComma != -1)
        {
            int secondComma = takebackStr.indexOf(',', firstComma + 1);
            if (secondComma != -1)
            {
                // Extract all parts
                String capturedPieces = takebackStr.substring(0, firstComma);
                String newFen = takebackStr.substring(firstComma + 1, secondComma);
                String side = takebackStr.substring(secondComma + 1);

                // Handle captured pieces
                int spaceIndex = capturedPieces.indexOf(' ');
                if (spaceIndex != -1)
                {
                    String firstPiece = capturedPieces.substring(0, spaceIndex);
                    String secondPiece = capturedPieces.substring(spaceIndex + 1);
                    eliminateFromGrave(firstPiece.charAt(0));
                    eliminateFromGrave(secondPiece.charAt(0));
                }
                else if (capturedPieces.length() > 0)
                {
                    eliminateFromGrave(capturedPieces.charAt(0));
                }

                Serial.print("New FEN: ");
                Serial.println(newFen);
                Serial.print("Side: ");
                Serial.println(side);

                updateMatrixFromFen(newFen);
                state.currentSide = (side == "1") ? GameState::Side::BOARD : GameState::Side::BLE;
            }
        }
        else
        {
            Serial.println("Error: Invalid takeback format - missing commas");
        }

        char newMatrix[10][10] = {0};
        reorderChessboardPlus(3, state.matrix, originalMatrix);
        compareMatrixVsSensorsPlus(0, state.matrix);
    }

    bool checkGameStatus()
    {
        int mode = bleManager.getModeChess();
        if (mode == 7)
        {
            // handle calibration
            mechanicalCalibration(bleManager.getCalibType());
            bleManager.setState("Playing");
            BleChess.setMode(2);
            return true;
        }
        if (mode == 9)
        {
            // new matrix reorder
            bleManager.setState("Setting Up");
            handleTakeback();
            bleManager.setState("Playing");
            BleChess.setMode(2);
            return true;
        }
        if (mode != 2 || globalConnect == "0")
        {
            state.isGameOver = true;
            return false;
        }
        return true;
    }

public:
    ChessGameManager(Bluetooth ble) : bleManager(ble)
    {
        memset(&state, 0, sizeof(state));
    };

    void run()
    {
        String currentSideInfo = BleChess.getPlayInfo();
        while (currentSideInfo == "")
        {
            currentSideInfo = BleChess.getPlayInfo();
            delay(100);
        }
        state.currentSide = currentSideInfo == "1" ? GameState::Side::BOARD : GameState::Side::BLE;
        Serial.println("currentSideInfo: " + currentSideInfo);
        Serial.printf("currentSide: %d\n", state.currentSide);

        if (!initializeGame())
            return;

        while (!state.isGameOver)
            if (!handleMove())
                break;

        // Cleanup and reset board
        bleManager.setMode(3);
        reorderChessboardPlus(0, nullptr, state.matrix);
    }
};

void playMode()
{
    ChessGameManager gameManager(BleChess);
    gameManager.run();
}

void soundEndGame()
{
    unsigned long timeWaiting = millis();
    String mksound = "";
    while (mksound == "" && millis() - timeWaiting < 100)
    {
        mksound = BleChess.getSoundCheck();
    }
    Serial.println("-------------------------------------------------timeWaiting: " + String(millis() - timeWaiting));
    Serial.println("Make Sound main: " + mksound);
    if (mksound == "1") // Check
    {
        Serial.println("Jaque SONIDO ");
        soundHandler(10);
    }
    if (mksound == "2") // Checkmate
    {
        Serial.println("Mate SONIDO ");
        soundHandler(11);
    }
    // BleChess.resetSoundCheck();
}

void automaticMechanicMovement(String movementString, char matrixToAutomaticMove[10][10])
{
    char colorTurn;
    char piezaChess = 'v';
    char movementChess = '!';

    int n = movementString.length();
    char array[n + 1];
    strcpy(array, movementString.c_str());

    char x_iniChar = array[2];
    char y_iniChar = array[3];
    char x_finChar = array[5];
    char y_finChar = array[6];

    //-------------------------------------Find Piece
    int x_ini = 0, x_fin = 0, y_ini = 0, y_fin = 0;

    char posCoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char posCoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
    for (int i = 0; i < 8; i++) // Scan in X
    {
        if (x_iniChar == posCoordBoardX[i]) // If the letter matches the letter in the array, return the position index
        {
            x_ini = i + 1;
        }
        if (x_finChar == posCoordBoardX[i]) // If the letter matches the letter in the array, return the position index
        {
            x_fin = i + 1;
        }
        if (y_iniChar == posCoordBoardY[i]) // If the letter matches the letter in the array, return the position index
        {
            y_ini = i + 1;
        }
        if (x_finChar == posCoordBoardY[i]) // If the letter matches the letter in the array, return the position index
        {
            y_fin = i + 1;
        }
    }

    piezaChess = matrixToAutomaticMove[x_ini][y_ini];
    if (piezaChess == '.')
    {
        piezaChess = 'N';
    }
    if (isUpperCase(piezaChess))
    {
        // Serial.println("White turn");
        colorTurn = 'W';
    }
    else if (isLowerCase(piezaChess))
    {
        // Serial.println("Black turn");
        colorTurn = 'B';
    }

    //-------------------------------------Find Piece

    //-------------------------------------Find Move
    if (array[4] == '-') // Simple move
    {
        movementChess = '0';
    }
    if (array[4] == 'x') // Capture
    {
        movementChess = '1';
        prevDeadPiecesCount++;
    }
    if (piezaChess == 'K' || piezaChess == 'k')
    {
        if (x_iniChar == 'e' && x_finChar == 'g' && y_iniChar == '1' && y_finChar == '1') // Short Castling
        {
            movementChess = '2';
        }
        if (x_iniChar == 'e' && x_finChar == 'c' && y_iniChar == '1' && y_finChar == '1') // Long Castling
        {
            movementChess = '3';
        }
        if (x_iniChar == 'e' && x_finChar == 'g' && y_iniChar == '8' && y_finChar == '8') // Short Castling
        {
            movementChess = '2';
        }
        if (x_iniChar == 'e' && x_finChar == 'c' && y_iniChar == '8' && y_finChar == '8') // Long Castling
        {
            movementChess = '3';
        }
    }

    if (movementChess == '!') // If the movement cannot be decoded, return
    {
        Serial.println("Movimiento: " + String(movementChess));
        Serial.println("===================================NO DECODIFICADO, RETURN");
        return;
    }

    Serial.println("Movimiento: " + String(movementChess));
    //-------------------------------------Find Move
    Serial.println("Movimiento automatico, Initial Matrix: ");
    printGenericMatrix(matrixToAutomaticMove, 10, 10);

    String fullMoves = decodeMovement(x_iniChar, y_iniChar, x_finChar, y_finChar, movementChess, colorTurn, matrixToAutomaticMove, true);
    Serial.println("Movimiento FULL: " + fullMoves);

    // take back movement without single move
    simplifiedMovement(fullMoves[0] - '0', fullMoves[1] - '0', fullMoves[2] - '0', fullMoves[3] - '0', matrixToAutomaticMove);
    Serial.println("Movimiento Simple: ");
    printGenericMatrix(matrixToAutomaticMove, 10, 10);
    if (fullMoves[4] != '/')
    {

        simplifiedMovement(fullMoves[4] - '0', fullMoves[5] - '0', fullMoves[6] - '0', fullMoves[7] - '0', matrixToAutomaticMove);
        Serial.println("Movimiento Comer: ");
        printGenericMatrix(matrixToAutomaticMove, 10, 10);
    }

    BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrix, matrixToAutomaticMove);
}

String detectChangePlus(char currentMatrix[10][10], int &specialMove)
{
    // Returns 1 if a change was detected and the piece has not been placed yet, 0 if a change was detected and the piece was already placed, -1 in a special case.
    // Serial.println("Detecting Change");
    bool changeFlag = LOW;
    int colocaPieza = -1;
    int iAnterior = -1;
    int jAnterior = -1;
    int iActual = -1;
    int jActual = -1;
    // TiempoAhora = millis();
    std::vector<int> vectorFinChangeX;
    std::vector<int> vectorFinChangeY;
    std::vector<int> vectorIniChangeX;
    std::vector<int> vectorIniChangeY;
    int countPieces = 0;

    unsigned long lastTimeFunctionRanMissingPieces = 0;
    unsigned long startTimeMissingPieces = millis();

    unsigned long startTime = millis();            // Variable to store the start time of the while loop
    unsigned long lastTimeFunctionRan = startTime; // Variable to store the last time the function ran
    unsigned long lastSoundTime = 0;               // Variable to store the last time the sound was triggered

    unsigned long matrixChangeStartTime = 0;         // Variable to store the start time of the detected change
    const unsigned long matrixChangeThreshold = 500; // Minimum time (in milliseconds) for a change to be considered valid
    String comandoJugada = "";

    while (true)
    {
        //------------Bluetooth Manager------------------
        int newData = BleChess.getModeChess();
        // int newMode = newData.toInt();
        if (newData != 2)
        {
            return "";
            break;
        }
        if (globalConnect == "0")
        {
            BleChess.setMode(3);
            return "";
            break;
        }
        //------------Bluetooth Manager------------------

        detectChessBoard(sensorMatrix);

        countPieces = 0;
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (sensorMatrix[i][j] == 0)
                {
                    countPieces++;
                }
            }
        }

        // Check if 3 seconds have passed since the last time the function ran
        if (millis() - lastTimeFunctionRan >= 3000)
        {
            // Check if there are less than 32 pieces on the chessboard
            if (countPieces < 32)
            {
                // Check if a sound was already triggered in the last 3 seconds
                if (millis() - lastSoundTime >= 5000)
                {
                    // Trigger the sound

                    soundHandler(6);
                    Serial.print("More than 5 sec without piece");
                    BleChess.sendMatrixToApp("ERROR: Chessboard and sensor matrix do not match.", sensorMatrix, currentMatrix);

                    lastSoundTime = millis(); // Update the last sound time
                }
            }

            // Update the last time the function ran
            lastTimeFunctionRan = millis();
        }

        bool matrixChanged = false;

        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (previousChessState[i][j] != sensorMatrix[i][j])
                {
                    matrixChanged = true;
                    // Serial.print("Changed: ");
                    // Serial.print(i);
                    // Serial.println(j);
                    break;
                }
            }
            if (matrixChanged)
                break;
        }

        if (matrixChanged)
        {
            if ((millis() - matrixChangeStartTime) >= matrixChangeThreshold)
            {
                matrixChangeStartTime = millis();
                for (int j = 0; j < 10; j++)
                {
                    for (int i = 0; i < 10; i++)
                    {
                        if (previousChessState[i][j] != sensorMatrix[i][j])
                        {
                            if (sensorMatrix[i][j] == false) // Piece placed
                            {
                                lastTimeFunctionRan = millis();
                                lastSoundTime = millis();
                                finChangeX = i;
                                finChangeY = j;
                                vectorFinChangeX.push_back(i);
                                vectorFinChangeY.push_back(j);
                                Serial.print("Quantity: ");
                                Serial.println(countPieces);
                                if (countPieces == 31)
                                {
                                    soundHandler(5);
                                }
                            }
                            if (sensorMatrix[i][j] == true) // Piece taken
                            {
                                lastTimeFunctionRan = millis();
                                lastSoundTime = millis();
                                iniChangeX = i;
                                iniChangeY = j;
                                vectorIniChangeX.push_back(i);
                                vectorIniChangeY.push_back(j);
                                soundHandler(5);
                            }
                        }
                    }
                }
                for (int jj = 0; jj < 10; jj++)
                {
                    for (int ii = 0; ii < 10; ii++)
                    {
                        previousChessState[ii][jj] = sensorMatrix[ii][jj];
                    }
                }
                if (countPieces == 32)
                {

                    break;
                }
            }
        }

        String voiceCommand = BleChess.getVoiceCommand();
        if (voiceCommand != "")
        {
            break;
        }
    }
    BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrix, currentMatrix);
    if (vectorFinChangeX.size() != 0 || vectorIniChangeX.size() != 0)
    {
        if (vectorFinChangeX.size() == vectorIniChangeX.size())
        {
            bool flagMove = false;
            bool flagGenerateCommand = false;
            bool flagCaptured = false;
            int iniChangeXCaptured = -1;
            int iniChangeYCaptured = -1;
            int finChangeXCaptured = -1;
            int finChangeYCaptured = -1;
            // --Sound to avoid movement to corners
            for (int i = 0; i < vectorIniChangeX.size(); i++)
            {
                if ((vectorIniChangeX[i] == 0 && vectorIniChangeY[i] == 0) || (vectorIniChangeX[i] == 0 && vectorIniChangeY[i] == 9) || (vectorIniChangeX[i] == 9 && vectorIniChangeY[i] == 0) || (vectorIniChangeX[i] == 9 && vectorIniChangeY[i] == 9))
                {
                    soundHandler(4);
                }
            }

            for (int i = 0; i < vectorFinChangeX.size(); i++)
            {
                if ((vectorFinChangeX[i] == 0 && vectorFinChangeY[i] == 0) || (vectorFinChangeX[i] == 0 && vectorFinChangeY[i] == 9) || (vectorFinChangeX[i] == 9 && vectorFinChangeY[i] == 0) || (vectorFinChangeX[i] == 9 && vectorFinChangeY[i] == 9))
                {
                    soundHandler(4);
                }
            }
            // --Sound to avoid movement to corners
            if (vectorFinChangeX.size() == 1)
            {
                // Serial.println("1 Move");
                int iniX = vectorIniChangeX[0];
                int iniY = vectorIniChangeY[0];
                int finX = vectorFinChangeX[0];
                int finY = vectorFinChangeY[0];
                if (((iniX > 0 && iniX < 9) && (iniY > 0 && iniY < 9)) && ((finX > 0 && finX < 9) && (finY > 0 && finY < 9)))
                {
                    Serial.println("Simple movement");
                    flagMove = true;
                    flagGenerateCommand = true;
                }
                else if (((iniX > 0 && iniX < 9) && (iniY > 0 && iniY < 9)) && ((finX == 0 || finX == 9) || (finY == 0 || finY == 9)))
                {
                    Serial.println("Piece captured");
                    flagMove = true;
                    soundHandler(5);
                }
                else if (((iniX == 0 || iniX == 9) || (iniY == 0 || iniY == 9)) && ((finX > 0 && finX < 9) && (finY > 0 && finY < 9)))
                {
                    Serial.println("Piece returned from captured");

                    flagMove = true;
                    soundHandler(5);
                }
                else
                {
                    Serial.println("Invalid movement");
                    soundHandler(4);
                    delay(50);
                }
                iniChangeX = vectorIniChangeX[0];
                iniChangeY = vectorIniChangeY[0];
                finChangeX = vectorFinChangeX[0];
                finChangeY = vectorFinChangeY[0];
            }
            if (vectorFinChangeX.size() == 2)
            {
                flagGenerateCommand = true;
                int indexSamePositionStart = -1;
                int indexSamePositionEnd = -1;
                Serial.print("Start:");
                //-- Just verbosing data and finding same position logic
                for (int i = 0; i < vectorIniChangeX.size(); i++)
                {
                    Serial.print("(");
                    Serial.print(vectorIniChangeX[i]);
                    Serial.print(",");
                    Serial.print(vectorIniChangeY[i]);
                    Serial.print(")");
                    Serial.print(",");
                    for (int j = 0; j < vectorFinChangeX.size(); j++)
                    {
                        if ((vectorIniChangeX[i] == vectorFinChangeX[j]) && (vectorIniChangeY[i] == vectorFinChangeY[j]))
                        {
                            indexSamePositionStart = i;
                            indexSamePositionEnd = j;
                        }
                    }
                }
                Serial.println("");
                Serial.print("Finnd:");
                for (int i = 0; i < vectorFinChangeX.size(); i++)
                {
                    Serial.print("(");
                    Serial.print(vectorFinChangeX[i]);
                    Serial.print(",");
                    Serial.print(vectorFinChangeY[i]);
                    Serial.print(")");
                    Serial.print(",");
                }
                Serial.println("");

                ///
                if (indexSamePositionStart > -1)
                {
                    if (indexSamePositionStart == 0)
                    {
                        flagMove = true;
                        flagCaptured = true;
                        // Move positions.
                        iniChangeX = vectorIniChangeX[1];
                        iniChangeY = vectorIniChangeY[1];
                        finChangeX = vectorIniChangeX[0];
                        finChangeY = vectorIniChangeY[0];

                        // Captured positions
                        iniChangeXCaptured = vectorIniChangeX[0];
                        iniChangeYCaptured = vectorIniChangeY[0];
                        if (indexSamePositionEnd == 0)
                        {
                            finChangeXCaptured = vectorFinChangeX[1];
                            finChangeYCaptured = vectorFinChangeY[1];
                        }
                        else if (indexSamePositionEnd == 1)
                        {
                            finChangeXCaptured = vectorFinChangeX[0];
                            finChangeYCaptured = vectorFinChangeY[0];
                        }
                    }
                    if (indexSamePositionStart == 1)
                    {
                        flagMove = true;
                        flagCaptured = true;
                        // Move positions
                        iniChangeX = vectorIniChangeX[0];
                        iniChangeY = vectorIniChangeY[0];
                        finChangeX = vectorIniChangeX[1];
                        finChangeY = vectorIniChangeY[1];
                        // Captured positions
                        iniChangeXCaptured = vectorIniChangeX[1];
                        iniChangeYCaptured = vectorIniChangeY[1];
                        if (indexSamePositionEnd == 0)
                        {
                            finChangeXCaptured = vectorFinChangeX[1];
                            finChangeYCaptured = vectorFinChangeY[1];
                        }
                        else if (indexSamePositionEnd == 1)
                        {
                            finChangeXCaptured = vectorFinChangeX[0];
                            finChangeYCaptured = vectorFinChangeY[0];
                        }
                    }
                }
                else
                {
                    soundHandler(6);
                    Serial.print("Error not find same position logic TODO check logic");
                }
            }
            if (vectorFinChangeX.size() > 2)
            {
                soundHandler(6);
                Serial.print("Multiple movements detected, check logic");
                Serial.print("Start:");
                for (int i = 0; i < vectorIniChangeX.size(); i++)
                {
                    Serial.print("(");
                    Serial.print(vectorIniChangeX[i]);
                    Serial.print(",");
                    Serial.print(vectorIniChangeY[i]);
                    Serial.print(")");
                    Serial.print(",");
                }
                Serial.println("");
                Serial.print("Finnd:");
                for (int i = 0; i < vectorFinChangeX.size(); i++)
                {
                    Serial.print("(");
                    Serial.print(vectorFinChangeX[i]);
                    Serial.print(",");
                    Serial.print(vectorFinChangeY[i]);
                    Serial.print(")");
                    Serial.print(",");
                }
            }

            if (flagCaptured)
            {

                // leerMatriz(matrizPlusGraveyard);
                char temp = currentMatrix[iniChangeXCaptured][iniChangeYCaptured];
                currentMatrix[finChangeXCaptured][finChangeYCaptured] = temp;
                // guardarMatriz(matrizPlusGraveyard);
            }

            if (flagMove)
            {
                char piezaEnMovimiento = 'v';
                if (flagGenerateCommand)
                {
                    // leerMatriz(matrizPlusGraveyard);
                    piezaEnMovimiento = currentMatrix[iniChangeX][iniChangeY];
                    bool colorPiezaActual = false;
                    char arrayComandoJugada[9];
                    char CoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
                    char CoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};

                    arrayComandoJugada[0] = 'M';
                    arrayComandoJugada[1] = ' ';
                    arrayComandoJugada[2] = '1';
                    arrayComandoJugada[3] = ' ';
                    arrayComandoJugada[4] = CoordBoardX[iniChangeX - 1];
                    arrayComandoJugada[5] = CoordBoardY[iniChangeY - 1];
                    if (flagCaptured == 0)
                    {
                        arrayComandoJugada[6] = '-';
                    }
                    else
                    {
                        arrayComandoJugada[6] = 'x';
                    }
                    arrayComandoJugada[7] = CoordBoardX[finChangeX - 1];
                    arrayComandoJugada[8] = CoordBoardY[finChangeY - 1];
                    for (int k = 0; k < 9; k++)
                    {
                        comandoJugada = comandoJugada + arrayComandoJugada[k];
                    }
                    // Serial.print("DAta to check castling");
                    // Serial.println(piezaEnMovimiento);
                    // Serial.println(CoordBoardX[iniChangeX - 1]);
                    // Serial.println(CoordBoardX[finChangeX - 1]);
                    // Serial.println("");
                    if (piezaEnMovimiento == 'K') // White pieces are being moved
                    {
                        if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'g') // Short castling
                        {
                            specialMove = 10; // To identify an ongoing castling move
                        }
                        if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'c') // Long castling
                        {
                            specialMove = 11; // To identify an ongoing castling move
                        }
                    }

                    if (piezaEnMovimiento == 'k') // Black pieces are being moved
                    {
                        if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'g') // Short castling
                        {
                            specialMove = 12; // To identify an ongoing castling move
                        }
                        if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'c') // Long castling
                        {
                            specialMove = 13; // To identify an ongoing castling move
                        }
                    }
                }

                piezaEnMovimiento = currentMatrix[iniChangeX][iniChangeY];
                currentMatrix[iniChangeX][iniChangeY] = '.';
                currentMatrix[finChangeX][finChangeY] = piezaEnMovimiento;
            }

            // flagNewMovement = false;
        }
        else
        {
            Serial.println("Error, the dimensios is not the same, could cause a crash.");
            Serial.print("REceive matrix from chessboard and show modal will help to solve the matrix?????????");
            Serial.println("Here only will compare chessboard, so chessboard should match and if count the 32 pieces pass");
            Serial.print("Because is possible a false movement in the captured pieces so, only count chessboard and bin sensrs");
            Serial.print("Start:");
            for (int i = 0; i < vectorIniChangeX.size(); i++)
            {
                Serial.print("(");
                Serial.print(vectorIniChangeX[i]);
                Serial.print(",");
                Serial.print(vectorIniChangeY[i]);
                Serial.print(")");
                Serial.print(",");
            }
            Serial.println("");
            Serial.print("Finnd:");
            for (int i = 0; i < vectorFinChangeX.size(); i++)
            {
                Serial.print("(");
                Serial.print(vectorFinChangeX[i]);
                Serial.print(",");
                Serial.print(vectorFinChangeY[i]);
                Serial.print(")");
                Serial.print(",");
            }
        }
    }

    return comandoJugada;
}

void stringToMatrix(String str, char matrix[10][10])
{
    int index = 0;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrix[i][j] = str[index];
            index++;
        }
    }
}
