/*
 * sensor_io.cpp
 * Low-level sensor I/O for the 10x10 Hall-effect sensor matrix.
 *
 * HARDWARE OVERVIEW:
 * The board uses two cascaded multiplexer layers to address 100 Hall sensors:
 *
 *   - Two 16-channel (CD74HC4067) multiplexers select which of up to 16 sensors
 *     on a given row/column grouping is connected to an output data pin.
 *     Control is shared: mux16_0, mux16_1, mux16_2 (3-bit address -> selects 1 of 8 channels).
 *
 *   - One 8-channel (CD74HC4051) multiplexer further selects which physical
 *     sensor group is activated.
 *     Control: mux8_0, mux8_1, mux8_2 (3-bit address -> selects 1 of 8 groups).
 *
 *   - Data output is read from one of 4 digital input pins (mux16Out_1..4),
 *     selected by the hundreds digit in the dirMux[][] address code.
 *
 * The dirMux[i][j] address encoding (3-digit decimal):
 *   Hundreds = muxesOut[] array index (which of the 4 output data pins)
 *   Tens     = mux16 3-bit channel selector value (0-7)
 *   Units    = mux8  3-bit channel selector value (0-7)
 *
 * updateSensors() is called frequently from the BLE FreeRTOS task running on
 * the second ESP32 core.  It reads raw hardware values into a local buffer,
 * applies the configured board rotation, and writes the result to sensorUpdate.
 * Other code reads from sensorUpdate to get the latest stable snapshot.
 *
 * Sensor logic convention:
 *   sensorMatrix[i][j] == false -> piece present  (sensor triggered / magnet detected)
 *   sensorMatrix[i][j] == true  -> square is empty
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "sensor_state.h"

// sensorMatrix is the global boolean matrix defined in main.cpp that is shared
// between the BLE task and the main game logic task.
extern bool sensorMatrix[10][10];

// ─────────────────────────────────────────────────────────────────────────────
// sensorsDir()
// Populates the dirMux[10][10] lookup table with the mux address codes for
// every cell in the 10x10 sensor matrix.
//
// Each entry is a 3-digit integer:
//   hundreds = index into muxesOut[] (1-4)
//   tens     = 3-bit selector for the 16-channel mux  (mux16_0/1/2 pins)
//   units    = 3-bit selector for the  8-channel mux  (mux8_0/1/2 pins)
//
// The order within each column corresponds to the physical wiring: index 0 is
// the sensor farthest from the board centre, index 9 is closest to centre.
// ─────────────────────────────────────────────────────────────────────────────
void sensorsDir()
{
    dirMux[0][0] = 464; // sensor farthest from middle
    dirMux[1][0] = 466;
    dirMux[2][0] = 467;
    dirMux[3][0] = 465;
    dirMux[4][0] = 462;
    dirMux[5][0] = 461;
    dirMux[6][0] = 460;
    dirMux[7][0] = 463;
    dirMux[8][0] = 440;
    dirMux[9][0] = 420; // sensor closest to middle
                        // barra2
    dirMux[0][1] = 414; // sensor farthest from middle
    dirMux[1][1] = 416;
    dirMux[2][1] = 417;
    dirMux[3][1] = 415;
    dirMux[4][1] = 412;
    dirMux[5][1] = 411;
    dirMux[6][1] = 410;
    dirMux[7][1] = 413;
    dirMux[8][1] = 470;
    dirMux[9][1] = 450;

    dirMux[0][2] = 324; // sensor farthest from middle
    dirMux[1][2] = 326;
    dirMux[2][2] = 327;
    dirMux[3][2] = 325;
    dirMux[4][2] = 322;
    dirMux[5][2] = 321;
    dirMux[6][2] = 320;
    dirMux[7][2] = 323;
    dirMux[8][2] = 430;
    dirMux[9][2] = 400;

    dirMux[0][3] = 344; // sensor farthest from middle
    dirMux[1][3] = 346;
    dirMux[2][3] = 347;
    dirMux[3][3] = 345;
    dirMux[4][3] = 342;
    dirMux[5][3] = 341;
    dirMux[6][3] = 340;
    dirMux[7][3] = 343;
    dirMux[8][3] = 360;
    dirMux[9][3] = 310;

    dirMux[0][4] = 334; // sensor farthest from middle
    dirMux[1][4] = 336;
    dirMux[2][4] = 337;
    dirMux[3][4] = 335;
    dirMux[4][4] = 332;
    dirMux[5][4] = 331;
    dirMux[6][4] = 330;
    dirMux[7][4] = 333;
    dirMux[8][4] = 300;
    dirMux[9][4] = 350;

    dirMux[0][5] = 214; // sensor farthest from middle
    dirMux[1][5] = 216;
    dirMux[2][5] = 217;
    dirMux[3][5] = 215;
    dirMux[4][5] = 212;
    dirMux[5][5] = 211;
    dirMux[6][5] = 210;
    dirMux[7][5] = 213;
    dirMux[8][5] = 230;
    dirMux[9][5] = 220;

    dirMux[0][6] = 204; // sensor farthest from middle
    dirMux[1][6] = 206;
    dirMux[2][6] = 207;
    dirMux[3][6] = 205;
    dirMux[4][6] = 202;
    dirMux[5][6] = 201;
    dirMux[6][6] = 200;
    dirMux[7][6] = 203;
    dirMux[8][6] = 240;
    dirMux[9][6] = 250;

    dirMux[0][7] = 154; // sensor farthest from middle
    dirMux[1][7] = 156;
    dirMux[2][7] = 157;
    dirMux[3][7] = 155;
    dirMux[4][7] = 152;
    dirMux[5][7] = 151;
    dirMux[6][7] = 150;
    dirMux[7][7] = 153;
    dirMux[8][7] = 120;
    dirMux[9][7] = 260;

    dirMux[0][8] = 114; // sensor farthest from middle
    dirMux[1][8] = 116;
    dirMux[2][8] = 117;
    dirMux[3][8] = 115;
    dirMux[4][8] = 112;
    dirMux[5][8] = 111;
    dirMux[6][8] = 110;
    dirMux[7][8] = 113;
    dirMux[8][8] = 130;
    dirMux[9][8] = 170;

    dirMux[0][9] = 164; // sensor farthest from middleiiiiiiiiiiiiiiiiiiiiiii
    dirMux[1][9] = 166;
    dirMux[2][9] = 167;
    dirMux[3][9] = 165;
    dirMux[4][9] = 162;
    dirMux[5][9] = 161;
    dirMux[6][9] = 160;
    dirMux[7][9] = 163;
    dirMux[8][9] = 100;
    dirMux[9][9] = 140;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateSensors()
// Called from the BLE FreeRTOS task (runs on a separate ESP32 core).
// Scans all 100 sensors through the mux chain, then rotates the raw result
// according to the board orientation setting before writing it to sensorUpdate.
//
// Board rotation cases (BleChess.getBoardRotation()):
//   0  -> no rotation (standard orientation)
//   1  -> 90-degree clockwise rotation
//  -1  -> 180-degree rotation
//   2  -> 270-degree clockwise rotation (90-degree counter-clockwise)
//
// The four corner cells (0,0), (0,9), (9,0), (9,9) are always forced to true
// (empty) because physical sensors at these locations are not used.
// ─────────────────────────────────────────────────────────────────────────────
void updateSensors()
{
    bool temp[10][10];
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            // Decode the 3-digit mux address for this cell
            int muxOut = dirMux[i][j] / 100;       // Get the hundreds digit
            int mux16 = (dirMux[i][j] % 100) / 10; // Get the tens digit
            int mux8 = dirMux[i][j] % 10;          // Get the units digit

            // Set the 3-bit address on the 16-channel mux selector pins
            digitalWrite(mux16_0, mux16 & 0b00000001);
            digitalWrite(mux16_1, (mux16 >> 1) & 0b00000001);
            digitalWrite(mux16_2, (mux16 >> 2) & 0b00000001);

            // Set the 3-bit address on the 8-channel mux selector pins
            digitalWrite(mux8_0, mux8 & 0b00000001);
            digitalWrite(mux8_1, (mux8 >> 1) & 0b00000001);
            digitalWrite(mux8_2, (mux8 >> 2) & 0b00000001);
            delayMicroseconds(1); // Reduced delay

            // Read the selected sensor via the appropriate mux output pin
            temp[i][j] = digitalRead(muxesOut[muxOut]);
        }
    }

    // Apply board rotation transform before writing to the shared sensorUpdate buffer
    switch (BleChess.getBoardRotation())
    {
    case 0:
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[i][j] = temp[i][j];
            }
        }
        break;
    case 1: // 90
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[10 - j - 1][i] = temp[i][j];
            }
        }
        break;
    case -1: // 180
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[10 - i - 1][10 - j - 1] = temp[i][j];
            }
        }
        break;
    case 2: // 270
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[j][10 - i - 1] = temp[i][j];
            }
        }
        break;
    default:
        // Unknown rotation: mark all cells as empty to avoid false positives
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[i][j] = true;
            }
        }
        break;
    }

    // Force the three unused corner cells to "empty" (true).
    // sensorUpdate[0][9] is the mechanical calibration reference sensor and is NOT forced.
    sensorUpdate[0][0] = true;
    // sensorUpdate[0][9] = false; THIS IS THE CALIBRATION ONE
    sensorUpdate[9][0] = true;
    sensorUpdate[9][9] = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// readRawSensors()
// Copies the current sensorUpdate[] snapshot into the caller-supplied array.
// This provides a consistent point-in-time view of the sensor state to callers
// that need to avoid partial updates while updateSensors() runs concurrently.
// ─────────────────────────────────────────────────────────────────────────────
void readRawSensors(bool sensorsValue[10][10])
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            sensorsValue[i][j] = sensorUpdate[i][j];
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// detectChessBoard()
// Reads the sensor matrix with anti-bouncing filtering.
//
// Performs up to 3 attempts to obtain a stable reading: two successive reads
// must agree on every cell before the result is accepted.  The delay between
// reads grows with each attempt (5ms, 10ms, 15ms) to give slow-transitioning
// sensors time to settle.
//
// sensVal is written with the final stable sensor state (true = empty).
// ─────────────────────────────────────────────────────────────────────────────
void detectChessBoard(bool sensVal[10][10])
{
    // mode  = false; normal Mode
    // mode = true; antiBonuncing Mode
    bool sensValAux[10][10];
    bool flag = true;
    int attempts = 0;
    int targetAttempts = 3;

    do
    {
        attempts++;
        flag = true;

        // First read into sensVal
        readRawSensors(sensVal);

        // Delay grows with each retry to allow sensors to settle
        int timeBetweenReadings = (5 * attempts);
        delay(timeBetweenReadings);

        // Second read into auxiliary buffer
        readRawSensors(sensValAux);

        // Compare the two reads; any mismatch triggers another attempt
        for (int i = 1; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                if (sensVal[i][j] != sensValAux[i][j])
                {
                    flag = false;
                    Serial.println("Error en la lectura de sensores");
                }
            }
        }

    } while (!flag && attempts <= targetAttempts);
}
