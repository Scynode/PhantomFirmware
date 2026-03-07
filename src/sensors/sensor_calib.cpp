/*
 * sensor_calib.cpp
 * Sensor position calibration routines.
 *
 * There are two complementary calibration processes:
 *
 *  1. sensorOffsetCalib()
 *     Called after mechanical homing to fine-tune the magnet's resting position.
 *     It moves the magnet slightly away from the current position along the vector
 *     toward the board centre (origin), compensating for any residual offset after
 *     stall-guard homing.
 *
 *  2. sensorsCalibration()
 *     Iterative "star-pattern" calibration: the magnet approaches the target square
 *     from 6 equally-spaced directions (every 60°) and records the position error
 *     each time. The average error is the offset. In correction mode (mode==1) the
 *     motor position registers are adjusted by 1.75× the measured error, and the
 *     procedure repeats until the error is within ±0.5mm or 5 iterations are done.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "sensor_state.h"
#include "../mech/mech_state.h"

// ─────────────────────────────────────────────────────────────────────────────
// sensorOffsetCalib()
// Fine-tunes the magnet position after mechanical homing.
//
// Algorithm:
//  1. Compute the current logical XY position from stepper encoders.
//  2. Build a unit vector from the current position toward the origin (0, 0).
//  3. Move 10mm along that vector (rawMovement with magnet=-4 → speed 200 mm/s).
//
// This nudges the carriage away from the end-stop corner toward the centre,
// correcting for any systematic overshoot from the stall-guard calibration.
// ─────────────────────────────────────────────────────────────────────────────
void sensorOffsetCalib()
{
    // Calculate current position from stepper motor step counters.
    // These forward-kinematics equations convert CoreXY step counts to mm.
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;

    // Calculate the vector from the current position toward the origin (0, 0)
    float vectorX = -currentXposition; // negative to go in the opposite direction toward the origin
    float vectorY = -currentYposition; // negative to go in the opposite direction toward the origin

    // Calculate the magnitude of the current vector from the current position to the origin
    float magnitude = sqrt(vectorX * vectorX + vectorY * vectorY);

    // Calculate the coordinates of the point 10 mm in the direction of the origin from the current position
    float xprima = currentXposition + (10 * vectorX / magnitude);
    float yprima = currentYposition + (10 * vectorY / magnitude);

    // Move the system to coordinates xprima, yprima (magnet=-4 uses speed 200 mm/s, raw XY coords)
    rawMovement(xprima, yprima, -4, currentXposition, currentYposition);
}

// ─────────────────────────────────────────────────────────────────────────────
// sensorsCalibration()
// Performs a star-pattern measurement to determine the position error at a
// given board square, and optionally applies a correction to the step counters.
//
// Parameters:
//   rowEnd   - target row in the 10x10 grid
//   colEnd   - target column in the 10x10 grid
//   mode     - 0: measure only (write result to totalX/totalY)
//              1: measure + apply correction + repeat until error < 0.5mm
//   totalX   - output: mean X error in mm (positive = carriage is too far right)
//   totalY   - output: mean Y error in mm
//
// How it works:
//  - The magnet is activated (electromagnet 1) and the carriage approaches the
//    target square from 6 equally-spaced angles (0°, 60°, 120°, 180°, 240°, 300°)
//    at a radius of 50mm.
//  - For each approach, rawMovement with magnet=-1 is used; this triggers the
//    "sensing" flag which stops motion as soon as the piece sensor fires.  The
//    residual position error (distance from the intended stop point) is recorded.
//  - The mean error across all 6 approaches is the calibration offset.
//  - In mode 1, the step-counter origin is shifted by 1.75× the mean error and
//    the measurement repeats until ≤ 0.5mm error is achieved or 5 tries expire.
// ─────────────────────────────────────────────────────────────────────────────
void sensorsCalibration(int rowEnd, int colEnd, int mode, float &totalX, float &totalY)
{
    // mode 0 -> sensor calibration
    // mode 1 -> sensor calibration with correction
    float posX = 0;
    float posY = 0;
    float anguloPizza = 60;         // Angular step between approach directions (degrees)
    float electroOffsetX = 0;
    float electroOffsetY = 0;
    int attempts = 0;

    // Convert grid coordinates to physical mm coordinates (origin = board centre)
    float coordXEnd = (50 * rowEnd) - 225;
    float coordYEnd = (-50 * colEnd) + 225;

    // Apply the physical offset for electromagnet 1 (upper-left quadrant magnet)
    calculateOffsets(1, electroOffsetX, electroOffsetY);
    coordXEnd = coordXEnd + electroOffsetX;
    coordYEnd = coordYEnd + electroOffsetY;

    // Activate electromagnet 1 at low power (25%) to gently attract the test piece
    activateElectromagnetV2(1, 25);
    delay(250);

    do
    {
        totalX = 0;
        totalY = 0;

        // Approach from 6 equally-spaced directions around the target square
        for (int angulo = 0; angulo < 360; angulo += anguloPizza)
        {
            float radianes = angulo * PI / 180.0;
            // Approach start point on a 50mm radius circle around the target
            float x = coordXEnd + 50 * cos(radianes);
            float y = coordYEnd + 50 * sin(radianes);

            // Move to approach start point (raw XY, no sensing, speed 50 mm/s)
            rawMovement(x, y, -2, posX, posY);

            // Approach target; magnet=-1 enables sensing: stops when sensor fires
            rawMovement(rowEnd, colEnd, -1, posX, posY);

            // Accumulate the stopping error returned by rawMovement
            totalX = totalX + posX;
            totalY = totalY + posY;
        }

        // Average the errors over all 6 directions
        totalX = totalX / (360 / anguloPizza);
        totalY = totalY / (360 / anguloPizza);

        Serial.print("error x");
        Serial.print(totalX);
        Serial.print("  error y");
        Serial.println(totalY);

        if (mode == 1)
        {
            attempts++;
            Serial.println("Medida Correctiva: ");

            // Apply a 1.75× amplified correction to compensate for mechanical elasticity
            float correctX = totalX * 1.75;
            float correctY = totalY * 1.75;

            Serial.print("correct x");
            Serial.print(correctX);
            Serial.print("  correct  y");
            Serial.println(correctY);

            // Shift the position reference by the correction amount
            rawMovement(correctX, correctY, -2, posX, posY);

            // Reset step counters so the new position is treated as the origin
            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
        }
        Serial.println("============================================");

    // Repeat in correction mode until error is within 0.5 mm or 5 attempts reached
    } while (mode == 1 && attempts <= 5 && (totalX <= -0.5 || totalX >= 0.5 || totalY <= -0.5 || totalY >= 0.5));

    deactivateAllMagnets();
}
