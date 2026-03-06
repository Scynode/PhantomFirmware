/*
 * motion.cpp
 * Core motion-execution routines: acceleration ramping, raw point-to-point
 * moves, StallGuard-based homing moves, and supporting ISRs.
 *
 * ACCELERATION RAMP ALGORITHM (accelRampV3):
 * ───────────────────────────────────────────
 * An S-curve velocity profile is computed using a pair of sigmoid functions:
 *   y1 = 1 / (1 + e^(-a1*(t - c1)))   → ramp-up half
 *   y2 = 1 / (1 + e^(-a2*(t - c2)))   → ramp-down half
 *   y  = |y1 - y2|                     → combined bell-shaped envelope
 *
 * The per-point speed is multiplied by y(t) so that the carriage accelerates
 * smoothly from rest, travels at full speed in the middle, then decelerates.
 * Parameters a1/a2 (slope) and c1/c2 (inflection time) are tuned dynamically
 * based on the total segment duration:
 *   Short segments (< 0.25 s): a=75, c1=0.03  (steep, quick ramp)
 *   Long  segments (≥ 0.25 s): a=35, c1 found dynamically so that the
 *     initial speed contribution is < 5000 steps/s (gentle start).
 *
 * RAWMOVEMENT COORDINATE SYSTEM:
 * ───────────────────────────────
 * Grid coordinates (row, col) map to physical mm via:
 *   X_mm = 50*row - 225   (centre of board = col 4.5 → 0mm; edge = ±225mm)
 *   Y_mm = -50*col + 225
 *
 * Magnet codes:
 *   magnet ≥ 1  : activate specified electromagnet, use grid coordinates
 *   magnet = -1 : like ≥1 but enable sensor-stop (stops when piece detected)
 *   magnet = -2 : raw XY in mm, speed 50 mm/s
 *   magnet = -3 : raw XY in mm, speed 300 mm/s
 *   magnet = -4 : raw XY in mm, speed 200 mm/s
 *
 * STALLGUARD (rawMovementStallGuard):
 * ─────────────────────────────────────
 * The TMC2209 SG_RESULT register decreases as the motor load increases.
 * During homing the carriage is driven toward a mechanical end-stop at high
 * speed.  The ISR (moveRawStallGuard) runs at ~2.5 kHz to advance the motors,
 * while the main loop samples SG_RESULT and computes the delta from the
 * previous sample.  A large delta (> threshold) indicates an abrupt load
 * spike caused by hitting the end-stop.
 *
 * MAGNET TIMER ISR (timeEnabled):
 * ─────────────────────────────────
 * A hardware timer fires every 10 seconds to check whether the electromagnet
 * has been on for more than 60 seconds.  If so, all magnets are cut and the
 * motors are disabled to prevent coil burnout.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "mech_state.h"
#include "../sensors/sensor_state.h"
#include <AccelStepper.h>

// ─────────────────────────────────────────────────────────────────────────────
// setCurrentPosition()
// Sets the logical XY position of the carriage by back-computing the
// corresponding step counts for each motor and writing them into the
// AccelStepper position registers.
//
// This is used after calibration to declare "I am now at position (x, y) mm"
// without actually moving the motors.
// ─────────────────────────────────────────────────────────────────────────────
void setCurrentPosition(float x, float y)
{
    // Always use 128 microsteps for the finest position resolution
    driverMicroSteps = 128;
    int m1 = round(-(driverMicroSteps * 5) * (x - y));
    int m2 = round(-(driverMicroSteps * 5) * (-x - y));

    stepper1.setCurrentPosition(m1);
    stepper2.setCurrentPosition(m2);
}

// ─────────────────────────────────────────────────────────────────────────────
// timeEnabled()  [IRAM_ATTR — runs from IRAM to survive cache misses]
// Hardware timer ISR — fires every 10 seconds.
// Cuts electromagnet power and disables motor outputs if the magnet has been
// continuously energised for more than 60 seconds.
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR timeEnabled()
{
    // Compare elapsed time with 60000 ms (60 seconds)
    if (millis() - timerMagnets > 60000) // 60000
    {
        deactivateAllMagnets(); // Deactivate all electromagnets
        stepper1.disableOutputs();
        stepper2.disableOutputs();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// rawMovement()
// Moves the carriage to a target position with coordinated two-axis motion.
//
// The dominant axis (larger step count) drives via AccelStepper::run() or
// runSpeed(), while the secondary axis is advanced in proportion each time the
// primary axis completes a step (Bresenham-style synchronisation).
//
// When sensingFlag is true (magnet==-1) the loop polls readRawSensors() every
// iteration and aborts as soon as the target square's sensor reads "occupied"
// (false), meaning the piece has arrived under the magnet.
//
// On return, lastRawpositionX/Y hold the residual position error in mm
// (how far the magnet stopped from the intended end point).
// ─────────────────────────────────────────────────────────────────────────────
void rawMovement(float rowEnd, float colEnd, int magnet, float &lastRawpositionX, float &lastRawpositionY)
{
    // electromagnets x, y
    // sensorMatrix filaend colend

    float coordXEnd = 0;
    float coordYEnd = 0;
    bool sensingFlag = false;

    float electroOffsetX = 0;
    float electroOffsetY = 0;
    float numberofSteps[2] = {0, 0};
    int setSpeedTesting = 300;
    double timeToPosition = 0;
    bool sensorsRM[10][10];

    int speedperStep[2] = {0, 0};

    if (magnet == -2)
    {
        // Raw XY mode at slow speed: rowEnd/colEnd are physical mm coordinates
        setSpeedTesting = 50;
        coordXEnd = rowEnd;
        coordYEnd = colEnd;
    }
    else if (magnet == -3)
    {
        // Raw XY mode at normal speed
        setSpeedTesting = 300;
        coordXEnd = rowEnd;
        coordYEnd = colEnd;
    }
    else if (magnet == -4)
    {
        // Raw XY mode at medium speed (used after stall-guard calibration)
        setSpeedTesting = 200;
        coordXEnd = rowEnd;
        coordYEnd = colEnd;
    }
    else
    {
        if (magnet == -1)
        {
            // Sensor-stop mode: use electromagnet 1, enable sensing, slow speed
            magnet = 1;
            sensingFlag = true;
            setSpeedTesting = 15;
        }
        // Convert grid (row, col) to physical mm
        coordXEnd = (50 * rowEnd) - 225;
        coordYEnd = (-50 * colEnd) + 225;
        calculateOffsets(magnet, electroOffsetX, electroOffsetY);
        coordXEnd = coordXEnd + electroOffsetX;
        coordYEnd = coordYEnd + electroOffsetY;
    }

    // -------------------------------------Inverse Kinematics------------------------------------------
    configAndCurrentPosManager(setSpeedTesting, driverMicroSteps);

#ifdef MODELFINAL
    numberofSteps[0] = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
#endif
#ifdef MODELEFRA
    numberofSteps[0] = round((driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round((driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
#endif

    // -------------------------------------Inverse Kinematics------------------------------------------

    // --------------------------------------------Speed------------------------------------------

#ifdef MODELFINAL
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;
#endif
#ifdef MODELEFRA
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * (driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / (driverMicroSteps * 5)) + currentYposition;
#endif

    double deltaX = coordXEnd - currentXposition;
    double deltaY = coordYEnd - currentYposition;

    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    timeToPosition = distance / setSpeedTesting; // The integer part of the division is the time to reach the position

    speedperStep[0] = ((numberofSteps[0] - stepper1.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    speedperStep[1] = ((numberofSteps[1] - stepper2.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    // --------------------------------------------Speed------------------------------------------

    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    stepper1.setMaxSpeed(10000000); // Prevent interference with speed calculations
    stepper2.setMaxSpeed(10000000); // Prevent interference with speed calculations
    float stepsM1 = numberofSteps[0] - stepper1.currentPosition();
    float stepsM2 = numberofSteps[1] - stepper2.currentPosition();

    float initialPosition1 = stepper1.currentPosition();
    float initialPosition2 = stepper2.currentPosition();

    float deltaXSteps = abs(stepsM1);
    float deltaYSteps = abs(stepsM2);

    // Bresenham ratio: secondary axis steps per primary axis step
    double stepIsDue = deltaXSteps > deltaYSteps ? static_cast<double>(deltaYSteps) / deltaXSteps : static_cast<double>(deltaXSteps) / deltaYSteps;
    double stepSum = 0.0;
    float dirDeGiro1 = stepsM1 > 0 ? 1 : -1;
    float dirDeGiro2 = stepsM2 > 0 ? 1 : -1;

    bool sensor = true;

    if (deltaXSteps > deltaYSteps)
    {
        // M1 is the dominant axis
        stepper1.setAcceleration(speedperStep[0] * 2);
        stepper1.setMaxSpeed(speedperStep[0] * 1);
        stepper1.moveTo(numberofSteps[0]);

        while ((abs(stepper1.currentPosition() - numberofSteps[0]) > 0) && sensor)
        {
            if (sensingFlag == true)
            {
                readRawSensors(sensorsRM);
                sensor = sensorsRM[int(rowEnd)][int(colEnd)];
            }

            stepper1.run();

            if (abs(stepper1.currentPosition() - initialPosition1) >= 1 && sensor)
            {
                stepSum += stepIsDue;
                initialPosition1 = stepper1.currentPosition();
                while (stepSum >= 1 && sensor)
                {
                    if (sensingFlag == true)
                    {
                        readRawSensors(sensorsRM);
                        sensor = sensorsRM[int(rowEnd)][int(colEnd)];
                    }

                    stepper2.setSpeed(1000000.0 * dirDeGiro2);
                    stepper2.runSpeed();
                    stepSum -= 1;
                }
            }
        }
    }
    else
    {
        // M2 is the dominant axis
        stepper2.setAcceleration(speedperStep[1] * 2);
        stepper2.setMaxSpeed(speedperStep[1] * 1);
        stepper2.moveTo(numberofSteps[1]);

        while ((abs(stepper2.currentPosition() - numberofSteps[1]) > 0) && sensor)
        {
            if (sensingFlag == true)
            {
                readRawSensors(sensorsRM);
                sensor = sensorsRM[int(rowEnd)][int(colEnd)];
            }

            stepper2.run();

            if (abs(stepper2.currentPosition() - initialPosition2) >= 1 && sensor)
            {
                stepSum += stepIsDue;
                initialPosition2 = stepper2.currentPosition();
                while (stepSum >= 1 && sensor)
                {
                    if (sensingFlag == true)
                    {
                        readRawSensors(sensorsRM);
                        sensor = sensorsRM[int(rowEnd)][int(colEnd)];
                    }
                    stepper1.setSpeed(1000000.0 * dirDeGiro1);
                    stepper1.runSpeed();
                    stepSum -= 1;
                }
            }
        }
    }

    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------

    // Compute actual final position and residual error
    lastRawpositionY = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    lastRawpositionX = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + lastRawpositionY;

    // Return residual error (actual position minus intended end point)
    lastRawpositionY = lastRawpositionY - coordYEnd;
    lastRawpositionX = lastRawpositionX - coordXEnd;

    // Reduce current to hold level after move completes (thermal protection)
    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
}

// ─────────────────────────────────────────────────────────────────────────────
// rawMovementStallGuard()
// Drives the carriage toward (coordXEnd, coordYEnd) while monitoring the
// TMC2209 StallGuard register on each axis for a load spike that indicates
// an end-stop collision.
//
// activeMotors controls which motors are enabled during this move:
//   1 → only driver (M1) active (driver2 toff=0)
//   2 → only driver2 (M2) active (driver toff=0)
//   other → both motors active
//
// The ISR moveRawStallGuard is activated 50ms into the move to allow the
// motors to spin up before stall detection begins.  A fixed window of
// 320 steps must be covered before a stall is declared.
//
// On return, threshold and thresholdD2 are updated with the most-frequent
// (reliable) SG delta seen during the move, which the caller stores for
// future use as the stall detection threshold.
// ─────────────────────────────────────────────────────────────────────────────
void rawMovementStallGuard(float coordXEnd, float coordYEnd, float setSpeedTesting, int activeMotors, int &threshold, int &thresholdD2)
{
    stepper1.setCurrentPosition(stepper1.currentPosition());
    stepper2.setCurrentPosition(stepper2.currentPosition());
    int speedperStep[2] = {0, 0};
    float numberofSteps[2] = {0, 0};

    // Enable only the requested motor(s)
    if (activeMotors == 1)
    {
        driver2.toff(0);
        driver.toff(1);
    }
    else if (activeMotors == 2)
    {
        driver.toff(0);
        driver2.toff(1);
    }
    else
    {
        driver.toff(1);
        driver2.toff(1);
    }

    // -------------------------------------Inverse Kinematics------------------------------------------
    numberofSteps[0] = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
    // -------------------------------------Inverse Kinematics------------------------------------------

    //  --------------------------------------------Speed------------------------------------------
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;

    double deltaX = coordXEnd - currentXposition;
    double deltaY = coordYEnd - currentYposition;

    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    double timeToPosition = distance / setSpeedTesting; // The integer part of the division is the time to reach the position

    speedperStep[0] = ((numberofSteps[0] - stepper1.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    speedperStep[1] = ((numberofSteps[1] - stepper2.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    int stepsarstart = 0;
    int stepsarstart2 = 0;

    stepper1.setAcceleration(speedperStep[0] * 1);
    stepper1.setSpeed(speedperStep[0] * 1);
    stepper1.moveTo(numberofSteps[0]);

    stepper2.setAcceleration(speedperStep[1] * 1);
    stepper2.setSpeed(speedperStep[1] * 1);
    stepper2.moveTo(numberofSteps[1]);

    int stall_data_prev_m2 = driver2.SG_RESULT();
    int stall_data_prev_m1 = driver.SG_RESULT();

    const int maxValueCount = 100;         // Maximum number of different values expected
    int values1[maxValueCount] = {0};      // Array to store changeInCalib1 values
    int values2[maxValueCount] = {0};      // Array to store changeInCalib2 values
    int frequencies1[maxValueCount] = {0}; // Array to store the frequency of each changeInCalib1 value
    int frequencies2[maxValueCount] = {0}; // Array to store the frequency of each changeInCalib2 value
    int valueCount1 = 0;                   // Number of distinct values found for changeInCalib1
    int valueCount2 = 0;                   // Number of distinct values found for changeInCalib2

    int mostFrequentValue1 = 0; // The most frequent value for changeInCalib1
    int mostFrequentValue2 = 0; // The most frequent value for changeInCalib2
    unsigned long timeBeforeMotors = millis();
    bool flagOnce = false;
    do
    {
        int distToReadStall = 320; // 240;
        int current_sg_result2 = driver2.SG_RESULT();
        int current_sg_result1 = driver.SG_RESULT();
        int changeInCalib2 = abs(stall_data_prev_m2 - current_sg_result2);
        int changeInCalib1 = abs(stall_data_prev_m1 - current_sg_result1);

        // Wait 50ms for motors to spin up, then start the ISR timer
        if (timeBeforeMotors + 50 < millis() && !flagOnce) //
        {
            if (timer1H == NULL)
            {
                timer1H = timerBegin(3, 2, true);
                timerAttachInterrupt(timer1H, &moveRawStallGuard, true);
                timerAlarmWrite(timer1H, 400, true);
            }
            timerAlarmEnable(timer1H);
            stepsarstart = stepper1.currentPosition();
            stepsarstart2 = stepper2.currentPosition();
            flagOnce = true;
        }

        // Track SG delta histogram for calibrating the threshold
        bool found1 = false;
        for (int i = 0; i < valueCount1; i++)
        {
            if (values1[i] == changeInCalib1)
            {
                frequencies1[i]++;
                found1 = true;
                break;
            }
        }

        if (!found1 && valueCount1 < maxValueCount)
        {
            values1[valueCount1] = changeInCalib1;
            frequencies1[valueCount1] = 1;
            valueCount1++;
        }

        bool found2 = false;
        for (int i = 0; i < valueCount2; i++)
        {
            if (values2[i] == changeInCalib2)
            {
                frequencies2[i]++;
                found2 = true;
                break;
            }
        }

        if (!found2 && valueCount2 < maxValueCount)
        {
            values2[valueCount2] = changeInCalib2;
            frequencies2[valueCount2] = 1;
            valueCount2++;
        }

        // Check if the required minimum travel distance has been covered
        bool distFlag = false;
        if (activeMotors == 1)
        {
            if (abs(stepper1.currentPosition() - stepsarstart) > distToReadStall)
            {
                distFlag = true;
            }
        }
        else if (activeMotors == 2)
        {
            if (abs(stepper2.currentPosition() - stepsarstart2) > distToReadStall)
            {
                distFlag = true;
            }
        }
        else
        {
            if ((abs(stepper1.currentPosition() - stepsarstart) > distToReadStall) && (abs(stepper2.currentPosition() - stepsarstart2) > distToReadStall))
            {
                distFlag = true;
            }
        }

        // If beyond the minimum distance and a stall spike is detected, stop
        if (distFlag && (changeInCalib1 > threshold || changeInCalib2 > thresholdD2))
        {
            break;
        }
        stall_data_prev_m2 = current_sg_result2;
        stall_data_prev_m1 = current_sg_result1;
    } while (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0);

    // Disable and stop the timer
    driver2.toff(0);
    driver.toff(0);

    if (timer1H != NULL)
    {
        timerAlarmDisable(timer1H);
        timerDetachInterrupt(timer1H);
        timerEnd(timer1H);
        timer1H = NULL; // Ensure the pointer is NULL after freeing the timer
    }

    // Find the most common (reliable) SG delta seen during the move — this is
    // the "normal running" SG delta and becomes the new baseline threshold.
    for (int i = 0; i < valueCount1; i++)
    {
        if (frequencies1[i] > 5 && values1[i] > mostFrequentValue1)
        {
            mostFrequentValue1 = values1[i];
        }
    }
    for (int i = 0; i < valueCount2; i++)
    {
        if (frequencies2[i] > 5 && values2[i] > mostFrequentValue2)
        {
            mostFrequentValue2 = values2[i];
        }
    }

    threshold = mostFrequentValue1;
    thresholdD2 = mostFrequentValue2;
}

// ─────────────────────────────────────────────────────────────────────────────
// moveRawStallGuard()  [IRAM_ATTR — runs from IRAM to survive cache misses]
// Hardware timer ISR triggered at ~2.5 kHz during rawMovementStallGuard().
// Advances both AccelStepper objects toward their target positions using the
// interrupt-safe runSpeedToPosition() method.
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR moveRawStallGuard()
{
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
}

// ─────────────────────────────────────────────────────────────────────────────
// accelRampV3()
// Full trajectory execution with S-curve velocity profiling and electromagnet
// management.
//
// Steps:
//  1. Deduplicate adjacent identical waypoints in the trajectory array.
//  2. Apply board-rotation transform to all waypoints.
//  3. Select the best electromagnet(s) for the path (by coverage analysis).
//  4. Apply electromagnet XY offset to all waypoints.
//  5. Compute inverse kinematics (XY mm → M1/M2 steps) for every waypoint.
//  6. Compute constant-speed per-segment speeds.
//  7. Detect direction-change waypoints and divide the path into segments.
//  8. Apply an S-curve (sigmoid) speed envelope to each segment.
//  9. Execute the motion: alternate between the dominant axis driving and
//     the secondary axis being stepped in proportion (Bresenham).
//
// Safety: if a move takes > 30 seconds the motors are halted and disabled
// to prevent thermal damage.
// ─────────────────────────────────────────────────────────────────────────────
void accelRampV3(float **finalTrajectory, int numPointsFinal, double setSpeedbyUser)
{
    configAndCurrentPosManager(setSpeedbyUser, driverMicroSteps);
    //===============================================DELETE TRAJECTORY DUPLICATES===============================================
    int size = numPointsFinal;
    int removedCount = 0;
    for (int i = 0; i < size; i++)
    {
        for (int j = i + 1; j < size;)
        {
            if (finalTrajectory[i][0] == finalTrajectory[j][0] && finalTrajectory[i][1] == finalTrajectory[j][1])
            {
                // Shift all subsequent elements one position forward
                for (int k = j; k < size - 1; k++)
                {
                    finalTrajectory[k][0] = finalTrajectory[k + 1][0];
                    finalTrajectory[k][1] = finalTrajectory[k + 1][1];
                }
                size--;         // Reduce the array size because a duplicate was removed
                removedCount++; // Increment the count of removed elements
            }
            else
            {
                j++; // Only increment j if no element was removed, to avoid skipping checks
            }
        }
    }
    numPointsFinal = numPointsFinal - removedCount;

    //===============================================DELETE TRAJECTORY DUPLICATES===============================================
    double **speedperStep = nullptr;
    int **numberofSteps = nullptr;
    double *timeToPosition = new double[numPointsFinal]; // Allocate space for numPointsFinal elements of type double

    numberofSteps = new int *[numPointsFinal];
    for (int i = 0; i < numPointsFinal; i++)
    {
        numberofSteps[i] = new int[2];
    }

    speedperStep = new double *[numPointsFinal];
    for (int i = 0; i < numPointsFinal; i++)
    {
        speedperStep[i] = new double[2];
    }
    //---------------------------------------Mechanism Tranform----------------------------------------

    for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
    {
        speedperStep[i][0] = finalTrajectory[i][0];
        speedperStep[i][1] = finalTrajectory[i][1];
    }
    switch (BleChess.getBoardRotation())
    {
    case 0:
        break;
    case 1:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            finalTrajectory[i][0] = speedperStep[i][1] * -1;
            ;
            finalTrajectory[i][1] = speedperStep[i][0];
            ;
        }
        break;
    case -1:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            finalTrajectory[i][0] = speedperStep[i][0] * -1;
            finalTrajectory[i][1] = speedperStep[i][1] * -1;
        }
        break;
    case 2:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            finalTrajectory[i][0] = speedperStep[i][1];
            finalTrajectory[i][1] = speedperStep[i][0] * -1;
        }
        break;
    default:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            finalTrajectory[i][0] = speedperStep[i][0];
            finalTrajectory[i][1] = speedperStep[i][1];
        }
        break;
    }
    //---------------------------------------Mechanism Tranform----------------------------------------

    //===============================================ELECTROMAGNET SELECTION BY PATH=======================================================
    int electromagnetCount[5] = {0};
    bool electromagnetInit[5] = {false};
    bool electromagnetEnd[5] = {false};
    int activeElectromagnets[5] = {0};
    int activeElectroCount = 0;
    // Define conditions for each electromagnet [left, top, bottom, right]
    int conditions[5][4] = {
        {},                     // Position 0 is not used, so indices match electromagnet numbers
        {-225, 197, -192, 182}, // Electromagnet 1
        {-214, 234, -152, 192}, // Electromagnet 2
        {-178, 197, -192, 228}, // Electromagnet 3
        {-214, 152, -234, 192}  // Electromagnet 4
    };

    for (int i = 0; i < numPointsFinal; i++) // Iterate over each trajectory point to check which electromagnet meets all conditions. If it does, increment that electromagnet's counter.
    {
        for (int j = 1; j <= 4; j++) // iterate over conditions for each electromagnet
        {
            if (finalTrajectory[i][0] >= conditions[j][0] && finalTrajectory[i][1] <= conditions[j][1] &&
                finalTrajectory[i][1] >= conditions[j][2] && finalTrajectory[i][0] <= conditions[j][3])
            {
                electromagnetCount[j]++;
                if (i == 0)
                    electromagnetInit[j] = true;
                if (i == numPointsFinal - 1)
                    electromagnetEnd[j] = true;
            }
        }
    }

    for (int i = 1; i <= 4; i++) // If an electromagnet can travel the entire trajectory, add it to the active electromagnets array
    {
        if (electromagnetCount[i] == numPointsFinal)
        {
            activeElectromagnets[activeElectroCount++] = i;
        }
    }
    //===============================================ELECTROMAGNET SELECTION BY PATH=======================================================

    //=======================================================MOVEMENT TYPE=======================================================
    float electroOffsetX = 0;
    float electroOffsetY = 0;

#ifdef MODELFINAL

    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;
#endif
#ifdef MODELEFRA
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * (driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / (driverMicroSteps * 5)) + currentYposition;
#endif

    int activeElectro = -1;
    int activeElectroInit = -1;
    int activeElectroEnd = -1;

    double minDistance = std::numeric_limits<double>::max();
    //--------------------------------------------------Multiple Electromagnets Movement-------------------------------------------------------
    if (activeElectroCount == 0) // Complex movement
    {

        for (int i = 1; i <= 4; i++) // Find the electromagnet closest to the start point
        {
            if (electromagnetCount[i] < minDistance && electromagnetInit[i] == true)
            {
                minDistance = electromagnetCount[i];
                activeElectroInit = i;
            }
        }

        minDistance = std::numeric_limits<double>::max();
        for (int i = 1; i <= 4; i++) // Find the electromagnet closest to the end point
        {
            if (electromagnetCount[i] < minDistance && electromagnetEnd[i] == true)
            {
                minDistance = electromagnetCount[i];
                activeElectroEnd = i;
            }
        }

        calculateOffsets(activeElectroInit, electroOffsetX, electroOffsetY);
        for (int i = 0; i < (numPointsFinal - electromagnetCount[activeElectroEnd]); i++) // Apply offset to all points in trajectory 1 and inverse kinematics
        {
            finalTrajectory[i][0] = finalTrajectory[i][0] + electroOffsetX;
            finalTrajectory[i][1] = finalTrajectory[i][1] + electroOffsetY;
        }

        calculateOffsets(activeElectroEnd, electroOffsetX, electroOffsetY);
        for (int i = (numPointsFinal - electromagnetCount[activeElectroEnd]); i < (numPointsFinal); i++) // Apply offset to all points in trajectory 2 and inverse kinematics
        {
            finalTrajectory[i][0] = finalTrajectory[i][0] + electroOffsetX;
            finalTrajectory[i][1] = finalTrajectory[i][1] + electroOffsetY;
        }
    }
    //--------------------------------------------------Multiple Electromagnets Movement-------------------------------------------------------

    //--------------------------------------------------One Electromagnets Movement-------------------------------------------------------
    else // Simple movement
    {

        for (int i = 0; i < activeElectroCount; i++) // Find the electromagnet closest to the start point
        {
            double deltaX, deltaY, distance;

            calculateOffsets(activeElectromagnets[i], electroOffsetX, electroOffsetY);
            deltaX = (finalTrajectory[0][0] + electroOffsetX) - currentXposition;
            deltaY = (finalTrajectory[0][1] + electroOffsetY) - currentYposition;
            distance = sqrt(deltaX * deltaX + deltaY * deltaY); // Calculate the distance between the start point and the electromagnet

            if (distance < minDistance) // If the distance is less than the minimum distance, update the active electromagnet
            {
                minDistance = distance;
                activeElectro = activeElectromagnets[i];
            }
        }

        calculateOffsets(activeElectro, electroOffsetX, electroOffsetY);
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            finalTrajectory[i][0] = finalTrajectory[i][0] + electroOffsetX;
            finalTrajectory[i][1] = finalTrajectory[i][1] + electroOffsetY;
        }
    }
    //--------------------------------------------------One Electromagnets Movement-------------------------------------------------------
    //==========================================================MOVEMENT TYPE=======================================================

    //=======================================================INVERSE KINEMATICS=======================================================
    for (int i = 0; i < numPointsFinal; i++)
    {
#ifdef MODELFINAL

        numberofSteps[i][0] = round(-(driverMicroSteps * 5) * (finalTrajectory[i][0] - finalTrajectory[i][1]));
        numberofSteps[i][1] = round(-(driverMicroSteps * 5) * (-finalTrajectory[i][0] - finalTrajectory[i][1]));
#endif
#ifdef MODELEFRA
        numberofSteps[i][0] = round((driverMicroSteps * 5) * (finalTrajectory[i][0] - finalTrajectory[i][1]));
        numberofSteps[i][1] = round((driverMicroSteps * 5) * (-finalTrajectory[i][0] - finalTrajectory[i][1]));
#endif
    }
    //=======================================================INVERSE KINEMATICS=======================================================

    //=====================================================SPEED AND ACCELERATION=========================================================
    float lowerThreshold = 0.52;                    // Approximately 30 degrees in radians
    float upperThreshold = 2 * PI - lowerThreshold; // Approximately 330 degrees in radians
    std::vector<int> directionChangePoints;

    //--------------------------------------------------Calculates Speed-------------------------------------------------------
    for (int i = 0; i < numPointsFinal; i++) // Iterate over each trajectory point and assign a constant speed to each element
    {
        double deltaX = finalTrajectory[i][0] - ((i == 0) ? currentXposition : finalTrajectory[i - 1][0]);
        double deltaY = finalTrajectory[i][1] - ((i == 0) ? currentYposition : finalTrajectory[i - 1][1]);

        double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        timeToPosition[i] = distance / setSpeedbyUser; // The integer part of the division is the time to reach the position

        double currentPositionInStepsX = (i == 0) ? stepper1.currentPosition() : numberofSteps[i - 1][0];
        double currentPositionInStepsY = (i == 0) ? stepper2.currentPosition() : numberofSteps[i - 1][1];

        speedperStep[i][0] = (timeToPosition[i] == 0) ? 0 : ((numberofSteps[i][0] - currentPositionInStepsX) / timeToPosition[i]); // assign speed to the speed matrix
        speedperStep[i][1] = (timeToPosition[i] == 0) ? 0 : ((numberofSteps[i][1] - currentPositionInStepsY) / timeToPosition[i]); // assign speed to the speed matrix

        if ((i > 1 && i < numPointsFinal - 1)) // Direction change; protected against adding duplicate points
        {
            double currentDir = atan2(finalTrajectory[i][1] - finalTrajectory[i - 1][1], finalTrajectory[i][0] - finalTrajectory[i - 1][0]);
            double nextDir = atan2(finalTrajectory[i + 1][1] - finalTrajectory[i][1], finalTrajectory[i + 1][0] - finalTrajectory[i][0]);
            double dirChange = abs(nextDir - currentDir);

            if (dirChange >= lowerThreshold && dirChange <= upperThreshold)
            {
                directionChangePoints.push_back(i);
            }
        }
    }
    //--------------------------------------------------Calculates Speed-------------------------------------------------------
    directionChangePoints.push_back(numPointsFinal - 1); // Add the last trajectory point (index-based, not data count)

    //--------------------------------------------------Calculates Accelararion-------------------------------------------------------
    for (int i = 0; i < directionChangePoints.size(); i++) // Accelerations per segment according to direction changes; here we know the number of segments.
    {
        double segmentTime = 0;
        double timeX = 0;
        double y1, y2, y, a1, a2, c1, c2;

        int segmentStart = (i == 0) ? 1 : directionChangePoints[i - 1] + 1; // Segment start
        int segmentEnd = directionChangePoints[i];                           // Segment end

        for (int i = segmentStart; i <= segmentEnd; ++i) // Calculate the time for each segment
        {
            segmentTime = (i == segmentStart) ? 0 : segmentTime + timeToPosition[i];
        }

        if (segmentTime < 0.25)
        {
            a1 = 75;
            a2 = 75;
            c1 = 0.03;
        }
        else
        {
            a1 = 35;
            a2 = 35;
            c1 = 0.005;

            float targetY = 5000;
            float calculatedY;
            bool found = false;

            while (!found) // Search for a dynamic c1 value.
            {
                double deltaSpeedM1 = abs(speedperStep[segmentStart + 2][0] - speedperStep[segmentStart + 1][0]);
                double deltaSpeedM2 = abs(speedperStep[segmentStart + 2][1] - speedperStep[segmentStart + 1][1]);

                if (deltaSpeedM1 > deltaSpeedM2) // Use the motor with the larger speed change for the calculation
                {
                    calculatedY = speedperStep[segmentStart + 1][0] * (1 / (1 + exp(-a1 * (-c1))));
                }
                else
                {
                    calculatedY = speedperStep[segmentStart + 1][1] * (1 / (1 + exp(-a1 * (-c1))));
                }

                if (abs(calculatedY) < targetY) // if the calculated Y meets the requirement, set the flag to exit the loop
                {
                    found = true;
                }
                else // If the target is not yet reached, calculate another C1
                {
                    c1 += 0.005; // Increment C1 by 0.005 and repeat
                }
            }
        }
        c2 = segmentTime - c1; // Calculate c2 based on c1

        for (int i = segmentStart; i <= segmentEnd; i++) // Generate s-curve accelerations for each element of the segment
        {
            timeX = timeX + ((i == segmentStart) ? 0 : timeToPosition[i]);

            y1 = 1 / (1 + exp(-a1 * (timeX - c1)));
            y2 = 1 / (1 + exp(-a2 * (timeX - c2)));
            y = abs((y1)-y2);

            speedperStep[i][0] = speedperStep[i][0] * y;
            speedperStep[i][1] = speedperStep[i][1] * y;
        }
    }
    //--------------------------------------------------Calculates Accelararion-------------------------------------------------------
    //================================================SPEED AND ACCELERATION=========================================================
    stepper1.setMaxSpeed(10000000); // Prevent interference with speed calculations
    stepper2.setMaxSpeed(10000000); // Prevent interference with speed calculations

    // ================================================== MOVE MOTORS =============================================================
    int speedThreshold = 10000;
    bool runProtection = false;
    bool graveyardMove = false;
    unsigned long timetosafety = millis();
    unsigned long timetoTurnOff = 30000; // 30 seconds to turn off the system

    if (finalTrajectory[numPointsFinal][0] > 200 || finalTrajectory[numPointsFinal][0] < -200 || finalTrajectory[numPointsFinal][1] > 200 || finalTrajectory[numPointsFinal][1] < -200)
    {
        int jumpToCenter = BleChess.getJumpToCenter();

        if (jumpToCenter == 1)
        {
            Serial.println("=====================================================================================JALON ELECTROIMAN");
            graveyardMove = true;
        }
    }

    unsigned long timeMovingLocal = millis();
    for (int i = 0; i < numPointsFinal; i++)
    {

        int stepsM1 = numberofSteps[i][0] - stepper1.currentPosition();
        int stepsM2 = numberofSteps[i][1] - stepper2.currentPosition();

        float initialPosition1 = stepper1.currentPosition();
        float initialPosition2 = stepper2.currentPosition();

        float deltaX = abs(stepsM1);
        float deltaY = abs(stepsM2);

        double stepIsDue = deltaX > deltaY ? static_cast<double>(deltaY) / deltaX : static_cast<double>(deltaX) / deltaY;
        double stepSum = 0.0;
        int dirDeGiro1 = stepsM1 > 0 ? 1 : -1;
        int dirDeGiro2 = stepsM2 > 0 ? 1 : -1;

        //--------------------------------------------Protects the motors from a sudden change in speed--------------------------------------
        if (runProtection == true)
        {
            stepper1.setMaxSpeed(10000000);
            stepper2.setMaxSpeed(10000000);
        }
        runProtection = false;
        if (i > 1) // Activate .run function if there is a very large speed change
        {
            if (speedperStep[i][1] - speedperStep[i - 1][1] > speedThreshold || speedperStep[i][0] - speedperStep[i - 1][0] > speedThreshold)
            {
                runProtection = true;
            }
        }
        //--------------------------------------------Protects the motors from a sudden change in speed--------------------------------------

        //--------------------------------------------Electromagnets activation and deactivation--------------------------------------------------
        if (i == 1) // activates electromagnet in the first cycle
        {
            activateElectromagnetV2((activeElectroCount == 0) ? activeElectroInit : activeElectro, 35);
            stepper1.setMaxSpeed(10000000);
            stepper2.setMaxSpeed(10000000);
        }

        if (graveyardMove == true && i == numPointsFinal - 13) // Dactivates electromagnet in the last 13 cycles so it makes a pull on the electromagnet
        {
            deactivateAllMagnets();
        }

        if (activeElectroCount == 0) // Mechanism is making a complex movement with a different electromagnet for each segment
        {
            if (i == (numPointsFinal - electromagnetCount[activeElectroEnd])) // Deactivates activeElectroInit
            {
                deactivateAllMagnets();
            }
            else if (i == (numPointsFinal - electromagnetCount[activeElectroEnd]) + 1) // Activates activeElectroEnd in the following cycle.
            {
                activateElectromagnetV2(activeElectroEnd, 35);
            }
        }

        if (millis() - timetosafety > timetoTurnOff) // if time is greater than 60 seconds, the electromagnet and motors are disabled
        {
            deactivateAllMagnets();
            stepper1.disableOutputs();
            stepper2.disableOutputs();
            while (true)
            {
                if (millis() - timetosafety > timetoTurnOff) // prints a message every timetoTurnOff seconds
                {
                    Serial.println("Safety Time, Electromagnet and motors disabled");
                    timetosafety = millis();
                }
            }
        }
        else
        {
            timetosafety = millis();
        }
        //--------------------------------------------Electromagnets activation and deactivation--------------------------------------------------

        // --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
        if (deltaX > deltaY)
        {
            if (i == 0 || runProtection == true)
            {
                stepper1.setAcceleration(speedperStep[i][0] * 2);
                stepper1.setMaxSpeed(speedperStep[i][0] * 1);
                stepper1.moveTo(numberofSteps[i][0]);
            }
            else
            {
                stepper1.setSpeed(speedperStep[i][0]);
                stepper2.setSpeed(speedperStep[i][1]);
            }
            while ((abs(stepper1.currentPosition() - numberofSteps[i][0]) > 0) && (millis() - timetosafety < timetoTurnOff))
            {
                if (i == 0 || runProtection == true)
                {
                    stepper1.run();
                }
                else
                {
                    stepper1.runSpeed();
                }

                if (abs(stepper1.currentPosition() - initialPosition1) >= 1)
                {
                    stepSum += stepIsDue;
                    initialPosition1 = stepper1.currentPosition();
                    while (stepSum >= 1 && (millis() - timetosafety < timetoTurnOff))
                    {
                        stepper2.setSpeed(1000000.0 * dirDeGiro2);
                        stepper2.runSpeed();
                        stepSum -= 1;
                    }
                }
            }
        }
        else
        {
            if (i == 0 || runProtection == true)
            {
                stepper2.setAcceleration(speedperStep[i][1] * 2);
                stepper2.setMaxSpeed(speedperStep[i][1] * 1);
                stepper2.moveTo(numberofSteps[i][1]);
            }
            else
            {
                stepper1.setSpeed(speedperStep[i][0]);
                stepper2.setSpeed(speedperStep[i][1]);
            }
            while ((abs(stepper2.currentPosition() - numberofSteps[i][1]) > 0) && (millis() - timetosafety < timetoTurnOff))
            {
                if (i == 0 || runProtection == true)
                {
                    stepper2.run();
                }
                else
                {
                    stepper2.runSpeed();
                }

                if (abs(stepper2.currentPosition() - initialPosition2) >= 1)
                {
                    stepSum += stepIsDue;
                    initialPosition2 = stepper2.currentPosition();
                    while (stepSum >= 1 && (millis() - timetosafety < timetoTurnOff))
                    {
                        stepper1.setSpeed(1000000.0 * dirDeGiro1);
                        stepper1.runSpeed();
                        stepSum -= 1;
                    }
                }
            }
        }
        // --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    }
    timeMovingLocal = millis() - timeMovingLocal;
    timeMoving = timeMoving + timeMovingLocal;

    if (graveyardMove)
    {
        activateElectromagnetV2(activeElectroCount == 0 ? activeElectroEnd : activeElectro, 35);
        delay(200);
    }

    if (testModeSculpture == false)
    {
        deactivateAllMagnets();
    }

    // ================================================== MOVE MOTORS =============================================================

    // ================================================= FREE MEMORY ==========================================================
    if (timeToPosition != nullptr)
    {
        delete[] timeToPosition;  // Free the entire array
        timeToPosition = nullptr; // Ensure the pointer does not point to a freed location
    }

    if (speedperStep != nullptr)
    {
        for (int i = 0; i < numPointsFinal; i++)
        {
            delete[] speedperStep[i];
        }
        delete[] speedperStep;
        speedperStep = nullptr;
    }

    if (numberofSteps != nullptr)
    {
        for (int i = 0; i < numPointsFinal; i++)
        {
            delete[] numberofSteps[i];
        }
        delete[] numberofSteps;
        numberofSteps = nullptr;
    }
    // At this point, the size and capacity are probably equal.
    directionChangePoints.clear();
    // Now the size is 0, but the capacity remains 1000.

    // If you really need to reduce the memory used by a vector after clearing or substantially modifying it:
    directionChangePoints.shrink_to_fit();
    // ================================================= FREE MEMORY ==========================================================
    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
}
