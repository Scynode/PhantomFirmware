#pragma once

/*
 * mech_state.h
 * Extern declarations for all mechanical subsystem globals.
 *
 * DRIVE SYSTEM OVERVIEW — CoreXY-style kinematics:
 * ─────────────────────────────────────────────────
 * The Phantom board uses two stepper motors (M1, M2) connected via a CoreXY
 * belt arrangement.  In CoreXY, both motors contribute to BOTH axes:
 *
 *   Physical X movement: both motors turn in the same direction
 *   Physical Y movement: motors turn in opposite directions
 *
 * Inverse kinematics (physical mm → motor steps):
 *   steps_M1 = -(driverMicroSteps * 5) * (X - Y)
 *   steps_M2 = -(driverMicroSteps * 5) * (-X - Y)
 *
 * Forward kinematics (motor steps → physical mm):
 *   Y = (steps_M1 + steps_M2) / (-2 * -(driverMicroSteps * 5))
 *   X = steps_M1 / -(driverMicroSteps * 5) + Y
 *
 * The factor 5 comes from the lead-screw / pulley pitch (5mm per full step).
 * driverMicroSteps is the current microstepping divisor (8, 16, 32, or 128).
 *
 * STEPPER DRIVER — TMC2209:
 *   Two TMC2209 drivers share a single UART bus (SERIAL_PORT2 / Serial1).
 *   They are distinguished by their UART address:
 *     driver  (M1) → DRIVER_ADDRESS1 = 0b00
 *     driver2 (M2) → DRIVER_ADDRESS2 = 0b11
 *   StallGuard (SG) is used for sensorless homing; the SG_RESULT register is
 *   monitored while the motor drives into a mechanical end-stop.
 */

#include <TMCStepper.h>
#include <AccelStepper.h>

// ── Stepper driver objects (TMC2209 over UART) ───────────────────────────────
extern TMC2209Stepper driver;   // Motor 1 (DRIVER_ADDRESS1 = 0b00)
extern TMC2209Stepper driver2;  // Motor 2 (DRIVER_ADDRESS2 = 0b11)

// ── AccelStepper motion control objects ──────────────────────────────────────
extern AccelStepper stepper1;   // Motor 1 step/dir interface
extern AccelStepper stepper2;   // Motor 2 step/dir interface

// ── Speed tracking ───────────────────────────────────────────────────────────
// Stores the last speed value passed to configAndCurrentPosManager() so that
// the driver configuration is only updated when the speed actually changes.
extern double lastspeedbyUser;

// ── Microstepping divisor ────────────────────────────────────────────────────
// Current TMC2209 microstepping setting (8, 16, 32, or 128).
// Initialised to -1 to force a configuration on the first call.
extern float driverMicroSteps;

// ── Hardware timers (ESP32 hardware timer handles) ───────────────────────────
// timer1H: used during rawMovementStallGuard to drive the ISR (moveRawStallGuard)
//          that calls stepper1/2.runSpeedToPosition() at a fixed interrupt rate.
// timer2H: used to automatically cut power to the electromagnet after 60 seconds
//          of continuous activation (safety / thermal protection).
extern hw_timer_t *timer1H;
extern hw_timer_t *timer2H;

// ── Electromagnet safety timer ───────────────────────────────────────────────
// Timestamp (millis()) set each time an electromagnet is activated.
// The timeEnabled() ISR compares the elapsed time against 60 000ms and cuts
// power if the limit is exceeded.
extern volatile unsigned long timerMagnets;

// ── Trajectory parameters ────────────────────────────────────────────────────
// Number of sample points per Bézier curve segment.
extern int totalPointsInCurve;

// Physical spacing between interpolated trajectory waypoints in mm.
// Smaller values give smoother motion but increase computation time.
extern float distanceBetweenPoints;
