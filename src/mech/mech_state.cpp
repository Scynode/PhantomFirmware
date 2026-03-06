/*
 * mech_state.cpp
 * Definitions of all mechanical subsystem global variables.
 *
 * These globals were previously defined at the top of scultpureMode.cpp.
 * Centralising them here prevents duplicate definitions when the mechanical
 * subsystem is split across multiple translation units.
 *
 * Include mech_state.h to access these from other .cpp files.
 */

#include "mech_state.h"
#include "../../include/config.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Arduino.h>

// ── TMC2209 UART stepper driver instances ────────────────────────────────────
// Both drivers share the same hardware serial bus (SERIAL_PORT2 = Serial1).
// They are identified by their UART addresses set via MS1/MS2 hardware pins:
//   driver  (M1): address 0b00 (MS1=LOW,  MS2=LOW)
//   driver2 (M2): address 0b11 (MS1=HIGH, MS2=HIGH)
// R_SENSE is the current-sense resistor value (0.11 Ω) used to calibrate RMS current.
TMC2209Stepper driver(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);

// ── AccelStepper motion control instances ────────────────────────────────────
// Configured in DRIVER mode (external step/dir signals).
// Motor 0 (stepper1) drives the first belt axis; Motor 1 (stepper2) the second.
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_0_STEP_PIN, MOTOR_0_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN);

// ── Speed tracking ────────────────────────────────────────────────────────────
// Remembers the last speed value used by configAndCurrentPosManager() so that
// the TMC2209 configuration is skipped when the speed has not changed.
double lastspeedbyUser = 0;

// ── Active microstepping divisor ─────────────────────────────────────────────
// Valid values: 8, 16, 32, 128.
// -1 forces a full reconfiguration on the very first call.
float driverMicroSteps = -1;

// ── Hardware timer handles (ESP32 IDF timer API) ─────────────────────────────
// timer1H is allocated inside rawMovementStallGuard() and freed when done.
// timer2H is allocated inside activateElectromagnetV2() and freed when all magnets off.
hw_timer_t *timer1H = NULL;
hw_timer_t *timer2H = NULL;

// ── Electromagnet safety cutoff timestamp ────────────────────────────────────
// Updated by activateElectromagnetV2(); checked inside the timeEnabled() ISR.
volatile unsigned long timerMagnets = 0;

// ── Trajectory resolution parameters ─────────────────────────────────────────
// Number of points computed for each Bézier curve segment (higher = smoother arcs).
int totalPointsInCurve = 90;

// Distance between consecutive waypoints along a straight interpolated segment (mm).
// 0.5mm gives fine resolution for smooth motion at typical chess-move speeds.
float distanceBetweenPoints = 0.5;
