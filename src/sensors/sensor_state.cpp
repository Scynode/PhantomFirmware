/*
 * sensor_state.cpp
 * Definitions of all sensor-related global variables.
 *
 * These globals were previously defined at the top of scultpureMode.cpp.
 * They are now centralised here so that any translation unit that includes
 * sensor_state.h has access to them via the extern declarations.
 *
 * muxesOut[]  - maps mux-output index (1-4) to ESP32 GPIO pin numbers.
 *               Index 0 is unused (chess squares use indices 1-4).
 * dirMux[][]  - 10x10 lookup table, populated by sensorsDir().
 *               Each entry is a 3-digit code:
 *                 hundreds digit -> which of the 4 mux output lines (muxesOut index)
 *                 tens digit     -> 3-bit selector value for the 16-channel mux (mux16 pins)
 *                 units digit    -> 3-bit selector value for the 8-channel mux (mux8 pins)
 */

#include "sensor_state.h"
#include "../../include/config.h"

// Sensor matrix dimensions (10 rows x 10 cols = chess 8x8 + 2-cell graveyard border)
const int rows = 10;
const int cols = 10;

// Output-pin mapping for the 4 mux data lines.
// muxesOut[0] is intentionally unused so that mux indices match the 1-based hardware numbering.
int muxesOut[5] = {0, mux16Out_1, mux16Out_2, mux16Out_3, mux16Out_4};

// Mux address table; values are filled in by sensorsDir() at startup.
int dirMux[10][10];

// Sensor state used internally by the sculpture-mode logic (e.g. movementType, calibration).
// false = piece detected on that square, true = square is empty.
bool sensorMatrixSc[10][10];

// Rolling sensor snapshot written by updateSensors() after applying board-rotation.
// All consumer code reads from this buffer.
bool sensorUpdate[10][10];
