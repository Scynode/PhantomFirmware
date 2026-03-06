#pragma once

/*
 * sensor_state.h
 * Extern declarations for all sensor-related global variables.
 *
 * SENSOR MATRIX LAYOUT (10x10):
 * The chess board is an 8x8 grid of squares, but the sensor matrix is 10x10
 * to accommodate "graveyard" cells:
 *
 *   col: 0    1    2    3    4    5    6    7    8    9
 * row 0: [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY]  <- graveyard row (captured pieces)
 * row 1: [GY] [a8] [b8] [c8] [d8] [e8] [f8] [g8] [h8] [GY]
 * row 2: [GY] [a7] [b7] [c7] [d7] [e7] [f7] [g7] [h7] [GY]
 * ...
 * row 8: [GY] [a1] [b1] [c1] [d1] [e1] [f1] [g1] [h1] [GY]
 * row 9: [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY]  <- graveyard row (captured pieces)
 *
 * sensorMatrix[i][j]: false = piece present (sensor triggered), true = no piece
 *
 * Rows/cols 0 and 9 form a 2-cell-wide border used as a graveyard for
 * captured pieces waiting to be removed from the board.
 */

// Number of rows and columns in the sensor matrix (10x10 for chess 8x8 + 2 border rows/cols)
extern const int rows;
extern const int cols;

// Lookup table: dirMux[row][col] encodes the mux address for each board cell.
// The value is a 3-digit decimal: hundreds=mux output index, tens=mux16 channel, units=mux8 channel.
extern int dirMux[10][10];

// Output pin indices for the 16-channel mux data lines (index 1-4 used; 0 unused).
extern int muxesOut[5];

// Local sensor state buffer maintained by scultpureMode.cpp logic.
// false = piece present, true = empty square.
extern bool sensorMatrixSc[10][10];

// Double-buffer: updateSensors() writes hardware readings here, rotated for board orientation.
// Other code reads from this buffer to get the latest sensor snapshot.
extern bool sensorUpdate[10][10];
