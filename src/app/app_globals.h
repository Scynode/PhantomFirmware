/**
 * @file app_globals.h
 * @brief Extern declarations for all global variables and free functions
 *        defined in the src/app/ layer (setup.cpp, modes.cpp, play_mode.cpp).
 *
 * Any translation unit that needs to access these symbols should include this
 * header.  The actual definitions live in setup.cpp (for data) and in the
 * individual mode/play files (for functions).
 *
 * ─── Global variables owned by setup.cpp ────────────────────────────────────
 *
 *   sensorMatrix      - live 10×10 boolean sensor state updated by bleTask.
 *                       false = piece present (sensor triggered),
 *                       true  = square empty.
 *
 *   previousChessState - snapshot of sensorMatrix from the last accepted move;
 *                        used by detectChangePlus() to identify delta events.
 *
 *   iniChangeX/Y       - column/row of the last piece-lift detected.
 *   finChangeX/Y       - column/row of the last piece-placement detected.
 *
 *   totalDeadPieces    - running count of captured pieces (unused at runtime,
 *                        kept for debugging convenience).
 *   pieceSensorCount   - scratch counter used during sensor self-tests.
 *   deadPieceSensorCount - scratch counter used during sensor self-tests.
 *   prevDeadPiecesCount  - used in automaticMechanicMovement() to track
 *                          whether a capture occurred in the current move.
 *
 *   mode     - active firmware mode (mirrors BleChess.getModeChess()).
 *   testFlag - NVS-persisted boot counter used to trigger test / factory-reset
 *              modes on repeated cold boots.
 */

#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Sensor state
// ---------------------------------------------------------------------------
/** 10×10 boolean sensor matrix.  Updated continuously by bleTask() on core 0.
 *  false = hall-effect sensor triggered (piece present).
 *  true  = no piece.
 *  The 8×8 inner region is the chess board; the outer ring is the graveyard. */
extern bool sensorMatrix[10][10];

/** Snapshot of sensorMatrix taken after each accepted move.
 *  detectChangePlus() compares incoming readings to this to detect piece lifts
 *  and placements. */
extern bool previousChessState[10][10];

// ---------------------------------------------------------------------------
// Move tracking
// ---------------------------------------------------------------------------
/** X (column) coordinate of the most-recently-lifted piece (1-indexed). */
extern int iniChangeX;
/** Y (row) coordinate of the most-recently-lifted piece (1-indexed). */
extern int iniChangeY;
/** X (column) coordinate of the most-recently-placed piece (1-indexed). */
extern int finChangeX;
/** Y (row) coordinate of the most-recently-placed piece (1-indexed). */
extern int finChangeY;

// ---------------------------------------------------------------------------
// Piece counters (debug / housekeeping)
// ---------------------------------------------------------------------------
extern int totalDeadPieces;      ///< Total captures since last reset
extern int pieceSensorCount;     ///< Scratch counter for test mode
extern int deadPieceSensorCount; ///< Scratch counter for test mode
extern int prevDeadPiecesCount;  ///< Captures before the current BLE move

// ---------------------------------------------------------------------------
// Mode state
// ---------------------------------------------------------------------------
/** Current operating mode, kept in sync with BleChess.getModeChess(). */
extern int mode;

/** NVS-persisted boot counter.
 *  2–7  → trigger hardware self-test (testMode).
 *  8–14 → trigger factory reset.
 *  Set to 0 after a clean boot; set to -1 if test mode was entered from the
 *  board (physical button / rapid power cycles). */
extern int testFlag;

// ---------------------------------------------------------------------------
// Free-function forward declarations
// ---------------------------------------------------------------------------

/** Euclidean distance between two 2-D points (in mm). */
float euclideanDistance(float x1, float y1, float x2, float y2);

/** Find the centre of the nearest 50 mm grid square to (x, y).
 *  @param[in]  x, y      Query point in mm (board coordinates, centre = 0,0).
 *  @param[out] centerX, centerY  Centre of the nearest square (mm).
 *  @param[out] kprima, lprima    1-based column and row indices of that square. */
void nearestCenter(float x, float y,
                   float &centerX, float &centerY,
                   float &kprima, float &lprima);

/** Halt the firmware, report @p error repeatedly via BLE and Serial, and beep
 *  continuously.  Never returns. */
void errorMessage(String error);

/** Convert a 100-character flat string (row-major) to a 10×10 char matrix. */
void stringToMatrix(String str, char matrix[10][10]);

/** Poll the BLE sound-check characteristic and play the appropriate end-of-game
 *  tone if the app signalled check (1) or checkmate (2). */
void soundEndGame();

/** Detect a physical piece movement on the board and return the encoded move
 *  string (e.g. "M 1 e2-e4") or "" if no valid complete move was detected.
 *  @param[in,out] currentMatrix  The current board state; updated in-place to
 *                                reflect the detected move.
 *  @param[out]    specialMove    Set to 10–13 if a castling king-lift was
 *                                detected (used to suppress premature validation). */
String detectChangePlus(char currentMatrix[10][10], int &specialMove);

/** Execute a BLE-commanded chess move mechanically.
 *  @param movementString  Move in the format "X Wx e2-e4" (simple) or
 *                         "X Wx e2xe4" (capture).
 *  @param matrixToAutomaticMove  Board state matrix; updated after the move. */
void automaticMechanicMovement(String movementString,
                               char matrixToAutomaticMove[10][10]);
