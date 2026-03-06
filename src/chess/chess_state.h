#pragma once

/*
 * chess_state.h
 * Extern declarations for chess-specific global state used across the chess
 * and sculpture-mode subsystems.
 *
 * ─── 10×10 CHESS MATRIX LAYOUT ───────────────────────────────────────────────
 * The physical chess board is 8×8, but the software matrix is 10×10 to provide
 * a one-cell-wide "graveyard" border on all four sides for captured pieces:
 *
 *   col:  0    1    2    3    4    5    6    7    8    9
 * row 0: [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY]  ← top graveyard
 * row 1: [GY] [a8] [b8] [c8] [d8] [e8] [f8] [g8] [h8] [GY]
 * row 2: [GY] [a7] [b7] [c7] [d7] [e7] [f7] [g7] [h7] [GY]
 *  ...
 * row 8: [GY] [a1] [b1] [c1] [d1] [e1] [f1] [g1] [h1] [GY]
 * row 9: [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY] [GY]  ← bottom graveyard
 *
 * Left  graveyard (col 0, rows 1-8): captured white non-pawn pieces.
 * Right graveyard (col 9, rows 1-8): captured black non-pawn pieces.
 * Top   graveyard (row 0, cols 1-8): captured black pawns.
 * Bottom graveyard (row 9, cols 1-8): captured white pawns.
 * Corner cells (0,0), (0,9), (9,0), (9,9) are never used.
 *
 * Piece encoding (ASCII characters):
 *   'R' white rook,  'N' white knight, 'B' white bishop,
 *   'Q' white queen, 'K' white king,   'P' white pawn
 *   'r' black rook,  'n' black knight, 'b' black bishop,
 *   'q' black queen, 'k' black king,   'p' black pawn
 *   '.' empty square
 *   'L' locked/already-placed (used temporarily by reorderChessboardPlus)
 *
 * ─── movChess ENCODING FORMAT ────────────────────────────────────────────────
 * movChess[7][250] stores the full decoded move sequence for the current game.
 * Each individual move is stored as a 7-character array (movChessInd[7]) with
 * the raw PGN token characters, padded with 'v' (void/unused).
 *
 * Examples of raw PGN tokens parsed into movChessInd:
 *   "e2-e4"  → ['e','2','-','e','4','v','v']  simple pawn move
 *   "Nb1-c3" → ['N','b','1','-','c','3','v']  knight move with piece prefix
 *   "O-O"    → ['O','-','O','v','v','v','v']  short castling
 *   "O-O-O"  → ['O','-','O','-','O','v','v']  long castling
 *   "e5xd6"  → ['e','5','x','d','6','v','v']  pawn capture
 *
 * decodeChessMove() converts each movChessInd to the format consumed by
 * decodeMovement():  [color][action][x_ini][y_ini][x_fin][y_fin]
 *   color : 'W' or 'B'
 *   action: '0'=simple move, '1'=capture, '2'=short castle, '3'=long castle
 *   x_ini, y_ini, x_fin, y_fin: board coordinate characters ('a'-'h', '1'-'8')
 */

#include <Arduino.h>

// ── Timing ────────────────────────────────────────────────────────────────────
// Tracks when the current sculpture/test session started (millis()).
// Used by simplifiedMovement to enforce the 10-minute test-mode timeout.
extern unsigned long timeSculpture;

// ── Per-move PGN token buffer ─────────────────────────────────────────────────
// Stores up to 250 raw PGN move tokens for a single game, each in a 7-char
// slot.  Populated by readFromFileSc() and consumed by decodeChessMove().
extern char movChess[7][250];

// ── Global reorder target matrix ─────────────────────────────────────────────
// Temporary target configuration used by reorderChessboardPlus().
// Modified in-place during the reorder algorithm; cleared when finished.
extern char targetMatrix[10][10];

// ── Reorder-in-progress flag ──────────────────────────────────────────────────
// Set to true while reorderChessboardPlus() is running so that
// simplifiedMovement() knows to skip the post-move sensor verification loop.
extern bool reorderChessboard;

// ── Embedded game database ────────────────────────────────────────────────────
// Array of PGN-format chess game strings stored in flash (program memory).
// Indices 0-199 are the built-in games.  Indices 200-299 refer to user-uploaded
// games stored in Preferences flash storage (NVS), accessed by readFromFileSc().
extern const char *games[];
