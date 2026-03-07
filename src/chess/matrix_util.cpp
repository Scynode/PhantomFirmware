/*
 * matrix_util.cpp
 * Utility functions for the 10×10 chess board matrix.
 *
 * THE 10×10 MATRIX FORMAT:
 * ─────────────────────────────────────────────────────────────────────────────
 * The board is represented as a char[10][10] where indices [row][col]:
 *
 *   Rows/cols 1-8 map to the 8×8 chess squares (row 1 = rank 8, row 8 = rank 1;
 *   col 1 = file a, col 8 = file h).
 *
 *   Rows/cols 0 and 9 form a one-cell-wide graveyard border used to park
 *   captured pieces off the active board without needing a separate structure:
 *
 *     Row 0 (cols 1-8): captured black pawns
 *     Row 9 (cols 1-8): captured white pawns
 *     Col 0 (rows 1-8): captured white non-pawn pieces
 *     Col 9 (rows 1-8): captured black non-pawn pieces
 *     Corner cells (0,0), (0,9), (9,0), (9,9): never used ('.' always)
 *
 * Piece encoding (single ASCII character):
 *   'R'/'r' rook, 'N'/'n' knight, 'B'/'b' bishop,
 *   'Q'/'q' queen, 'K'/'k' king, 'P'/'p' pawn
 *   Upper-case = white, lower-case = black, '.' = empty
 *   'L' = locked (already in correct position, skip during reorder)
 */

#include "scultpureMode.h"
#include <Arduino.h>
#include "config.h"

/*
 * printGenericMatrix()
 * ─────────────────────────────────────────────────────────────────────────────
 * Prints a char matrix to Serial with row and column index headers.
 *
 * The output is transposed internally so that column indices run along the top
 * and row indices down the left side, matching the visual orientation of the
 * physical board when viewed from behind the white pieces.
 *
 * Parameters:
 *   matrix   – the matrix to print (declared as [][10] to match the extern in
 *              scultpureMode.h, which uses the global 'cols' constant)
 *   numRows  – number of rows (typically 10)
 *   numCols  – number of columns (typically 10)
 *
 * Side effects: writes to Serial.
 */
void printGenericMatrix(char matrix[][10], int numRows, int numCols)
{
    char vectNumeros[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    char matrixAux[numRows + 1][numCols + 1];

    // Copy the source matrix into matrixAux shifted by one cell in each
    // dimension, leaving row 0 and col 0 free for index labels.
    for (int j = 1; j < numRows + 1; j++)
    {
        for (int i = 1; i < numCols + 1; i++)
        {
            matrixAux[i][j] = matrix[i - 1][j - 1];
        }
    }

    // Add column index numbers to the top row.
    for (int i = 1; i < numCols + 1; i++)
    {
        matrixAux[i][0] = vectNumeros[i - 1];
    }

    // Add row index numbers to the left column.
    for (int j = 1; j < numRows + 1; j++)
    {
        matrixAux[0][j] = vectNumeros[j - 1];
    }

    // Top-left corner is blank.
    matrixAux[0][0] = ' ';

    // Print column-major (j = row, i = col) so the board looks correct.
    for (int j = 0; j < numRows + 1; j++)
    {
        for (int i = 0; i < numCols + 1; i++)
        {
            Serial.print(matrixAux[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

/*
 * initMatrixPlus()
 * ─────────────────────────────────────────────────────────────────────────────
 * Initialises the 10×10 chess matrix to the standard chess starting position.
 *
 * All 100 cells are first set to '.' (empty), then the 32 pieces are placed
 * in their opening positions:
 *   Row 1, cols 1-8: black back rank  (r n b q k b n r)
 *   Row 2, cols 1-8: black pawns      (p p p p p p p p)
 *   Row 7, cols 1-8: white pawns      (P P P P P P P P)
 *   Row 8, cols 1-8: white back rank  (R N B Q K B N R)
 *
 * The graveyard border cells (row 0, row 9, col 0, col 9) remain '.' because
 * no pieces have been captured yet.
 *
 * Parameters:
 *   matrixToInit – the 10×10 matrix to initialise (modified in-place)
 */
void initMatrixPlus(char matrixToInit[10][10])
{
    // Clear the entire matrix.
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrixToInit[i][j] = '.';
        }
    }

    // Standard piece ordering along the back rank (files a-h).
    char vectChessW[8] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[8] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};

    for (int i = 1; i < 9; i++)
    {
        matrixToInit[i][1] = vectChessB[i - 1]; // black back rank (rank 8)
        matrixToInit[i][2] = 'p';               // black pawns (rank 7)
        matrixToInit[i][8] = vectChessW[i - 1]; // white back rank (rank 1)
        matrixToInit[i][7] = 'P';               // white pawns (rank 2)
    }
}
