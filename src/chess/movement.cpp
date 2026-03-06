/*
 * movement.cpp
 * Physical chess piece movement functions.
 *
 * This file implements the core routines that translate logical chess moves
 * into physical magnet-driven piece movements on the Phantom board:
 *
 *   simplifiedMovement()       – moves a single piece from one square to another
 *   decodeMovement()           – converts a decoded PGN move into a sequence of
 *                                 physical movement commands encoded as a String
 *   reorderChessboardPlus()    – rearranges pieces on the board to match a target
 *                                 configuration using a nearest-neighbour approach
 *   centerInitialPiecesSc()    – nudges every piece to the centre of its square
 *   compareMatrixVsSensorsPlus()– reconciles the virtual board matrix with actual
 *                                 sensor readings and corrects discrepancies
 *   findNearestEmptyPosition() – BFS-style search for the closest empty square
 *   findNearestPiecePosition() – BFS-style search for the nearest matching piece
 *
 * ─── MOVE ENCODING FORMAT ────────────────────────────────────────────────────
 * decodeMovement() returns a String whose characters encode one or two physical
 * movements:
 *
 *   Simple move  :  "[rowInit][colInit][rowEnd][colEnd]/"
 *   Capture      :  "[captRow][captCol][gravRow][gravCol][rowInit][colInit][rowEnd][colEnd]/"
 *   Short castle :  "58788868/"  (white) or "51718161/"  (black)
 *   Long  castle :  "58381848/"  (white) or "51311141/"  (black)
 *
 * All row/col values are single ASCII digits ('0'-'9'), matching the 10×10
 * matrix indices.  The '/' delimiter separates individual half-moves.
 *
 * ─── reorderChessboardPlus() ALGORITHM ───────────────────────────────────────
 * The function iterates over every cell of the target matrix in column-major
 * order.  For each target cell that still needs a piece:
 *
 *   1. Locate the nearest matching piece in the current matrix using a
 *      BFS-like expanding-square search (findNearestPiecePosition).
 *   2. If the target cell is already occupied by a DIFFERENT piece:
 *      a. If that blocking piece is needed elsewhere and its destination is
 *         empty → push it directly to its own target (saves a step).
 *      b. Otherwise → push it to the nearest empty square first, then move
 *         the desired piece in.
 *   3. If the target cell is empty → move the desired piece directly.
 *   4. Each time the function processes a cell it looks at sensorMatrixSc to
 *      account for pieces placed by the user during the reorder sequence.
 *
 * ─── compareMatrixVsSensorsPlus() LOGIC ──────────────────────────────────────
 * Positive mode (>= 0):
 *   Loops until the sensor matrix matches the virtual matrix.  On the first
 *   mismatch it calls centerInitialPiecesSc (COMPARE_CENTRA bit); on the
 *   second mismatch it recalibrates (COMPARE_HOME bit); on the third or later
 *   it sends an error to the BLE app and waits for manual correction.
 *
 * Negative mode (< 0):
 *   Used in interactive play mode.  Loops until the user has correctly placed
 *   all pieces, playing a sound every 5 seconds while pieces remain misplaced,
 *   and a success sound once everything matches.
 */

#include "scultpureMode.h"
#include <Arduino.h>
#include "config.h"
#include "chess_state.h"
#include "../sensors/sensor_state.h"
#include "../mech/mech_state.h"
#include "BLE.h"

/*
 * simplifiedMovement()
 * ─────────────────────────────────────────────────────────────────────────────
 * Moves a chess piece physically from one board square to another.
 *
 * Generates a trajectory, runs the stepper motors, then verifies that the
 * sensor detects the piece has moved.  Retries up to targetAttempts times.
 * If still unsuccessful in normal mode, waits for the user to manually move
 * the piece; in test mode, halts with a buzzer warning.
 *
 * After a successful move the virtual matrix (matrixToMove) is updated and
 * compareMatrixVsSensorsPlus() is called to verify board consistency.
 *
 * Parameters:
 *   squareRowInit  – source row index (0-9 in the 10×10 matrix)
 *   squareColInit  – source column index
 *   squareRowEnd   – destination row index
 *   squareColEnd   – destination column index
 *   matrixToMove   – virtual board matrix updated after the move (in-place)
 *
 * Side effects: drives stepper motors, updates previousChessState and
 *               matrixToMove, may call compareMatrixVsSensorsPlus().
 */
void simplifiedMovement(int squareRowInit, int squareColInit, int squareRowEnd, int squareColEnd, char matrixToMove[10][10])
{
    int initialMode = BleChess.getModeChess();
    if (squareRowInit == squareRowEnd && squareColInit == squareColEnd)
    {
        String error = "ERROR SC-MOV-1. Movement From: " + String(squareRowInit) + String(squareColInit) + (" To: ") + String(squareRowEnd) + String(squareColEnd) + (" Modo: ") + String(initialMode) + ". Send error code to Phantom Team please.";
        Serial.println("ERROR: Same position. No movement." + error);
        BleChess.sendTestModeError(error);
        return;
    }
    else if (squareRowInit < 0 || squareRowInit > 9 || squareColInit < 0 || squareColInit > 9 || squareRowEnd < 0 || squareRowEnd > 9 || squareColEnd < 0 || squareColEnd > 9)
    {
        String error = "ERROR SC-MOV-2. Movement From: " + String(squareRowInit) + String(squareColInit) + (" To: ") + String(squareRowEnd) + String(squareColEnd) + (" Modo: ") + String(initialMode) + ". Send error code to Phantom Team please.";
        Serial.println("ERROR: Out of bounds. No movement." + error);
        BleChess.sendTestModeError(error);
        return;
    }

    int attempts = 0;
    int targetAttempts = 0;
    int numPointsFinal = 0;
    bool pieceArrived = false;

    // Sculpture mode gets more retry attempts because the board is running
    // autonomously; play mode (mode 2) gets fewer since a human is present.
    targetAttempts = (initialMode == 1) ? 5 : 2;
    if (testModeSculpture == true)
    {
        targetAttempts = 1;
        Serial.println("Test Mode");
    }
    else
    {
        Serial.println((initialMode == 1) ? "Sculpture Mode" : "Play Mode");
    }

    unsigned long timerlocal = millis();
    // Attempt to move the piece up to targetAttempts times.
    while (!pieceArrived && attempts < targetAttempts && BleChess.getModeChess() == initialMode)
    {
        if (attempts > 0)
        {
            Serial.print("ERROR: Piece not detected at final Position. Try number: ");
            Serial.println(attempts);
        }

        // Generate and execute the physical trajectory.
        float **finalTrajectory = generateTrajectory(squareRowInit, squareColInit, squareRowEnd, squareColEnd, numPointsFinal);
        accelRampV3(finalTrajectory, numPointsFinal, BleChess.getMechanismSpeed());
        if (finalTrajectory != nullptr)
        {
            for (int i = 0; i < numPointsFinal; i++)
            {
                delete[] finalTrajectory[i];
            }
            delete[] finalTrajectory;
            finalTrajectory = nullptr;
        }

        // Verify the move via sensors: source should now be empty, destination occupied.
        detectChessBoard(sensorMatrixSc);
        pieceArrived = (sensorMatrixSc[squareRowInit][squareColInit] && !sensorMatrixSc[squareRowEnd][squareColEnd]) ? true : false;
        attempts++;
    }

    // Determine whether to wait for a human to fix the board after exhausting retries.
    bool waitforPiece = (initialMode == 1 || (initialMode == 2 && reorderChessboard == false)) ? true : false;
    if (attempts > targetAttempts && BleChess.getModeChess() == initialMode)
    {
        if (testModeSculpture == false) // Normal mode: wait for human intervention.
        {
            while (!pieceArrived && waitforPiece)
            {
                if (millis() - timerlocal > 10000)
                {
                    Serial.println("ERROR: Too many attempts. Mechanism Disengage move the piece manually.");
                    String message = "ERROR1: (" + String(squareRowInit) + String(squareColInit) + String(squareRowEnd) + String(squareColEnd) + ")";
                    BleChess.sendMatrixToApp(message, sensorMatrixSc, matrixToMove);
                    Serial.println();
                    timerlocal = millis();
                }
                detectChessBoard(sensorMatrixSc);
                pieceArrived = (sensorMatrixSc[squareRowInit][squareColInit] && !sensorMatrixSc[squareRowEnd][squareColEnd]) ? true : false;
            }
        }
        else // Test mode: halt with repeated buzzer warning.
        {
            deactivateAllMagnets();
            Serial.println("ERROR TEST MODE: Too many attempts" + String(squareRowInit) + String(squareColInit) + String(squareRowEnd) + String(squareColEnd));
            BleChess.sendTestModeError("ERROR TEST MODE: Too many attempts" + String(squareRowInit) + String(squareColInit) + String(squareRowEnd) + String(squareColEnd));
            unsigned long timerlocal = millis();
            while (true)
            {
                cute._tone(NOTE_G3, 50, 100);
                if (millis() - timerlocal > 10000)
                {
                    Serial.println("ERROR TEST MODE: Too many attempts" + String(squareRowInit) + String(squareColInit) + String(squareRowEnd) + String(squareColEnd));
                    BleChess.sendTestModeError("ERROR TEST MODE: Too many attempts" + String(squareRowInit) + String(squareColInit) + String(squareRowEnd) + String(squareColEnd));
                    timerlocal = millis();
                }
            }
        }
    }

    // Update the virtual matrix to reflect the completed move.
    matrixToMove[squareRowEnd][squareColEnd] = matrixToMove[squareRowInit][squareColInit];
    matrixToMove[squareRowInit][squareColInit] = '.';

    // Update the previous-state shadow matrix used by play-mode detection.
    previousChessState[squareRowEnd][squareColEnd] = false;
    previousChessState[squareRowInit][squareColInit] = true;

    if (testModeSculpture == false) // Normal mode: verify board consistency.
    {
        if (BleChess.getModeChess() == 1)
        {
            compareMatrixVsSensorsPlus(COMPARE_IMPRIME | COMPARE_HOME | COMPARE_CENTRA, matrixToMove);
            BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrixSc, matrixToMove);
        }
    }
    else // Test mode: just print elapsed time and check the 10-minute limit.
    {
        deactivateAllMagnets();

        unsigned long elapsedTime = millis() - timeSculpture;
        unsigned long minutes = (elapsedTime / 60000) % 60;
        unsigned long seconds = (elapsedTime / 1000) % 60;
        Serial.printf("Elapsed time: %lu min %lu sec\n", minutes, seconds);
        Serial.println("Piece moved successfully.");
        Serial.println();

        if (millis() - timeSculpture > 600000) // 10 minutes
        {
            Serial.println("✓ Sculpture Mode Test Finished");
            BleChess.setMode(4);
            return;
        }
    }
}

/*
 * decodeMovement()
 * ─────────────────────────────────────────────────────────────────────────────
 * Converts a decoded PGN move token into a String of physical movement commands
 * that sculptureMain() can feed directly into simplifiedMovement().
 *
 * Parameters:
 *   x_inichar   – file character of the source square ('a'-'h')
 *   y_inichar   – rank character of the source square ('1'-'8')
 *   x_finchar   – file character of the destination square
 *   y_finchar   – rank character of the destination square
 *   action      – move type: '0'=simple, '1'=capture, '2'=short castle,
 *                             '3'=long castle
 *   piezacolor  – moving side: 'W' (white) or 'B' (black)
 *   matrixToDecode – current virtual board matrix (used for capture graveyard
 *                    placement and en-passant detection)
 *   mode        – true = read real sensors; false = assume all squares empty
 *                 (used during game pre-processing in readFromFileSc)
 *
 * Returns:
 *   A String encoding the physical moves required:
 *     Simple move:  "[r0][c0][r1][c1]/"
 *     Capture:      "[captR][captC][gravR][gravC][r0][c0][r1][c1]/"
 *     Short castle: "58788868/" or "51718161/"
 *     Long  castle: "58381848/" or "51311141/"
 *
 * Side effects: reads sensorMatrixSc when mode==true.
 */
String decodeMovement(char x_inichar, char y_inichar, char x_finchar, char y_finchar, char action, char piezacolor, char matrixToDecode[10][10], bool mode)
{
    int x_ini;
    int y_ini;
    int x_fin;
    int y_fin;
    int x_finaux;
    int y_finaux;
    bool flagAlPassant = false;
    bool lugarDisponible = false;

    String sendMove = "";
    bool chess_color = (piezacolor == 'W') ? true : false; // false = black, true = white

    // Map board file/rank characters to 10×10 matrix indices (1-based).
    char posCoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char posCoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};

    for (int i = 0; i < 8; i++)
    {
        if (x_inichar == posCoordBoardX[i]) x_ini = i + 1;
        if (x_finchar == posCoordBoardX[i]) x_fin = i + 1;
        if (y_inichar == posCoordBoardY[i]) y_ini = i + 1;
        if (y_finchar == posCoordBoardY[i]) y_fin = i + 1;
    }

    // Populate sensorMatrixSc: either from real hardware or assume all empty.
    if (mode == true)
    {
        detectChessBoard(sensorMatrixSc);
    }
    else
    {
        for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                sensorMatrixSc[i][j] = 1;
            }
        }
    }

    //── Capture move ─────────────────────────────────────────────────────────
    if (action == '1')
    {
        // Invert color: we are moving the CAPTURED piece to the graveyard first.
        chess_color = !chess_color;

        if (chess_color == false) // black piece captured; send to black graveyards
        {
            if (matrixToDecode[x_fin][y_fin] == '.') // en passant: no piece at destination
            {
                if (matrixToDecode[x_fin][y_fin + 1] == 'P' || matrixToDecode[x_fin][y_fin + 1] == 'p')
                {
                    Serial.println("Al passant");
                    flagAlPassant = true;
                    y_fin = y_fin + 1; // the captured pawn is one rank behind
                }
                else
                {
                    Serial.println("No hay pieza en el destino, verificar partida.");
                }
            }
            if (matrixToDecode[x_fin][y_fin] == 'P' || matrixToDecode[x_fin][y_fin] == 'p')
            {
                // Pawns go to the bottom graveyard row (row 9).
                for (int i = 1; i <= 8; i++)
                {
                    if (matrixToDecode[i][9] == '.' && sensorMatrixSc[i][9] == 1)
                    {
                        x_finaux = i;
                        y_finaux = 9;
                        lugarDisponible = true;
                    }
                }
            }
            else
            {
                // Non-pawn pieces go to the right graveyard column (col 9).
                for (int i = 1; i <= 8; i++)
                {
                    if (matrixToDecode[9][i] == '.' && sensorMatrixSc[9][i] == 1)
                    {
                        x_finaux = 9;
                        y_finaux = i;
                        lugarDisponible = true;
                    }
                }
            }
        }
        else if (chess_color == true) // white piece captured; send to white graveyards
        {
            if (matrixToDecode[x_fin][y_fin] == '.') // en passant
            {
                if (matrixToDecode[x_fin][y_fin - 1] == 'P' || matrixToDecode[x_fin][y_fin - 1] == 'p')
                {
                    Serial.println("Al passant");
                    flagAlPassant = true;
                    y_fin = y_fin - 1;
                }
                else
                {
                    Serial.println("No hay pieza en el destino, verificar partida.");
                }
            }

            if (matrixToDecode[x_fin][y_fin] == 'P' || matrixToDecode[x_fin][y_fin] == 'p')
            {
                // Pawns go to the top graveyard row (row 0).
                for (int i = 8; i >= 1; i--)
                {
                    if (matrixToDecode[i][0] == '.' && sensorMatrixSc[i][0] == 1)
                    {
                        x_finaux = i;
                        y_finaux = 0;
                        lugarDisponible = true;
                    }
                }
            }
            else
            {
                // Non-pawn pieces go to the left graveyard column (col 0).
                for (int i = 8; i >= 1; i--)
                {
                    if (matrixToDecode[0][i] == '.' && sensorMatrixSc[0][i] == 1)
                    {
                        x_finaux = 0;
                        y_finaux = i;
                        lugarDisponible = true;
                    }
                }
            }
        }

        // Fallback: any available graveyard border cell (not a corner).
        if (!lugarDisponible)
        {
            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < 10; i++)
                {
                    if (matrixToDecode[i][j] == '.' && sensorMatrixSc[i][j] == 1 && (i == 0 || i == 9 || j == 0 || j == 9) && !(i == 0 && j == 0) && !(i == 9 && j == 9) && !(i == 0 && j == 9) && !(i == 9 && j == 0))
                    {
                        x_finaux = i;
                        y_finaux = j;
                        lugarDisponible = true;
                    }
                }
            }
        }

        // First sub-move: captured piece → graveyard.
        sendMove = String(x_fin) + String(y_fin) + String(x_finaux) + String(y_finaux);

        // Restore y_fin to the correct destination for en passant.
        if (flagAlPassant == true)
        {
            if (chess_color == false)
            {
                y_fin = y_fin - 1;
            }
            else if (chess_color == true)
            {
                y_fin = y_fin + 1;
            }
        }

        // Second sub-move: moving piece → destination square.
        sendMove = sendMove + String(x_ini) + String(y_ini) + String(x_fin) + String(y_fin);
    }
    //── Short castling ────────────────────────────────────────────────────────
    else if (action == '2')
    {
        // White: king e1→g1 (58→78) + rook h1→f1 (88→68)
        // Black: king e8→g8 (51→71) + rook h8→f8 (81→61)
        if (chess_color == true)
        {
            sendMove = "58788868";
        }
        else
        {
            sendMove = "51718161";
        }
    }
    //── Long castling ─────────────────────────────────────────────────────────
    else if (action == '3')
    {
        // White: king e1→c1 (58→38) + rook a1→d1 (18→48)
        // Black: king e8→c8 (51→31) + rook a8→d8 (11→41)
        if (chess_color == true)
        {
            sendMove = "58381848";
        }
        else
        {
            sendMove = "51311141";
        }
    }
    //── Simple move ───────────────────────────────────────────────────────────
    else
    {
        sendMove = String(x_ini) + String(y_ini) + String(x_fin) + String(y_fin);
    }

    sendMove = sendMove + "/"; // delimiter
    return sendMove;
}

/*
 * reorderChessboardPlus()
 * ─────────────────────────────────────────────────────────────────────────────
 * Physically rearranges pieces on the board so that matrixToReorder matches
 * the desired target configuration.
 *
 * Uses a nearest-neighbour greedy algorithm (see file header for details).
 * The global targetMatrix is used as a working copy of the desired state;
 * cells already satisfied are marked 'L' to skip them in subsequent passes.
 *
 * Parameters:
 *   mode             – 0 = initial chess position, anything else = custom
 *   targetMatrixAux  – desired board state (ignored when mode == 0; may be
 *                      nullptr when mode == 0)
 *   matrixToReorder  – current board state (modified in-place to reflect
 *                      moves made; restored to target state when done)
 *
 * Side effects: drives motors (via simplifiedMovement), reads sensors, updates
 *               both targetMatrix and matrixToReorder, sets/clears
 *               reorderChessboard flag.
 */
void reorderChessboardPlus(int mode, char targetMatrixAux[10][10], char matrixToReorder[10][10])
{
    // mode 0: Reorder to the initial position
    // mode 1: Reorder white pieces
    // mode 2: Reorder black pieces
    // mode 3: Reorder to a custom position
    reorderChessboard = true;

    Serial.println("-----------------------------------REORDENAMIENTO AUTOMATICO");

    // Standard back-rank piece ordering (files a-h).
    char vectChessW[10] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[10] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};

    // Clear the global target matrix.
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            targetMatrix[i][j] = '.';
        }
    }

    // Populate targetMatrix from mode or from the provided targetMatrixAux.
    if (mode == 0) // Reorder to the initial position
    {
        for (int i = 1; i < 9; i++)
        {
            targetMatrix[i][1] = vectChessB[i - 1];
        }
        for (int i = 1; i < 9; i++)
        {
            targetMatrix[i][2] = 'p';
        }

        for (int i = 1; i < 9; i++)
        {
            targetMatrix[i][8] = vectChessW[i - 1];
        }
        for (int i = 1; i < 9; i++)
        {
            targetMatrix[i][7] = 'P';
        }
    }
    else // Reorder to the custom position provided by the caller.
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                targetMatrix[i][j] = targetMatrixAux[i][j];
            }
        }
    }

    // Mark cells where the piece is already correct so we don't move them.
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            if (targetMatrix[i][j] == matrixToReorder[i][j] && matrixToReorder[i][j] != '.')
            {
                targetMatrix[i][j] = 'L'; // locked – already in place
            }
        }
    }

    Serial.println("Matriz Actual Inicial: ");
    printGenericMatrix(matrixToReorder, 10, 10);

    Serial.println("Matriz Objetivo Inicial: ");
    printGenericMatrix(targetMatrix, 10, 10);

    // ── Main reorder loop: iterate target in column-major order ──────────────
    for (int expColTarget = 0; expColTarget < 10; expColTarget++)
    {
        for (int expRowTarget = 0; expRowTarget < 10; expRowTarget++)
        {
            // Re-scan sensors to account for pieces placed by the user.
            detectChessBoard(sensorMatrixSc);
            for (int i = 0; i < 10; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    // If a cell is empty in the virtual matrix but the sensor
                    // sees a piece there (user placed it), lock that cell.
                    if (matrixToReorder[j][i] == '.' && targetMatrix[j][i] != 'L' && targetMatrix[j][i] != '.' && !sensorMatrixSc[j][i])
                    {
                        targetMatrix[j][i] = 'L';
                        matrixToReorder[j][i] = 'L';
                    }
                }
            }

            // Only act on cells that still need a piece.
            if (targetMatrix[expRowTarget][expColTarget] != '.' && targetMatrix[expRowTarget][expColTarget] != 'L')
            {
                int flagPushOutStyle = 0;
                int blockingFinalPosL = 0;
                int blockingFinalPosK = 0;

                // Find the nearest piece of the required type.
                int nearestPieceInt = findNearestPiecePosition(expRowTarget, expColTarget, targetMatrix[expRowTarget][expColTarget], matrixToReorder);
                int nearestPieceRow = nearestPieceInt / 10;
                int nearestPieceCol = nearestPieceInt % 10;

                if (nearestPieceInt != -1)
                {
                    // Check whether the destination cell is blocked.
                    if (matrixToReorder[expRowTarget][expColTarget] != '.' || sensorMatrixSc[expRowTarget][expColTarget] == 0)
                    {
                        // Look for a cell in the target that needs the blocking piece
                        // AND whose destination is currently empty (push-out optimisation).
                        for (int l = 0; l < 10 && flagPushOutStyle != 1; l++)
                        {
                            for (int k = 0; k < 10 && flagPushOutStyle != 1; k++)
                            {
                                if (targetMatrix[k][l] == matrixToReorder[expRowTarget][expColTarget] && (matrixToReorder[k][l] == '.' || sensorMatrixSc[k][l] == 1))
                                {
                                    flagPushOutStyle = 1;
                                    blockingFinalPosK = k;
                                    blockingFinalPosL = l;
                                }
                            }
                        }

                        if (flagPushOutStyle == 1)
                        {
                            // Move the blocking piece directly to its own target.
                            simplifiedMovement(expRowTarget, expColTarget, blockingFinalPosK, blockingFinalPosL, matrixToReorder);
                            targetMatrix[blockingFinalPosK][blockingFinalPosL] = 'L';

                            // Now the destination is free – move the desired piece.
                            simplifiedMovement(nearestPieceRow, nearestPieceCol, expRowTarget, expColTarget, matrixToReorder);
                            targetMatrix[expRowTarget][expColTarget] = 'L';
                        }
                        else
                        {
                            // No suitable direct target – move blocking piece to any empty space.
                            int emptyPos = findNearestEmptyPosition(expRowTarget, expColTarget, matrixToReorder);
                            int emptyPosRow = emptyPos / 10;
                            int emptyPosCol = emptyPos % 10;

                            simplifiedMovement(expRowTarget, expColTarget, emptyPosRow, emptyPosCol, matrixToReorder);

                            // Destination now empty – move desired piece.
                            simplifiedMovement(nearestPieceRow, nearestPieceCol, expRowTarget, expColTarget, matrixToReorder);
                            targetMatrix[expRowTarget][expColTarget] = 'L';
                        }
                    }
                    else
                    {
                        // Destination empty – simple direct move.
                        simplifiedMovement(nearestPieceRow, nearestPieceCol, expRowTarget, expColTarget, matrixToReorder);
                        targetMatrix[expRowTarget][expColTarget] = 'L';
                    }
                }
                else
                {
                    // No matching piece found anywhere – accept the current state.
                    matrixToReorder[expRowTarget][expColTarget] = targetMatrix[expRowTarget][expColTarget];
                }
            }
        }
    }

    // Clear both matrices and repopulate matrixToReorder with the final state.
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            targetMatrix[i][j] = '.';
            matrixToReorder[i][j] = '.';
        }
    }

    if (mode == 0) // Restore matrixToReorder to the initial chess position.
    {
        if (mode == 0)
        {
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][1] = vectChessB[i - 1];
            }
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][2] = 'p';
            }

            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][8] = vectChessW[i - 1];
            }
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][7] = 'P';
            }
        }
    }
    else // Restore matrixToReorder from the custom target.
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                matrixToReorder[i][j] = targetMatrixAux[i][j];
            }
        }
    }

    reorderChessboard = false;
    Serial.println("-----------------------------------REORDENAMIENTO AUTOMATICO TERMINA");
}

/*
 * centerInitialPiecesSc()
 * ─────────────────────────────────────────────────────────────────────────────
 * Nudges every piece to the centre of its square using a small out-and-back
 * trajectory.  Used after initial setup or mechanical recalibration to ensure
 * pieces are well-seated on their hall-effect sensors.
 *
 * Parameters:
 *   mode           – 0 = initialise from standard start position first
 *                    1 = use the matrix already in matrixToCenter
 *                    2 = only centre pieces where the sensor currently sees a
 *                        piece that differs from the virtual matrix
 *   matrixToCenter – virtual board matrix (updated in-place if mode == 0)
 *
 * Side effects: drives motors, reads sensors, may return early if mode changes.
 */
void centerInitialPiecesSc(int mode, char matrixToCenter[10][10])
{
    int numPoints = 0;
    int numPointsFinal = 0;
    int outPos = 8;
    int centeringDistance = 1; // must be non-zero; otherwise a memory error occurs.
    bool sensorsMatrix[10][10];

    BleChess.setState("Setting Up");
    Serial.println("-----------------------------------CENTRADO");

    if (mode == 0)
    {
        initMatrixPlus(matrixToCenter);
    }

    detectChessBoard(sensorsMatrix);

    // Build a printable auxiliary matrix for debugging.
    char matrixAux[10][10];
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            if ((matrixToCenter[i][j] != '.' && (mode == 0 || mode == 1)) || (matrixToCenter[i][j] != '.' && sensorsMatrix[i][j] && mode == 2))
            {
                matrixAux[i][j] = matrixToCenter[i][j];
            }
            else
            {
                matrixAux[i][j] = '.';
            }
        }
    }
    printGenericMatrix(matrixAux, 10, 10);

    // Iterate column-major, alternating direction each column (boustrophedon)
    // to minimise total travel distance.
    for (int j = 0; j < 10; j++)
    {
        int iStart = (j % 2 == 0) ? 0 : 9;
        int iEnd   = (j % 2 == 0) ? 10 : -1;
        int iStep  = (j % 2 == 0) ? 1 : -1;

        for (int i = iStart; i != iEnd; i += iStep)
        {
            if ((matrixToCenter[i][j] != '.' && (mode == 0 || mode == 1)) || (matrixToCenter[i][j] != '.' && sensorsMatrix[i][j] && mode == 2))
            {
                int coordXBase = (50 * i) - 225;
                int coordYBase = (-50 * j) + 225;

                int coordXEnd = coordXBase;
                int coordYEnd = coordYBase;

                // ── First trajectory: nudge piece slightly away from centre ──
                if (i >= 1 && i <= 8 && j >= 1 && j <= 8) // Inside the board
                {
                    coordXEnd -= outPos;
                    coordYEnd += outPos;
                }
                else if (i == 0) // left graveyard
                {
                    coordYEnd -= outPos - 2;
                }
                else if (i == 9) // right graveyard
                {
                    coordYEnd += outPos + 2;
                }
                else if (j == 0) // top graveyard
                {
                    coordXEnd += outPos;
                    coordYEnd -= outPos;
                }
                else if (j == 9) // bottom graveyard
                {
                    coordXEnd += outPos;
                    coordYEnd += outPos;
                }

                float **trajectoryAux = interpolatePoints(coordXBase, coordYBase, coordXEnd, coordYEnd, numPointsFinal);

                // ── Second trajectory: return piece to (adjusted) centre ──────
                int coordXInit = coordXEnd;
                int coordYInit = coordYEnd;

                coordXEnd = coordXBase;
                coordYEnd = coordYBase;

                if (i >= 1 && i <= 8 && j >= 1 && j <= 8)
                {
                    coordXEnd += centeringDistance;
                    coordYEnd -= centeringDistance;
                }
                else if (i == 0)
                {
                    coordYEnd += centeringDistance + 2;
                }
                else if (i == 9)
                {
                    coordYEnd -= centeringDistance - 2;
                }
                else if (j == 0)
                {
                    coordXEnd -= centeringDistance;
                    coordYEnd += centeringDistance;
                }
                else if (j == 9)
                {
                    coordXEnd -= centeringDistance;
                    coordYEnd -= centeringDistance;
                }

                float **interpolatedPoints = interpolatePoints(coordXInit, coordYInit, coordXEnd, coordYEnd, numPoints);

                // ── Combine both trajectories into one contiguous array ───────
                int totalPoints = numPointsFinal + numPoints;

                float **combinedPoints = new float *[totalPoints];
                for (int i = 0; i < totalPoints; i++)
                {
                    combinedPoints[i] = new float[2];
                }

                for (int i = 0; i < numPointsFinal; i++)
                {
                    combinedPoints[i][0] = trajectoryAux[i][0];
                    combinedPoints[i][1] = trajectoryAux[i][1];
                }
                for (int i = 0; i < numPoints; i++)
                {
                    combinedPoints[i + numPointsFinal][0] = interpolatedPoints[i][0];
                    combinedPoints[i + numPointsFinal][1] = interpolatedPoints[i][1];
                }

                // Free intermediate trajectory buffers.
                if (trajectoryAux != nullptr)
                {
                    for (int i = 0; i < numPointsFinal; i++)
                    {
                        delete[] trajectoryAux[i];
                    }
                    delete[] trajectoryAux;
                    trajectoryAux = nullptr;
                }
                if (interpolatedPoints != nullptr)
                {
                    for (int i = 0; i < numPoints; i++)
                    {
                        delete[] interpolatedPoints[i];
                    }
                    delete[] interpolatedPoints;
                    interpolatedPoints = nullptr;
                }

                // Execute the combined nudge trajectory.
                accelRampV3(combinedPoints, totalPoints, 100);

                if (combinedPoints != nullptr)
                {
                    for (int i = 0; i < totalPoints; i++)
                    {
                        delete[] combinedPoints[i];
                    }
                    delete[] combinedPoints;
                    combinedPoints = nullptr;
                }
            }

            // ── BLE interrupt check: pause or mode-change while centering ─────
            while (BleChess.getModeChess() == 1 && BleChess.getPauseInfo() == 1)
            {
            }
            if (BleChess.getModeChess() != 1)
            {
                Serial.println("----------------------Centrado Interrumpido");
                BleChess.setState("Running");
                Serial.println("----------Change Mode Centrar");
                return;
            }
        }
    }

    sensorOffsetCalib();
    BleChess.setState(BleChess.getPauseInfo() == 1 ? "Paused" : "Running");
    Serial.println("-----------------------------------CENTRADO TERMINA");
}

/*
 * compareMatrixVsSensorsPlus()
 * ─────────────────────────────────────────────────────────────────────────────
 * Compares the virtual board matrix against the physical sensor readings and
 * takes corrective action when they disagree.
 *
 * Positive mode (mode >= 0):
 *   Loops until the sensor matrix matches the virtual matrix.  Uses the
 *   COMPARE_IMPRIME / COMPARE_CENTRA / COMPARE_HOME bit flags (defined in
 *   config.h) to select which recovery actions to take.
 *
 * Negative mode (mode < 0):
 *   Used in interactive play mode.  Waits for the human player to manually
 *   correct the board; plays an audio cue every 5 s; exits when the mode
 *   changes or when pieces are all in position.
 *
 * Parameters:
 *   mode            – bitmask of COMPARE_* flags, or negative for play mode
 *   matrixToCompare – virtual board matrix to validate against sensors
 *
 * Returns:
 *   Number of pieces out of position (0 = all correct, -1 = mode changed).
 */
int compareMatrixVsSensorsPlus(int mode, char matrixToCompare[10][10])
{
    static String newMode = "1";
    int piecesOutOfPos = 0;
    int cyclesUntilExit = 0;
    int difSensMatrix = 0;
    bool matrixPlusSensors[10][10];
    bool sensorMatrixScAux[10][10];

    // Convert the virtual matrix to bool: '.' = empty (true), anything else = occupied (false).
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            matrixPlusSensors[i][j] = matrixToCompare[i][j] == '.';
        }
    }

    unsigned long startTime = millis();

    if (mode >= 0) // ── Autonomous/sculpture mode ────────────────────────────
    {
        do
        {
            piecesOutOfPos = 0;
            cyclesUntilExit++;
            detectChessBoard(sensorMatrixSc);

            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < 10; i++)
                {
                    piecesOutOfPos += (matrixPlusSensors[i][j] != sensorMatrixSc[i][j]);
                    difSensMatrix  += (sensorMatrixScAux[i][j] != sensorMatrixSc[i][j]);
                    sensorMatrixScAux[i][j] = sensorMatrixSc[i][j];
                }
            }

            // Print and act on the first three cycles, whenever the board changes,
            // or every 10 seconds.
            if ((cyclesUntilExit == 1 || cyclesUntilExit == 2 || cyclesUntilExit == 3 || difSensMatrix > 0 || millis() - startTime >= 10000) && (mode & COMPARE_IMPRIME))
            {
                Serial.println();
                if (piecesOutOfPos == 0)
                {
                    Serial.println("Chessboard and sensor matrix match.");
                }
                else
                {
                    Serial.print("ERROR: Chessboard and sensor matrix");

                    if (cyclesUntilExit == 1 && (mode & COMPARE_CENTRA))
                    {
                        Serial.println(" do not match. Centering pieces affected...");
                        centerInitialPiecesSc(2, matrixToCompare);
                    }
                    else if (cyclesUntilExit == 2 && (mode & COMPARE_HOME))
                    {
                        Serial.println(" still do not match. Recalibrating...");
                        mechanicalCalibration(BleChess.getCalibType());
                        Serial.printf("Calibration Finished \n Robot Initialized \n Robot Configured, Centering pieces affected...");
                        centerInitialPiecesSc(2, matrixToCompare);
                    }
                    else if (cyclesUntilExit >= 3)
                    {
                        Serial.println(" still do not match. Please check the board and try again.");
                        BleChess.sendMatrixToApp("ERROR: Chessboard and sensor matrix do not match.", sensorMatrixSc, matrixToCompare);
                    }
                }

                printGenericMatrix(matrixToCompare, 10, 10);
                Serial.println("");
                for (int j = 0; j < 10; j++)
                {
                    for (int i = 0; i < 10; i++)
                    {
                        Serial.print(sensorMatrixSc[i][j]);
                        Serial.print(" ");
                    }
                    Serial.println("");
                }
                Serial.println("");

                difSensMatrix = 0;
                startTime = millis();
            }

            newMode = BleChess.getModeChess();
        } while (piecesOutOfPos > 0 && (mode & COMPARE_IMPRIME) && newMode.toInt() == 1);
    }
    else // ── Interactive play mode ────────────────────────────────────────────
    {
        do
        {
            piecesOutOfPos = 0;
            cyclesUntilExit++;
            detectChessBoard(sensorMatrixSc);

            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < 10; i++)
                {
                    piecesOutOfPos += (matrixPlusSensors[i][j] != sensorMatrixSc[i][j]);
                    difSensMatrix  += (sensorMatrixScAux[i][j] != sensorMatrixSc[i][j]);
                    sensorMatrixScAux[i][j] = sensorMatrixSc[i][j];
                }
            }

            if (piecesOutOfPos == 0)
            {
                Serial.println("Chessboard and sensor matrix match.");
            }
            else if (piecesOutOfPos > 0)
            {
                // Play a warning sound every 5 seconds while the board is wrong.
                if (millis() - startTime >= 5000 || cyclesUntilExit == 1)
                {
                    soundHandler(6);
                    Serial.println("ERROR: Chessboard and sensor matrix do not match. Wait for user to center.");
                    startTime = millis();
                }
                if (difSensMatrix > 0 || cyclesUntilExit == 1)
                {
                    BleChess.sendMatrixToApp("ERROR: Chessboard and sensor matrix do not match.", sensorMatrixSc, matrixToCompare);
                }
            }
            difSensMatrix = 0;

            newMode = BleChess.getModeChess();
            if (globalConnect == "0")
            {
                BleChess.setMode(3);
                return -1;
                break;
            }

            // Play success sound once pieces return to correct positions.
            if (cyclesUntilExit > 1 && piecesOutOfPos == 0)
            {
                soundHandler(5);
            }
        } while (piecesOutOfPos > 0 && newMode.toInt() == 2 && globalConnect == "1");
    }

    return piecesOutOfPos;
}

/*
 * findNearestEmptyPosition()
 * ─────────────────────────────────────────────────────────────────────────────
 * Searches outward from (currentRowAux, currentColAux) in an expanding square
 * pattern to find the nearest cell that is either empty in the virtual matrix
 * OR reported empty by the sensors.
 *
 * Corner cells (0,0), (0,9), (9,0), (9,9) are always skipped.
 *
 * Parameters:
 *   currentRowAux    – row of the centre cell for the search
 *   currentColAux    – column of the centre cell
 *   matrixToSearch   – virtual board matrix to check for empty cells ('.')
 *
 * Returns:
 *   (row * 10 + col) of the nearest empty cell, or -1 if none found within
 *   a search radius of 11.
 */
int findNearestEmptyPosition(int currentRowAux, int currentColAux, char matrixToSearch[10][10])
{
    int currentRow = currentRowAux;
    int currentCol = currentColAux;
    int rango = 1;

    while (true)
    {
        for (int j = -rango; j <= rango; ++j)
        {
            for (int i = -rango; i <= rango; ++i)
            {
                int newRow = currentRow + i;
                int newCol = currentCol + j;

                // Stay within bounds and skip corners.
                if (newRow >= 0 && newRow < 10 && newCol >= 0 && newCol < 10 &&
                    !(newRow == 0 && newCol == 0) && !(newRow == 0 && newCol == 9) &&
                    !(newRow == 9 && newCol == 0) && !(newRow == 9 && newCol == 9))
                {
                    if (matrixToSearch[newRow][newCol] == '.' || sensorMatrixSc[newRow][newCol] == 1)
                    {
                        return (newRow * 10) + newCol;
                    }
                }
            }
        }

        rango++;
        if (rango > 11)
        {
            return -1; // No empty position found within search radius.
        }
    }
}

/*
 * findNearestPiecePosition()
 * ─────────────────────────────────────────────────────────────────────────────
 * Searches outward from (currentRowAux, currentColAux) for the nearest cell
 * that contains targetPiece in the virtual matrix AND is confirmed occupied by
 * the sensor (sensorMatrixSc == 0) AND is not already locked (targetMatrix != 'L').
 *
 * Parameters:
 *   currentRowAux   – row of the centre cell for the search
 *   currentColAux   – column of the centre cell
 *   targetPiece     – the piece character to look for (e.g. 'P', 'n')
 *   matrixToSearch  – virtual board matrix
 *
 * Returns:
 *   (row * 10 + col) of the nearest matching piece, or -1 if none found
 *   within a search radius of 11.
 */
int findNearestPiecePosition(int currentRowAux, int currentColAux, char targetPiece, char matrixToSearch[10][10])
{
    int currentRow = currentRowAux;
    int currentCol = currentColAux;
    int rango = 1;

    while (true)
    {
        for (int j = -rango; j <= rango; ++j)
        {
            for (int i = -rango; i <= rango; ++i)
            {
                int newRow = currentRow + i;
                int newCol = currentCol + j;

                if (newRow >= 0 && newRow < 10 && newCol >= 0 && newCol < 10 &&
                    !(newRow == 0 && newCol == 0) && !(newRow == 0 && newCol == 9) &&
                    !(newRow == 9 && newCol == 0) && !(newRow == 9 && newCol == 9))
                {
                    // Piece must exist in virtual matrix, be confirmed by sensor,
                    // and not already be locked/assigned.
                    if (targetMatrix[newRow][newCol] != 'L' &&
                        matrixToSearch[newRow][newCol] == targetPiece &&
                        sensorMatrixSc[newRow][newCol] == 0)
                    {
                        return (newRow * 10) + newCol;
                    }
                }
            }
        }

        rango++;
        if (rango > 11)
        {
            return -1; // No matching piece found within search radius.
        }
    }
}
