/**
 * @file play_mode.cpp
 * @brief Interactive two-player chess mode (mode 2) implementation.
 *
 * This file implements the full interactive chess play session:
 *
 *   playMode()                   – entry point called from loop(); creates a
 *                                  ChessGameManager and calls run().
 *
 *   ChessGameManager             – state-machine class that manages one complete
 *                                  game session.
 *
 *   detectChangePlus()           – detects physical piece movements on the board
 *                                  using the hall-effect sensor matrix.
 *
 *   automaticMechanicMovement()  – executes a BLE-commanded chess move
 *                                  by physically moving the piece via the magnet
 *                                  carriage.
 *
 *   soundEndGame()               – plays check / checkmate sound if the app
 *                                  signals via UUID_MAKE_SOUND.
 *
 *   stringToMatrix()             – converts a flat 100-character string into
 *                                  the 10×10 board matrix representation.
 *
 * ─── ChessGameManager state machine ─────────────────────────────────────────
 *
 *  run()
 *   ├─ Waits for play-info (who moves first: "1"=BOARD, "2"=BLE)
 *   ├─ initializeGame()
 *   │    ├─ Waits for FEN / matrix string from app (UUID_MATRIX_INIT_GAME)
 *   │    ├─ Parses it with stringToMatrix()
 *   │    └─ setupBoard(): physically arranges pieces to match the FEN using
 *   │         reorderChessboardPlus() + compareMatrixVsSensorsPlus()
 *   └─ Game loop: handleMove() until game over
 *        ├─ Side == BOARD: detectChangePlus() → verifyMovement() → BLE side
 *        └─ Side == BLE:   verifNewCommandBluetooth() → automaticMechanicMovement()
 *                           → BOARD side
 *
 *  checkGameStatus() handles mid-game interruptions:
 *    mode 7  → perform mechanical re-calibration then resume
 *    mode 9  → handle takeback (rewind one move via handleTakeback())
 *    mode ≠ 2 or BLE disconnected → game over
 *
 * ─── detectChangePlus() algorithm ───────────────────────────────────────────
 *
 *  Runs a continuous sensor polling loop until a complete move is detected:
 *   1. If mode changes away from 2, return "".
 *   2. Scan sensorMatrix; compute piece count.
 *   3. If > 3 s since last change and < 32 pieces, play alert sound.
 *   4. Diff against previousChessState; if changed for > 500 ms (debounce),
 *      record lifts (vectorIniChangeX/Y) and placements (vectorFinChangeX/Y).
 *   5. When piece count reaches 32 again, classify the move:
 *      - 1 lift + 1 placement: simple move or capture to/from graveyard.
 *      - 2 lifts + 2 placements with a "same position" overlap: capture
 *        (one position lifted and placed on the same square = the destination
 *        of the capturing piece, the other = the captured piece's trajectory).
 *      - > 2 changes: multi-piece error; play error sound.
 *   6. Update currentMatrix in-place; return the move string.
 *
 * ─── Move string format ──────────────────────────────────────────────────────
 *
 *  "M 1 e2-e4"   simple move (piece on e2 moves to e4)
 *  "M 1 e2xe4"   capture    (piece on e2 captures on e4)
 *  Column letters: a–h (left to right).
 *  Row numbers:    8–1 (top to bottom in the physical matrix, i.e. row 1 of
 *                  the matrix = rank 8 of the chess board).
 *
 * ─── Castling detection ──────────────────────────────────────────────────────
 *
 *  specialMove values set in detectChangePlus():
 *   10 = White short castling (K: e1→g1) in progress
 *   11 = White long  castling (K: e1→c1) in progress
 *   12 = Black short castling (K: e8→g8) in progress
 *   13 = Black long  castling (K: e8→c8) in progress
 *
 *  verifyMovement() skips the BLE validation for the second half of a castling
 *  move (the rook move) by checking specialMoveActive.
 */

// ── Includes ─────────────────────────────────────────────────────────────────
#include <config.h>
#include "BLE.h"
#include "scultpureMode.h"
#include <CuteBuzzerSounds.h>
#include <Arduino.h>
#include <vector>
#include "app_globals.h"

// =============================================================================
// stringToMatrix
// =============================================================================

/**
 * @brief Convert a 100-character flat string into a 10×10 char matrix.
 *
 * The string is read in row-major order: characters 0–9 → row 0,
 * characters 10–19 → row 1, …, characters 90–99 → row 9.
 * Each character is stored at matrix[col][row] (column-major storage).
 *
 * @param str     100-character string encoding the board state.
 * @param matrix  Destination 10×10 matrix (column-major, [col][row]).
 */
void stringToMatrix(String str, char matrix[10][10])
{
    int index = 0;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrix[i][j] = str[index];
            index++;
        }
    }
}

// =============================================================================
// soundEndGame
// =============================================================================

/**
 * @brief Poll UUID_MAKE_SOUND for up to 100 ms and play the end-of-move tone.
 *
 * The chess engine sets UUID_MAKE_SOUND to:
 *   "1" → the move puts the opponent in check      → play check sound
 *   "2" → the move delivers checkmate              → play checkmate sound
 *   ""  → normal move                              → no sound
 *
 * The polling window is intentionally short (100 ms) so that gameplay is not
 * visually delayed; if the BLE notification arrives after the window the sound
 * is simply skipped.
 */
void soundEndGame()
{
    unsigned long timeWaiting = millis();
    String mksound = "";
    while (mksound == "" && millis() - timeWaiting < 100)
    {
        mksound = BleChess.getSoundCheck();
    }
    Serial.println("-------------------------------------------------timeWaiting: " +
                   String(millis() - timeWaiting));
    Serial.println("Make Sound main: " + mksound);
    if (mksound == "1") /* Check */
    {
        Serial.println("Jaque SONIDO ");
        soundHandler(10);
    }
    if (mksound == "2") /* Checkmate */
    {
        Serial.println("Mate SONIDO ");
        soundHandler(11);
    }
}

// =============================================================================
// automaticMechanicMovement
// =============================================================================

/**
 * @brief Execute a BLE-commanded chess move by physically moving a piece.
 *
 * Parses the move string, looks up the piece and move type in the board matrix,
 * generates a full movement plan via decodeMovement(), then executes the move
 * via simplifiedMovement().  Updates the board matrix in the caller's context.
 *
 * Move string format examples:
 *   "X Wx e2-e4"   (simple move, piece on e2 goes to e4)
 *   "X Wx e2xe4"   (capture, piece on e2 takes on e4)
 * where:
 *   [0] = command type (ignored; typically 'M' from the BLE protocol)
 *   [2] = start column letter  (a–h)
 *   [3] = start rank   digit   (1–8)
 *   [4] = separator: '-' (simple), 'x' (capture)
 *   [5] = end column letter
 *   [6] = end rank digit
 *
 * Castling is detected when the king ('K'/'k') moves two squares sideways.
 * In that case the rook move is encoded implicitly in decodeMovement().
 *
 * @param movementString        Encoded move from the BLE engine.
 * @param matrixToAutomaticMove Board matrix; updated in-place after the move.
 */
void automaticMechanicMovement(String movementString, char matrixToAutomaticMove[10][10])
{
    char colorTurn;
    char piezaChess = 'v';    /* The piece being moved (defaulted to 'v' = unknown) */
    char movementChess = '!'; /* '!' = invalid/unrecognised move type */

    /* Convert String to a C-string for character indexing. */
    int n = movementString.length();
    char array[n + 1];
    strcpy(array, movementString.c_str());

    /* Extract the four coordinate characters from the move string. */
    char x_iniChar = array[2]; /* Start column letter (a–h) */
    char y_iniChar = array[3]; /* Start rank   digit  (1–8) */
    char x_finChar = array[5]; /* End column letter   (a–h) */
    char y_finChar = array[6]; /* End rank   digit    (1–8) */

    // ── Translate letters/digits to 1-based matrix indices ───────────────────
    int x_ini = 0, x_fin = 0, y_ini = 0, y_fin = 0;

    /* Column mapping: 'a'=index 1, 'b'=2, …, 'h'=8 */
    char posCoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    /* Rank mapping: '8'=index 1 (top), '7'=2, …, '1'=8 (bottom) */
    char posCoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};

    for (int i = 0; i < 8; i++)
    {
        if (x_iniChar == posCoordBoardX[i]) x_ini = i + 1;
        if (x_finChar == posCoordBoardX[i]) x_fin = i + 1;
        if (y_iniChar == posCoordBoardY[i]) y_ini = i + 1;
        if (x_finChar == posCoordBoardY[i]) y_fin = i + 1; /* Note: uses x_finChar for y-rank lookup */
    }

    // ── Identify piece and move colour ───────────────────────────────────────
    piezaChess = matrixToAutomaticMove[x_ini][y_ini];
    if (piezaChess == '.')
    {
        /* No piece found at source – default to knight to avoid crash. */
        piezaChess = 'N';
    }
    if (isUpperCase(piezaChess))
    {
        colorTurn = 'W'; /* White piece */
    }
    else if (isLowerCase(piezaChess))
    {
        colorTurn = 'B'; /* Black piece */
    }

    // ── Classify the move type ────────────────────────────────────────────────
    if (array[4] == '-') /* Simple displacement, no capture */
    {
        movementChess = '0';
    }
    if (array[4] == 'x') /* Capture: opponent piece at destination */
    {
        movementChess = '1';
        prevDeadPiecesCount++; /* Track that a capture occurred */
    }

    /* Castling detection: king moves two files left or right.
     * The castling move type replaces the simple/capture type: */
    if (piezaChess == 'K' || piezaChess == 'k')
    {
        if (x_iniChar == 'e' && x_finChar == 'g' && y_iniChar == '1' && y_finChar == '1')
            movementChess = '2'; /* White short castling */
        if (x_iniChar == 'e' && x_finChar == 'c' && y_iniChar == '1' && y_finChar == '1')
            movementChess = '3'; /* White long castling  */
        if (x_iniChar == 'e' && x_finChar == 'g' && y_iniChar == '8' && y_finChar == '8')
            movementChess = '2'; /* Black short castling */
        if (x_iniChar == 'e' && x_finChar == 'c' && y_iniChar == '8' && y_finChar == '8')
            movementChess = '3'; /* Black long castling  */
    }

    if (movementChess == '!')
    {
        /* Could not decode the move – bail out safely. */
        Serial.println("Movimiento: " + String(movementChess));
        Serial.println("===================================NO DECODIFICADO, RETURN");
        return;
    }

    Serial.println("Movimiento: " + String(movementChess));

    // ── Generate and execute the physical movement ────────────────────────────
    Serial.println("Movimiento automatico, Initial Matrix: ");
    printGenericMatrix(matrixToAutomaticMove, 10, 10);

    /* decodeMovement() returns an encoded string of sub-moves (each 4 digits =
     * one simplifiedMovement() call).  A '/' separates the main move from an
     * optional second sub-move (used for captures and castling). */
    String fullMoves = decodeMovement(
        x_iniChar, y_iniChar, x_finChar, y_finChar,
        movementChess, colorTurn, matrixToAutomaticMove, true);
    Serial.println("Movimiento FULL: " + fullMoves);

    /* Execute the primary sub-move (first 4 encoded digits). */
    simplifiedMovement(
        fullMoves[0] - '0', fullMoves[1] - '0',
        fullMoves[2] - '0', fullMoves[3] - '0',
        matrixToAutomaticMove);
    Serial.println("Movimiento Simple: ");
    printGenericMatrix(matrixToAutomaticMove, 10, 10);

    /* Execute the optional secondary sub-move if present (no '/' at index 4). */
    if (fullMoves[4] != '/')
    {
        simplifiedMovement(
            fullMoves[4] - '0', fullMoves[5] - '0',
            fullMoves[6] - '0', fullMoves[7] - '0',
            matrixToAutomaticMove);
        Serial.println("Movimiento Comer: ");
        printGenericMatrix(matrixToAutomaticMove, 10, 10);
    }

    /* Send the updated matrix to the app so it can refresh its view. */
    BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrix, matrixToAutomaticMove);
}

// =============================================================================
// detectChangePlus
// =============================================================================

/**
 * @brief Detect a complete physical chess move from the sensor matrix.
 *
 * Runs in a tight polling loop until:
 *   • A valid move has been fully completed (all 32 pieces on the board again),
 *   • A voice command arrives via BLE, or
 *   • The BLE mode changes away from 2.
 *
 * The 500 ms debounce threshold (matrixChangeThreshold) ensures that transient
 * sensor readings during a piece lift/place are not mis-classified as moves.
 *
 * @param[in,out] currentMatrix  Current board state; updated in-place after a
 *                               successful move is identified.
 * @param[out]    specialMove    Set to 10–13 if a castling king-lift is
 *                               detected (caller uses this to suppress
 *                               validation until the rook is also moved).
 * @return  Move string (e.g. "M 1 e2-e4") or "" if no move / mode changed.
 */
String detectChangePlus(char currentMatrix[10][10], int &specialMove)
{
    // Accumulate detected lifts and placements within the current move.
    std::vector<int> vectorFinChangeX; /* Columns where pieces were placed    */
    std::vector<int> vectorFinChangeY; /* Rows    where pieces were placed    */
    std::vector<int> vectorIniChangeX; /* Columns where pieces were lifted    */
    std::vector<int> vectorIniChangeY; /* Rows    where pieces were lifted    */
    int countPieces = 0;

    unsigned long startTime = millis();
    unsigned long lastTimeFunctionRan = startTime;
    unsigned long lastSoundTime = 0;

    /* Minimum time (ms) a sensor difference must persist to be accepted as a
     * real event (filters vibration and transient sensor noise). */
    unsigned long matrixChangeStartTime = 0;
    const unsigned long matrixChangeThreshold = 500;

    String comandoJugada = ""; /* The generated move command string */

    while (true)
    {
        // ── BLE mode guard ────────────────────────────────────────────────────
        /* If the app changed mode, abort immediately. */
        int newData = BleChess.getModeChess();
        if (newData != 2)
        {
            return "";
        }
        if (globalConnect == "0")
        {
            /* BLE disconnected – force pause mode so the board doesn't keep moving. */
            BleChess.setMode(3);
            return "";
        }

        // ── Refresh sensor matrix ─────────────────────────────────────────────
        detectChessBoard(sensorMatrix);

        /* Count pieces currently on the board. */
        countPieces = 0;
        for (int j = 0; j < 10; j++)
            for (int i = 0; i < 10; i++)
                if (sensorMatrix[i][j] == 0)
                    countPieces++;

        // ── Alert if pieces are missing for > 3 s ────────────────────────────
        if (millis() - lastTimeFunctionRan >= 3000)
        {
            if (countPieces < 32)
            {
                if (millis() - lastSoundTime >= 5000)
                {
                    soundHandler(6); /* "something is wrong" alert tone */
                    Serial.print("More than 5 sec without piece");
                    BleChess.sendMatrixToApp(
                        "ERROR: Chessboard and sensor matrix do not match.",
                        sensorMatrix, currentMatrix);
                    lastSoundTime = millis();
                }
            }
            lastTimeFunctionRan = millis();
        }

        // ── Detect matrix changes ─────────────────────────────────────────────
        bool matrixChanged = false;
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (previousChessState[i][j] != sensorMatrix[i][j])
                {
                    matrixChanged = true;
                    break;
                }
            }
            if (matrixChanged) break;
        }

        if (matrixChanged)
        {
            /* Debounce: only accept the change if it has persisted for at least
             * matrixChangeThreshold ms since the first sign of change. */
            if ((millis() - matrixChangeStartTime) >= matrixChangeThreshold)
            {
                matrixChangeStartTime = millis();

                /* Classify each changed cell as a lift or a placement. */
                for (int j = 0; j < 10; j++)
                {
                    for (int i = 0; i < 10; i++)
                    {
                        if (previousChessState[i][j] != sensorMatrix[i][j])
                        {
                            if (sensorMatrix[i][j] == false) /* Piece placed onto square */
                            {
                                lastTimeFunctionRan = millis();
                                lastSoundTime = millis();
                                finChangeX = i;
                                finChangeY = j;
                                vectorFinChangeX.push_back(i);
                                vectorFinChangeY.push_back(j);
                                Serial.print("Quantity: ");
                                Serial.println(countPieces);
                                if (countPieces == 31)
                                {
                                    /* One piece mid-air – play placement prompt. */
                                    soundHandler(5);
                                }
                            }
                            if (sensorMatrix[i][j] == true) /* Piece lifted from square */
                            {
                                lastTimeFunctionRan = millis();
                                lastSoundTime = millis();
                                iniChangeX = i;
                                iniChangeY = j;
                                vectorIniChangeX.push_back(i);
                                vectorIniChangeY.push_back(j);
                                soundHandler(5); /* Lift sound */
                            }
                        }
                    }
                }

                /* Snapshot the new sensor state. */
                for (int jj = 0; jj < 10; jj++)
                    for (int ii = 0; ii < 10; ii++)
                        previousChessState[ii][jj] = sensorMatrix[ii][jj];

                /* Once all 32 pieces are back on the board, the move is complete. */
                if (countPieces == 32)
                {
                    break;
                }
            }
        }

        /* A voice command from the app can also terminate the wait. */
        String voiceCommand = BleChess.getVoiceCommand();
        if (voiceCommand != "")
        {
            break;
        }
    }

    /* Sync the app's matrix view. */
    BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrix, currentMatrix);

    // ── Classify and encode the detected move ─────────────────────────────────
    if (vectorFinChangeX.size() == 0 && vectorIniChangeX.size() == 0)
    {
        return comandoJugada; /* No changes accumulated – voice command exit */
    }

    if (vectorFinChangeX.size() != vectorIniChangeX.size())
    {
        /* Dimension mismatch – log diagnostic info and return without a command. */
        Serial.println("Error, the dimensios is not the same, could cause a crash.");
        Serial.print("REceive matrix from chessboard and show modal will help to solve the matrix?????????");
        Serial.println("Here only will compare chessboard, so chessboard should match and if count the 32 pieces pass");
        Serial.print("Because is possible a false movement in the captured pieces so, only count chessboard and bin sensrs");
        Serial.print("Start:");
        for (int i = 0; i < (int)vectorIniChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorIniChangeX[i]);
            Serial.print(",");
            Serial.print(vectorIniChangeY[i]);
            Serial.print(")");
            Serial.print(",");
        }
        Serial.println("");
        Serial.print("Finnd:");
        for (int i = 0; i < (int)vectorFinChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorFinChangeX[i]);
            Serial.print(",");
            Serial.print(vectorFinChangeY[i]);
            Serial.print(")");
            Serial.print(",");
        }
        return comandoJugada;
    }

    /* Matched sizes – analyse the change vectors. */
    bool flagMove = false;
    bool flagGenerateCommand = false;
    bool flagCaptured = false;
    int iniChangeXCaptured = -1;
    int iniChangeYCaptured = -1;
    int finChangeXCaptured = -1;
    int finChangeYCaptured = -1;

    /* Reject moves that start or end in a corner (unreachable by the mechanism). */
    for (int i = 0; i < (int)vectorIniChangeX.size(); i++)
    {
        if ((vectorIniChangeX[i] == 0 && vectorIniChangeY[i] == 0) ||
            (vectorIniChangeX[i] == 0 && vectorIniChangeY[i] == 9) ||
            (vectorIniChangeX[i] == 9 && vectorIniChangeY[i] == 0) ||
            (vectorIniChangeX[i] == 9 && vectorIniChangeY[i] == 9))
        {
            soundHandler(4); /* "invalid position" alert */
        }
    }
    for (int i = 0; i < (int)vectorFinChangeX.size(); i++)
    {
        if ((vectorFinChangeX[i] == 0 && vectorFinChangeY[i] == 0) ||
            (vectorFinChangeX[i] == 0 && vectorFinChangeY[i] == 9) ||
            (vectorFinChangeX[i] == 9 && vectorFinChangeY[i] == 0) ||
            (vectorFinChangeX[i] == 9 && vectorFinChangeY[i] == 9))
        {
            soundHandler(4);
        }
    }

    if (vectorFinChangeX.size() == 1)
    {
        /* Simple case: one lift and one placement. */
        int iniX = vectorIniChangeX[0];
        int iniY = vectorIniChangeY[0];
        int finX = vectorFinChangeX[0];
        int finY = vectorFinChangeY[0];

        if (((iniX > 0 && iniX < 9) && (iniY > 0 && iniY < 9)) &&
            ((finX > 0 && finX < 9) && (finY > 0 && finY < 9)))
        {
            /* Piece moved within the 8×8 playing field. */
            Serial.println("Simple movement");
            flagMove = true;
            flagGenerateCommand = true;
        }
        else if (((iniX > 0 && iniX < 9) && (iniY > 0 && iniY < 9)) &&
                 ((finX == 0 || finX == 9) || (finY == 0 || finY == 9)))
        {
            /* Piece moved from board to graveyard border = capture. */
            Serial.println("Piece captured");
            flagMove = true;
            soundHandler(5);
        }
        else if (((iniX == 0 || iniX == 9) || (iniY == 0 || iniY == 9)) &&
                 ((finX > 0 && finX < 9) && (finY > 0 && finY < 9)))
        {
            /* Piece returned from graveyard (undo / takeback scenario). */
            Serial.println("Piece returned from captured");
            flagMove = true;
            soundHandler(5);
        }
        else
        {
            Serial.println("Invalid movement");
            soundHandler(4);
            delay(50);
        }

        iniChangeX = vectorIniChangeX[0];
        iniChangeY = vectorIniChangeY[0];
        finChangeX = vectorFinChangeX[0];
        finChangeY = vectorFinChangeY[0];
    }

    if (vectorFinChangeX.size() == 2)
    {
        /* Two lifts and two placements: could be a capture where one position
         * appears in both vectors (the destination of the capturing piece). */
        flagGenerateCommand = true;
        int indexSamePositionStart = -1;
        int indexSamePositionEnd = -1;

        /* Log all positions for debugging. */
        Serial.print("Start:");
        for (int i = 0; i < (int)vectorIniChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorIniChangeX[i]);
            Serial.print(",");
            Serial.print(vectorIniChangeY[i]);
            Serial.print(")");
            Serial.print(",");
            /* Find a position that appears in both the start and end vectors. */
            for (int j = 0; j < (int)vectorFinChangeX.size(); j++)
            {
                if ((vectorIniChangeX[i] == vectorFinChangeX[j]) &&
                    (vectorIniChangeY[i] == vectorFinChangeY[j]))
                {
                    indexSamePositionStart = i;
                    indexSamePositionEnd = j;
                }
            }
        }
        Serial.println("");
        Serial.print("Finnd:");
        for (int i = 0; i < (int)vectorFinChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorFinChangeX[i]);
            Serial.print(",");
            Serial.print(vectorFinChangeY[i]);
            Serial.print(")");
            Serial.print(",");
        }
        Serial.println("");

        if (indexSamePositionStart > -1)
        {
            /* The "same position" is where the capturing piece lands.
             * The other position in the start vector is where the capturing
             * piece started; the other position in the end vector is where
             * the captured piece ended up (graveyard). */
            if (indexSamePositionStart == 0)
            {
                flagMove = true;
                flagCaptured = true;
                iniChangeX = vectorIniChangeX[1];
                iniChangeY = vectorIniChangeY[1];
                finChangeX = vectorIniChangeX[0];
                finChangeY = vectorIniChangeY[0];
                iniChangeXCaptured = vectorIniChangeX[0];
                iniChangeYCaptured = vectorIniChangeY[0];
                finChangeXCaptured = (indexSamePositionEnd == 0)
                                         ? vectorFinChangeX[1]
                                         : vectorFinChangeX[0];
                finChangeYCaptured = (indexSamePositionEnd == 0)
                                         ? vectorFinChangeY[1]
                                         : vectorFinChangeY[0];
            }
            if (indexSamePositionStart == 1)
            {
                flagMove = true;
                flagCaptured = true;
                iniChangeX = vectorIniChangeX[0];
                iniChangeY = vectorIniChangeY[0];
                finChangeX = vectorIniChangeX[1];
                finChangeY = vectorIniChangeY[1];
                iniChangeXCaptured = vectorIniChangeX[1];
                iniChangeYCaptured = vectorIniChangeY[1];
                finChangeXCaptured = (indexSamePositionEnd == 0)
                                         ? vectorFinChangeX[1]
                                         : vectorFinChangeX[0];
                finChangeYCaptured = (indexSamePositionEnd == 0)
                                         ? vectorFinChangeY[1]
                                         : vectorFinChangeY[0];
            }
        }
        else
        {
            soundHandler(6);
            Serial.print("Error not find same position logic TODO check logic");
        }
    }

    if (vectorFinChangeX.size() > 2)
    {
        /* Three or more changes: something went wrong (multiple pieces moved?). */
        soundHandler(6);
        Serial.print("Multiple movements detected, check logic");
        Serial.print("Start:");
        for (int i = 0; i < (int)vectorIniChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorIniChangeX[i]);
            Serial.print(",");
            Serial.print(vectorIniChangeY[i]);
            Serial.print(")");
            Serial.print(",");
        }
        Serial.println("");
        Serial.print("Finnd:");
        for (int i = 0; i < (int)vectorFinChangeX.size(); i++)
        {
            Serial.print("(");
            Serial.print(vectorFinChangeX[i]);
            Serial.print(",");
            Serial.print(vectorFinChangeY[i]);
            Serial.print(")");
            Serial.print(",");
        }
    }

    // ── Update currentMatrix to reflect the detected move ────────────────────
    if (flagCaptured)
    {
        /* Move the captured piece to its graveyard destination in the matrix. */
        char temp = currentMatrix[iniChangeXCaptured][iniChangeYCaptured];
        currentMatrix[finChangeXCaptured][finChangeYCaptured] = temp;
    }

    if (flagMove)
    {
        char piezaEnMovimiento = 'v';
        if (flagGenerateCommand)
        {
            /* Build the move command string ("M 1 e2-e4" format). */
            piezaEnMovimiento = currentMatrix[iniChangeX][iniChangeY];
            char CoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
            char CoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};

            char arrayComandoJugada[9];
            arrayComandoJugada[0] = 'M';
            arrayComandoJugada[1] = ' ';
            arrayComandoJugada[2] = '1';
            arrayComandoJugada[3] = ' ';
            arrayComandoJugada[4] = CoordBoardX[iniChangeX - 1];
            arrayComandoJugada[5] = CoordBoardY[iniChangeY - 1];
            arrayComandoJugada[6] = (flagCaptured == 0) ? '-' : 'x';
            arrayComandoJugada[7] = CoordBoardX[finChangeX - 1];
            arrayComandoJugada[8] = CoordBoardY[finChangeY - 1];
            for (int k = 0; k < 9; k++)
                comandoJugada = comandoJugada + arrayComandoJugada[k];

            /* Detect castling: if the king moved two files, set specialMove so
             * verifyMovement() knows to accept the subsequent rook move without
             * re-asking the engine for validation. */
            if (piezaEnMovimiento == 'K') /* White king */
            {
                if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'g')
                    specialMove = 10; /* White short castling */
                if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'c')
                    specialMove = 11; /* White long castling  */
            }
            if (piezaEnMovimiento == 'k') /* Black king */
            {
                if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'g')
                    specialMove = 12; /* Black short castling */
                if (CoordBoardX[iniChangeX - 1] == 'e' && CoordBoardX[finChangeX - 1] == 'c')
                    specialMove = 13; /* Black long castling  */
            }
        }

        /* Update the logical board matrix. */
        piezaEnMovimiento = currentMatrix[iniChangeX][iniChangeY];
        currentMatrix[iniChangeX][iniChangeY] = '.';
        currentMatrix[finChangeX][finChangeY] = piezaEnMovimiento;
    }

    return comandoJugada;
}

// =============================================================================
// GameState – plain data struct shared by ChessGameManager
// =============================================================================

/**
 * @brief Holds all mutable state for one game session.
 *
 * Kept as a plain struct (no methods) so that the state is easy to inspect
 * in a debugger and could be serialised if needed in future.
 */
struct GameState
{
    char matrix[10][10]; ///< Current 10×10 board matrix (column-major [col][row])

    /** Which side is expected to move next. */
    enum class Side
    {
        BOARD = 1, ///< Human player moves a physical piece
        BLE   = 2  ///< BLE engine (remote player / computer) sends the move
    } currentSide;

    int  specialMove;               ///< Castling type in progress (10–13) or 0
    bool specialMoveActive = false; ///< True while the second half of a castling is pending
    bool isGameOver;                ///< True when the game has ended
};

// =============================================================================
// ChessGameManager – single-game session controller
// =============================================================================

/**
 * @brief Manages the complete lifecycle of one interactive chess game.
 *
 * The public interface is a single run() call; all internal state lives in
 * the private GameState struct.
 */
class ChessGameManager
{
private:
    GameState  state;
    Bluetooth &bleManager;

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * @brief Initialise the board for a new game.
     *
     * Waits for the app to send the initial board FEN/matrix via
     * UUID_MATRIX_INIT_GAME, then physically arranges the pieces using
     * reorderChessboardPlus() and verifies sensor agreement.
     *
     * @return true on success, false if mode changed before init completed.
     */
    bool initializeGame()
    {
        initMatrixPlus(state.matrix);

        String matrixGame = waitForValidMatrix();
        if (matrixGame.isEmpty())
        {
            Serial.println("Failed to get valid initial matrix");
            return false;
        }
        bleManager.resetMatrixInitGame();

        setupBoard(matrixGame);
        state.isGameOver = false;
        return true;
    }

    /**
     * @brief Spin until the app sends a non-empty matrix init string.
     * @return The matrix string, or "" if the mode changed.
     */
    String waitForValidMatrix()
    {
        while (true)
        {
            String matrix = bleManager.getMatrixInitGame();
            if (matrix != "" && matrix != String('\0'))
                return matrix;

            if (!checkGameStatus())
                return "";
            delay(100);
        }
    }

    /** Debug helper: print a 10×10 matrix to Serial. */
    void printMatrix(char matrix[10][10])
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                Serial.print(matrix[i][j]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    /**
     * @brief Parse the init matrix string and physically arrange pieces.
     *
     * Converts the 100-char string to a matrix, calls reorderChessboardPlus()
     * to move pieces to their starting positions, then verifies sensor readings
     * match the expected matrix.
     *
     * @param matrixGame  100-char board state string from the app.
     */
    void setupBoard(const String &matrixGame)
    {
        char initMatrix[10][10] = {0};
        stringToMatrix(matrixGame, initMatrix);
        printMatrix(initMatrix);

        bleManager.setState("Setting Up");
        /* Rearrange pieces to match initMatrix (mode 3 = rearrange to target). */
        reorderChessboardPlus(3, initMatrix, state.matrix);
        bleManager.resetMatrixInitGame();
        bleManager.resetVerifNewCommandBluetooth();

        /* Verify sensors match; nudge any pieces that are off-centre. */
        compareMatrixVsSensorsPlus(-1, state.matrix);
        detectChessBoard(sensorMatrix);
        bleManager.sendMatrixToApp("CLEAN: Match.", sensorMatrix, state.matrix);
        bleManager.setState("Playing");

        /* Snapshot the sensor state so detectChangePlus() has a baseline. */
        for (int j = 0; j < 10; j++)
            for (int i = 0; i < 10; i++)
                previousChessState[i][j] = sensorMatrix[i][j];
    }

    /**
     * @brief Send the move to the app for legality validation.
     *
     * Writes the move string to UUID_STATUS_BOARD and polls UUID_CHECK_MOVE
     * for "1" (legal) or "2" (illegal).  Castling rook-moves are auto-accepted.
     *
     * @param move  The move command string (e.g. "M 1 e2-e4").
     * @return true = legal, false = illegal.
     */
    bool verifyMovement(const String &move)
    {
        if (move.length() < 6)
            return false;

        char x1 = move.charAt(4), y1 = move.charAt(5);
        char x2 = move.charAt(7), y2 = move.charAt(8);

        if (!isValidCoordinate(x1, y1) || !isValidCoordinate(x2, y2))
            return false;

        /* Second half of a castling move: accept without asking the engine. */
        if (state.specialMoveActive)
        {
            state.specialMoveActive = false;
            state.specialMove = 0;
            return true;
        }

        /* First half of a castling move: mark it so the rook move is auto-accepted. */
        if (state.specialMove > 0 && !state.specialMoveActive)
            state.specialMoveActive = true;

        /* Ask the BLE engine whether the move is legal. */
        bleManager.setCheckMove("0");
        bleManager.setStatus(move);
        while (true)
        {
            String response = bleManager.getCheckMove();
            if (response == "1")
                return true;
            if (response == "2")
            {
                soundHandler(4); /* Illegal move alert sound */
                return false;
            }
            delay(100);
        }
    }

    /** @return true if (x,y) is a valid algebraic coordinate ('a'–'h', '1'–'8'). */
    bool isValidCoordinate(char x, char y)
    {
        return (x >= 'a' && x <= 'h') && (y >= '1' && y <= '8');
    }

    /**
     * @brief Wait for the next move from either the board or the BLE engine.
     *
     * On the BOARD side: polls detectChangePlus() (and voice commands) until
     * a valid, legal move is detected, then switches to BLE side.
     *
     * On the BLE side: polls verifNewCommandBluetooth() until a move arrives,
     * executes it mechanically, then switches to BOARD side.
     *
     * @return true to continue playing, false to end the game.
     */
    bool handleMove()
    {
        while (true)
        {
            if (state.currentSide == GameState::Side::BOARD)
            {
                Serial.println("Waiting for board move");
                String move = detectChangePlus(state.matrix, state.specialMove);
                /* Voice commands bypass normal move detection. */
                String voiceCmd = bleManager.getVoiceCommand();
                if (voiceCmd.length() > 0)
                {
                    Serial.printf("Voice Command: %s\n", voiceCmd.c_str());
                    automaticMechanicMovement(voiceCmd, state.matrix);
                    state.currentSide = GameState::Side::BLE;
                    bleManager.resetVoiceCommand();
                    move = "1"; /* Dummy value – signals success */
                    return true;
                }
                if (move.length() > 0 && verifyMovement(move))
                {
                    soundHandler(5); /* Move accepted sound */
                    if (state.specialMoveActive)
                        continue; /* Await rook move (castling) */
                    soundEndGame();
                    state.currentSide = GameState::Side::BLE;
                    return true;
                }
            }
            else if (state.currentSide == GameState::Side::BLE)
            {
                Serial.println("Waiting for BLE move");
                String move = bleManager.verifNewCommandBluetooth();
                if (move.length() > 0)
                {
                    Serial.printf("BLE Move: %s\n", move.c_str());
                    automaticMechanicMovement(move, state.matrix);
                    bleManager.resetVerifNewCommandBluetooth();
                    state.currentSide = GameState::Side::BOARD;
                    soundEndGame();
                    return true;
                }
            }

            if (!checkGameStatus())
                return false;
            delay(1000);
        }
    }

    /**
     * @brief Update the center 8×8 of state.matrix from a FEN string,
     *        preserving the graveyard border pieces.
     *
     * The FEN position is mapped to matrix columns 1–8 (a–h) and rows 1–8
     * (rank 8 = row 1, rank 1 = row 8).  The graveyard border (row 0, row 9,
     * column 0, column 9) is saved before parsing and restored afterwards.
     *
     * @param fen  Standard FEN string (only the position part is used).
     */
    void updateMatrixFromFen(const String &fen)
    {
        /* Save border (graveyard) pieces. */
        char graveyardTop[10];
        char graveyardBottom[10];
        char graveyardLeft[8];
        char graveyardRight[8];

        for (int i = 0; i < 10; i++)
        {
            graveyardTop[i]    = state.matrix[i][0];
            graveyardBottom[i] = state.matrix[i][9];
        }
        for (int i = 1; i < 9; i++)
        {
            graveyardLeft[i - 1]  = state.matrix[0][i];
            graveyardRight[i - 1] = state.matrix[9][i];
        }

        /* Parse the FEN position part (everything before the first space). */
        String fenPosition = fen.substring(0, fen.indexOf(' '));
        int row = 1, col = 1;

        /* Clear the 8×8 playing area. */
        for (int i = 1; i <= 8; i++)
            for (int j = 1; j <= 8; j++)
                state.matrix[i][j] = '.';

        /* Populate from FEN: '/' increments row, digits skip empty squares,
         * letters place pieces. */
        for (char c : fenPosition)
        {
            if (c == '/')
            {
                row++;
                col = 1;
            }
            else if (isdigit(c))
            {
                col += c - '0';
            }
            else
            {
                state.matrix[col][row] = c;
                col++;
            }
        }

        /* Restore graveyard border. */
        for (int i = 0; i < 10; i++)
        {
            state.matrix[i][0] = graveyardTop[i];
            state.matrix[i][9] = graveyardBottom[i];
        }
        for (int i = 1; i < 9; i++)
        {
            state.matrix[0][i] = graveyardLeft[i - 1];
            state.matrix[9][i] = graveyardRight[i - 1];
        }
    }

    /**
     * @brief Remove one occurrence of @p pieceChar from the graveyard border.
     *
     * Searches the four border rows/columns and clears the first matching cell.
     * Used during takeback processing to un-capture a piece.
     *
     * @param pieceChar  Piece character to remove (e.g. 'P', 'p', 'R', etc.).
     */
    void eliminateFromGrave(char pieceChar)
    {
        const int borders[4][2] = {
            {0, -1}, /* Top row    (row 0)    */
            {9, -1}, /* Bottom row (row 9)    */
            {-1, 0}, /* Left col   (col 0)    */
            {-1, 9}  /* Right col  (col 9)    */
        };

        bool found = false;

        /* Check top and bottom rows. */
        for (int i = 0; i < 2 && !found; i++)
        {
            if (borders[i][0] != -1)
            {
                for (int j = 1; j < 9 && !found; j++)
                {
                    if (state.matrix[j][borders[i][0]] == pieceChar)
                    {
                        state.matrix[j][borders[i][0]] = '.';
                        found = true;
                    }
                }
            }
        }

        /* Check left and right columns. */
        for (int i = 2; i < 4 && !found; i++)
        {
            if (borders[i][1] != -1)
            {
                for (int j = 1; j < 9 && !found; j++)
                {
                    if (state.matrix[borders[i][1]][j] == pieceChar)
                    {
                        state.matrix[borders[i][1]][j] = '.';
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            Serial.print("Removed piece from graveyard: ");
            Serial.println(pieceChar);
        }
        else
        {
            Serial.print("Piece not found in graveyard: ");
            Serial.println(pieceChar);
        }
    }

    /** @brief memcpy-based 10×10 matrix copy helper. */
    void copyMatrix(char source[10][10], char dest[10][10])
    {
        for (int i = 0; i < 10; i++)
            memcpy(dest[i], source[i], 10 * sizeof(char));
    }

    /**
     * @brief Handle a takeback request from the app.
     *
     * The takeback message format (from UUID_TAKEBACK) is:
     *   "capturedPieces,newFen,side"
     * where:
     *   capturedPieces  = space-separated list of piece chars to remove from
     *                     the graveyard (e.g. "P p" or "N")
     *   newFen          = FEN string of the board after the takeback
     *   side            = "1" if BOARD moves next, "2" if BLE moves next
     *
     * After parsing, the board is physically rearranged to the new position.
     */
    void handleTakeback()
    {
        char originalMatrix[10][10] = {0};
        copyMatrix(state.matrix, originalMatrix);

        /* Wait for the takeback message. */
        String takebackStr = "";
        while (takebackStr == "")
        {
            String str = bleManager.getTakeback();
            if (str != "" && str != String('\0'))
                takebackStr = str;
            delay(100);
        }

        /* Parse: "capturedPieces,newFen,side" */
        int firstComma = takebackStr.indexOf(',');
        if (firstComma != -1)
        {
            int secondComma = takebackStr.indexOf(',', firstComma + 1);
            if (secondComma != -1)
            {
                String capturedPieces = takebackStr.substring(0, firstComma);
                String newFen         = takebackStr.substring(firstComma + 1, secondComma);
                String side           = takebackStr.substring(secondComma + 1);

                /* Remove un-captured pieces from the graveyard matrix. */
                int spaceIndex = capturedPieces.indexOf(' ');
                if (spaceIndex != -1)
                {
                    /* Two captured pieces (e.g. a capture move was taken back). */
                    String firstPiece  = capturedPieces.substring(0, spaceIndex);
                    String secondPiece = capturedPieces.substring(spaceIndex + 1);
                    eliminateFromGrave(firstPiece.charAt(0));
                    eliminateFromGrave(secondPiece.charAt(0));
                }
                else if (capturedPieces.length() > 0)
                {
                    /* One captured piece. */
                    eliminateFromGrave(capturedPieces.charAt(0));
                }

                Serial.print("New FEN: ");
                Serial.println(newFen);
                Serial.print("Side: ");
                Serial.println(side);

                updateMatrixFromFen(newFen);
                state.currentSide = (side == "1") ? GameState::Side::BOARD
                                                   : GameState::Side::BLE;
            }
        }
        else
        {
            Serial.println("Error: Invalid takeback format - missing commas");
        }

        /* Physically rearrange pieces to the new position and verify sensors. */
        reorderChessboardPlus(3, state.matrix, originalMatrix);
        compareMatrixVsSensorsPlus(0, state.matrix);
    }

    /**
     * @brief Check whether the game should continue or be interrupted.
     *
     * Handles two special modes that can be inserted into the game flow:
     *   mode 7  → perform mechanical re-calibration (homing) mid-game
     *   mode 9  → process a takeback request
     *
     * @return true  if the game should continue (mode is still 2 or was
     *               temporarily 7/9 and has been handled).
     * @return false if the mode changed to something else or BLE disconnected.
     */
    bool checkGameStatus()
    {
        int mode = bleManager.getModeChess();
        if (mode == 7)
        {
            /* Re-calibrate mid-game (e.g. after a stall was detected). */
            mechanicalCalibration(bleManager.getCalibType());
            bleManager.setState("Playing");
            BleChess.setMode(2);
            return true;
        }
        if (mode == 9)
        {
            /* Takeback: rewind the board one move. */
            bleManager.setState("Setting Up");
            handleTakeback();
            bleManager.setState("Playing");
            BleChess.setMode(2);
            return true;
        }
        if (mode != 2 || globalConnect == "0")
        {
            state.isGameOver = true;
            return false;
        }
        return true;
    }

public:
    /**
     * @brief Construct a ChessGameManager for @p ble.
     * @param ble  Reference to the global Bluetooth instance (BleChess).
     */
    ChessGameManager(Bluetooth ble) : bleManager(ble)
    {
        memset(&state, 0, sizeof(state));
    }

    /**
     * @brief Run one complete game session.
     *
     * Blocks until the game ends (mode change, disconnect, or game-over signal).
     * After the game loop exits, moves pieces back to the home position and
     * sets the mode to 3 (pause).
     */
    void run()
    {
        /* Wait for the app to tell us who moves first. */
        String currentSideInfo = BleChess.getPlayInfo();
        while (currentSideInfo == "")
        {
            currentSideInfo = BleChess.getPlayInfo();
            delay(100);
        }
        state.currentSide = (currentSideInfo == "1")
                                ? GameState::Side::BOARD
                                : GameState::Side::BLE;
        Serial.println("currentSideInfo: " + currentSideInfo);
        Serial.printf("currentSide: %d\n", (int)state.currentSide);

        if (!initializeGame())
            return;

        while (!state.isGameOver)
            if (!handleMove())
                break;

        /* Game over: pause mode and reset board to home position. */
        bleManager.setMode(3);
        reorderChessboardPlus(0, nullptr, state.matrix);
    }
};

// =============================================================================
// playMode – entry point for mode 2
// =============================================================================

/**
 * @brief Interactive two-player chess mode entry point.
 *
 * Creates a ChessGameManager on the stack and runs one complete game session.
 * Returns when the session ends (mode change, disconnect, or game over).
 */
void playMode()
{
    ChessGameManager gameManager(BleChess);
    gameManager.run();
}
