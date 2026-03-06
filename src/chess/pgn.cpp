/*
 * pgn.cpp
 * PGN game reading, move decoding, and sculpture main loop.
 *
 * SPIFFS storage: games stored as files (indices 200-299 in NVS/Preferences),
 * fall back to embedded games[] array (indices 0-199) if not found in storage.
 *
 * sculptureMain() orchestration: load game → decode moves → play moves →
 * center pieces → repeat. Supports pause/resume/single-step/rewind via BLE.
 *
 * The timeToMove delay (BleChess.getTimeToMove() seconds) is waited between
 * each move when not paused. In paused mode, getSingleMove() returns:
 *   1 = advance one move, 0 = go back one move, -1 = do nothing.
 */

#include "scultpureMode.h"
#include <Arduino.h>
#include "config.h"
#include "chess_state.h"
#include "BLE.h"
#include <vector>

/*
 * sculptureMain()
 * ─────────────────────────────────────────────────────────────────────────────
 * Main loop for autonomous sculpture/playback mode (BLE mode 1).
 *
 * State machine:
 *   1. Init board matrix, center pieces, verify sensors.
 *   2. Outer loop: select next game from playlist, decode all moves.
 *   3. Inner loop: play each move with inter-move delay (timeToMove).
 *      - Pause:  wait for getSingleMove() to return non-(-1).
 *      - Play:   advance automatically, break when game finished or game changed.
 *      - Rewind: reverse the last encoded move pair.
 *   4. After game ends: reorderChessboardPlus, optional recalibration, advance playlist.
 *   5. Repeat until getModeChess() != 1.
 */
void sculptureMain()
{
    int cuentapatternsToHome = 0;
    char sculpturePlusMatrix[10][10];

    //── Turn-on routine ───────────────────────────────────────────────────────
    initMatrixPlus(sculpturePlusMatrix);
    BleChess.setMode(1);
    if (testModeSculpture == false)
    {
        Serial.printf("\n \n=======================SCULPTURE MAIN======================= \n \n");
        centerInitialPiecesSc(0, sculpturePlusMatrix);
        compareMatrixVsSensorsPlus(COMPARE_IMPRIME | COMPARE_CENTRA, sculpturePlusMatrix);
        BleChess.sendMatrixToApp("CLEAN: Match.", sensorMatrixSc, sculpturePlusMatrix);
    }
    else
    {
        Serial.printf("\n \n======================SCULPTURE MAIN TEST MODE======================= \n \n");
        timeSculpture = millis();
    }

    do // Outer loop: play next game in playlist
    {
        int moveCount = 0;

        // Wait if no games are queued in the playlist.
        if (BleChess.gameToPlay(0) == -1)
        {
            while (BleChess.gameToPlay(0) == -1 && BleChess.getModeChess() == 1)
            {
                if (BleChess.getPauseInfo() == 0)
                {
                    BleChess.setPauseInfo(1);
                    Serial.println("No Games in Now Playing");
                    BleChess.setState("No Games in Now Playing");
                    BleChess.sendTestModeError("No Games to play, swipe left in anygame in the library to add games to now playing.");
                }
            }
        }

        int retry = 0;
        int gameToPlay = BleChess.gameToPlay(0);
        String fullMoves = readFromFileSc(gameToPlay);
        while (fullMoves == "EMPTY" && retry < 3)
        {
            fullMoves = readFromFileSc(gameToPlay);
            retry++;
            delay(1000);
        }
        if (fullMoves == "EMPTY")
        {
            Serial.println("Error reading file");
            BleChess.sendTestModeError("ERROR SC-MAIN-280\nSend error code to Phantom Team please.");
            BleChess.setPauseInfo(1);
            esp_restart();
        }

        int movementsinGame = (fullMoves.length());

        int previousPause = BleChess.getPauseInfo();
        BleChess.setState(previousPause == 1 ? "Paused" : "Running");

        do // Inner loop: play each move
        {
            int moveType = 0; // -1 = no-op, 0 = move back, 1 = move next
            int pauseInfo = BleChess.getPauseInfo();

            // Send pause state update only when it changes.
            if (previousPause != pauseInfo)
            {
                BleChess.setState(pauseInfo == 1 ? "Paused" : "Running");
                previousPause = pauseInfo;
            }

            // Break if the game or mode changed externally.
            if (gameToPlay != BleChess.gameToPlay(0) || BleChess.getModeChess() != 1)
            {
                break;
            }

            if (pauseInfo != 1) // ── Running: advance automatically ──────────
            {
                moveType = 1;
                if (moveCount != 0) // First move executes immediately; subsequent moves wait.
                {
                    int timeToMove = BleChess.getTimeToMove() * 1000;
                    unsigned long timeToMoveStart = millis();
                    bool getOutofWhile = false;
                    while (millis() - timeToMoveStart < timeToMove)
                    {
                        if (gameToPlay != BleChess.gameToPlay(0) || BleChess.getModeChess() != 1)
                        {
                            getOutofWhile = true;
                            break;
                        }
                    }
                    if (getOutofWhile)
                    {
                        break;
                    }
                }
            }
            else // ── Paused: single-step forward or backward ──────────────
            {
                moveType = BleChess.getSingleMove();

                if (moveCount >= movementsinGame)
                {
                    if (moveType == 1)
                    {
                        moveType = -1;
                        BleChess.sendTestModeError("No more moves next, game finished, move back or change game.");
                    }
                }

                if (moveCount == 0)
                {
                    if (moveType == 0)
                    {
                        moveType = -1;
                        BleChess.sendTestModeError("No more previous moves, game start, move next, press play or change game.");
                    }
                }
            }

            if (moveType == 1) // Advance: execute next half-move (or full move)
            {
                simplifiedMovement(fullMoves[moveCount] - '0', fullMoves[moveCount + 1] - '0',
                                   fullMoves[moveCount + 2] - '0', fullMoves[moveCount + 3] - '0',
                                   sculpturePlusMatrix);
                if (fullMoves[moveCount + 4] == '/')
                {
                    moveCount += 5;
                }
                else
                {
                    simplifiedMovement(fullMoves[moveCount + 4] - '0', fullMoves[moveCount + 5] - '0',
                                       fullMoves[moveCount + 6] - '0', fullMoves[moveCount + 7] - '0',
                                       sculpturePlusMatrix);
                    moveCount += 9;
                }
            }
            else if (moveType == 0) // Rewind: reverse last half-move (or full move)
            {
                simplifiedMovement(fullMoves[moveCount - 3] - '0', fullMoves[moveCount - 2] - '0',
                                   fullMoves[moveCount - 5] - '0', fullMoves[moveCount - 4] - '0',
                                   sculpturePlusMatrix);
                if (fullMoves[moveCount - 6] == '/' || moveCount - 6 <= 0)
                {
                    moveCount -= 5;
                }
                else
                {
                    simplifiedMovement(fullMoves[moveCount - 7] - '0', fullMoves[moveCount - 6] - '0',
                                       fullMoves[moveCount - 9] - '0', fullMoves[moveCount - 8] - '0',
                                       sculpturePlusMatrix);
                    moveCount -= 9;
                }
            }

            // When all moves are exhausted: auto-advance or pause.
            if (moveCount >= movementsinGame)
            {
                if (BleChess.getAutoPlayback())
                {
                    break;
                }
                else
                {
                    BleChess.setPauseInfo(1);
                }
            }

        } while (true);

        fullMoves = "";
        Serial.println("-----------------------------------PREPARANDO TABLERO");
        BleChess.setState("Setting Up");

        // Reset the board for the next game.
        reorderChessboardPlus(0, nullptr, sculpturePlusMatrix);
        int patternsToHomeInt = BleChess.getpatternsToHome();

        // Count completed games (>85 % of moves played) for recalibration scheduling.
        if (moveCount >= (int)(movementsinGame * 0.85))
        {
            cuentapatternsToHome++;
        }

        // Periodically recalibrate the mechanism.
        if (cuentapatternsToHome >= patternsToHomeInt)
        {
            mechanicalCalibration(BleChess.getCalibType());
            centerInitialPiecesSc(0, sculpturePlusMatrix);
            cuentapatternsToHome = 0;
        }

        // Advance the playlist only if the game ended naturally (not by user change).
        if (gameToPlay == BleChess.gameToPlay(0))
        {
            Serial.println("FIN NANTURAL DEL JUEGO");
            BleChess.gameToPlay(1);
        }

        BleChess.setPauseInfo(0);
        Serial.println("-----------------------------------PREPARANDO TABLERO TERMINA");
    } while (BleChess.getModeChess() == 1);
}

/*
 * readFromFileSc()
 * ─────────────────────────────────────────────────────────────────────────────
 * Reads a chess game by index and decodes it into a compact move string that
 * sculptureMain() can replay directly.
 *
 * Index < 200  : reads from the embedded games[] array in flash.
 * Index 200-299: reads from NVS Preferences namespace "PL_pgn".
 *
 * The function walks the PGN token stream, calling decodeChessMove() and
 * decodeMovement() for each token to produce a String of the form:
 *   "[r0][c0][r1][c1]/[r0][c0][r1][c1]/..."
 * where each group of 4 digits encodes one physical movement.
 *
 * Parameters:
 *   gameToPlay – index into games[] or NVS key
 *
 * Returns:
 *   Decoded move string, or "EMPTY" on error.
 */
String readFromFileSc(int gameToPlay)
{
    int index_mov = 0;
    int indexA = 0;
    int gameLenght = 0;
    char movChessInd[7];
    const char *game = "";
    char gameBuffer[2048];

    if (gameToPlay < 200)
    {
        gameLenght = strlen(games[gameToPlay]);
        game = games[gameToPlay];
    }
    else if (gameToPlay > 200 && gameToPlay < 300)
    {
        std::string gameToPlayAux = std::to_string(gameToPlay);
        const char *gameToPlayPref = gameToPlayAux.c_str();
        Serial.print("Game to play pref: ");
        Serial.println(gameToPlayPref);
        preferences.begin("PL_pgn");
        String gameString = preferences.getString(gameToPlayPref, "EMPTY");
        if (gameString == "EMPTY")
        {
            Serial.println("PLAYLIST READ EMPTY");
            return "EMPTY";
        }
        preferences.end();
        strncpy(gameBuffer, gameString.c_str(), sizeof(gameBuffer) - 1);
        gameBuffer[sizeof(gameBuffer) - 1] = '\0';
        game = gameBuffer;
        Serial.print("Game to play: ");
        Serial.println(game);
        gameLenght = strlen(game);
        Serial.print("Game length: ");
        Serial.println(gameLenght);
    }
    else
    {
        Serial.println("INDEX OUT OF BOUNDS");
    }

    char matrixToSimulateGame[10][10];
    String fullMoves = "";

    initMatrixPlus(matrixToSimulateGame);
    indexA = 0;

    while (indexA < gameLenght)
    {
        if (game[indexA] == ' ')
        {
            for (int i = 0; i < 7; i++)
            {
                movChessInd[i] = 'v';
            }

            indexA++;
            int indexAInit = indexA;
            while (game[indexA] != ' ' && game[indexA] != '.' && indexA < gameLenght)
            {
                movChessInd[indexA - indexAInit] = game[indexA];
                indexA++;
            }

            if (game[indexA] == ' ')
            {
                String auxDecode = decodeChessMove(movChessInd, index_mov);
                String localMove = decodeMovement(auxDecode[2], auxDecode[3], auxDecode[4], auxDecode[5],
                                                  auxDecode[1], auxDecode[0], matrixToSimulateGame, false);

                // Simulate the move on the local matrix for correct graveyard placement.
                matrixToSimulateGame[localMove[2] - '0'][localMove[3] - '0'] = matrixToSimulateGame[localMove[0] - '0'][localMove[1] - '0'];
                matrixToSimulateGame[localMove[0] - '0'][localMove[1] - '0'] = '.';

                if (localMove[4] != '/')
                {
                    matrixToSimulateGame[localMove[6] - '0'][localMove[7] - '0'] = matrixToSimulateGame[localMove[4] - '0'][localMove[5] - '0'];
                    matrixToSimulateGame[localMove[4] - '0'][localMove[5] - '0'] = '.';
                }

                fullMoves = fullMoves + localMove;
                index_mov++;
            }
        }
        else
        {
            indexA++;
        }
    }

    return fullMoves;
}

/*
 * decodeChessMove()
 * ─────────────────────────────────────────────────────────────────────────────
 * Converts a 7-character raw PGN token (movChess[7]) into the intermediate
 * format consumed by decodeMovement():
 *   [color][action][x_ini][y_ini][x_fin][y_fin]
 *
 * color  : 'W' (even moveCount) or 'B' (odd moveCount)
 * action : '0'=simple, '1'=capture, '2'=short castle, '3'=long castle
 * x/y    : board coordinate characters directly from the token
 *
 * Parameters:
 *   movChess  – 7-char token array (padded with 'v')
 *   moveCount – half-move index (0-based); even = white, odd = black
 *
 * Returns:
 *   6-character String "[color][action][x_ini][y_ini][x_fin][y_fin]",
 *   or "[color][action]    " for castling moves.
 */
String decodeChessMove(char movChess[7], int moveCount)
{
    int int_accion = -1;
    String strChessMove = "";

    strChessMove = (moveCount % 2 == 0) ? "W" : "B";

    // Detect action type by scanning for special characters in the token.
    char v_accion[5] = {'-', ' ', 'x', '+', 'O'};
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            if (movChess[i] == v_accion[j])
            {
                if (v_accion[j] == 'x') int_accion = 0;     // capture
                if (v_accion[j] == 'O') int_accion++;        // each 'O' increments (1=short, 2=long)
            }
        }
    }

    // Map int_accion to action character.
    strChessMove += int_accion == -1 ? 0 : int_accion == 0 ? 1 : int_accion + 1;

    if (int_accion < 1) // Simple move or capture: extract coordinate characters.
    {
        // Mask out all non-coordinate characters.
        char mask[12] = {'-', ' ', 'x', '+', 'O', 'K', 'Q', 'R', 'B', 'N', 'v'};
        int posIndex[4];
        int k = 0;
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (movChess[i] == mask[j]) break;
                if (j == 11)
                {
                    posIndex[k] = i;
                    k++;
                }
            }
        }
        strChessMove += movChess[posIndex[0]];
        strChessMove += movChess[posIndex[1]];
        strChessMove += movChess[posIndex[2]];
        strChessMove += movChess[posIndex[3]];
        return strChessMove;
    }
    else // Castling: coordinates not needed (hardcoded in decodeMovement).
    {
        strChessMove += "    ";
        return strChessMove;
    }
}
