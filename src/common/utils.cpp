/*
 * utils.cpp
 * Common utility functions shared across subsystems.
 *
 * soundHandler() plays audio cues via CuteBuzzerSounds.
 * Sound mode meanings:
 *   0  – startup/boot chime (S_CONNECTION)
 *   1  – alert: 6× short C7 beeps
 *   2  – alert to change special modes: 1× short C7 beep
 *   3  – testing sound: fart sequence
 *   4  – incorrect movement (volume-dependent)
 *   5  – correct movement (volume-dependent)
 *   6  – chessboard/sensor mismatch warning
 *   7  – factory reset sound
 *   8  – low battery (S_DISCONNECTION)
 *   9  – battery OK (PIRATES theme)
 *   10 – check warning (double Gb3)
 *   11 – checkmate (long Gb3)
 */

#include "scultpureMode.h"
#include <Arduino.h>
#include "config.h"
#include <CuteBuzzerSounds.h>

void soundHandler(int soundMode)
{
    int _sound = BleChess.getSoundLevel();

    switch (soundMode)
    {
    case 0: // startup/boot chime
        cute.play(S_CONNECTION);
        break;
    case 1: // alert: repeated short beeps
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        break;
    case 2: // alert to change special modes
        cute._tone(NOTE_C7, 50, 30);
        break;
    case 3: // testing sound
        cute.play(S_FART1);
        cute.play(S_FART2);
        cute.play(S_FART3);
        break;
    case 4: // incorrect movement
        if (_sound == 3)
        {
            cute._tone(NOTE_C7, 200, 10);
            delay(100);
            cute._tone(NOTE_C7, 200, 10);
        }
        else if (_sound == 2)
        {
            cute._tone(NOTE_A2, 50, 30);
            delay(50);
            cute._tone(NOTE_A2, 50, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
            delay(50);
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 5: // correct movement
        if (_sound == 3)
        {
            cute._tone(NOTE_C7, 200, 10);
        }
        else if (_sound == 2)
        {
            cute._tone(NOTE_C4, 50, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 6: // chessboard/sensor mismatch
        if (_sound == 2)
        {
            cute.play(S_CONNECTION);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 7: // factory reset
        if (_sound == 2)
        {
            cute.play(S_MODE3);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 8: // low battery / no battery
        cute.play(S_DISCONNECTION);
        break;
    case 9: // battery OK
        cute.play(PIRATES);
        break;
    case 10: // check warning
        if (_sound == 2)
        {
            delay(1000);
            cute._tone(NOTE_Gb3, 1000, 30);
            delay(100);
            cute._tone(NOTE_Gb3, 1000, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 1000, 30);
            delay(50);
            cute._tone(NOTE_A0, 1000, 30);
        }
        break;
    case 11: // checkmate
        if (_sound == 2)
        {
            cute._tone(NOTE_Gb3, 2000, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A1, 2000, 30);
        }
        break;
    default:
        Serial.print("Invalid Sound -----Bug detected 0x01 ");
        break;
    }
}
