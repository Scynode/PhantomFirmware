/*
 * electromagnets.cpp
 * Electromagnet activation and deactivation for piece manipulation.
 *
 * ELECTROMAGNET LAYOUT:
 * ──────────────────────
 * Four electromagnets are mounted under the board, each covering one quadrant:
 *   1 → upper-left  quadrant  (magnet1 pin, GPIO 19)
 *   2 → lower-left  quadrant  (magnet2 pin, GPIO 22)
 *   3 → upper-right quadrant  (magnet3 pin, GPIO 21)
 *   4 → lower-right quadrant  (magnet4 pin, GPIO 23)
 *
 * Each magnet is driven by a low-side MOSFET.  Setting the GPIO LOW energises
 * the coil; setting it HIGH cuts power (active LOW logic).
 *
 * POWER CONTROL:
 * ───────────────
 * The current implementation uses simple digital on/off control (GPIO LOW = full
 * power, GPIO HIGH = off).  A legacy PWM-based implementation is preserved in
 * comments above the active code; the PWM version used ledcSetup / ledcAttachPin /
 * ledcWrite to modulate duty cycle for reduced-power holding current.
 *
 * SAFETY TIMER:
 * ──────────────
 * activateElectromagnetV2() starts a hardware timer (timer2H) on the first call.
 * The timer fires every 10 seconds.  The timeEnabled() ISR (in motion.cpp)
 * checks whether the magnet has been on for > 60 seconds and cuts power if so,
 * preventing coil overheating.
 *
 * deactivateAllMagnets() stops and destroys timer2H.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "mech_state.h"

// ─────────────────────────────────────────────────────────────────────────────
// activateElectromagnetV2()
// Turns on the specified electromagnet at full power (digital LOW).
//
// optionElectro: 1-4 selects which magnet to activate (see layout above).
// power:         parameter is accepted for API compatibility with the legacy PWM
//                version but is not used in the current digital implementation.
//
// All other magnets are turned off before the selected one is activated to
// ensure only one magnet is energised at a time.
//
// The safety timer is started on the first call after deactivation.
// ─────────────────────────────────────────────────────────────────────────────
void activateElectromagnetV2(int optionElectro, int power)
{
    // Serial.print("Activating electromagnet: ");
    // Serial.println(optionElectro);

    // Update the activation time of the electromagnets
    timerMagnets = millis();

    // Initialize the timer if not already initialized
    if (timer2H == NULL)
    {
        timer2H = timerBegin(3, 80, true); // Prescaler 80 for 1us resolution with an 80MHz clock
        timerAttachInterrupt(timer2H, &timeEnabled, true);
        timerAlarmWrite(timer2H, 10000000, true); // 10 seconds
        timerAlarmEnable(timer2H);
        // Serial.println("==============================================Timer enabled");
    }

    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Turn off all electromagnets initially (active LOW: HIGH = off)
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(magnetPins[i], HIGH); // Turn off the electromagnet
    }

    // Verify that the electromagnet option is valid
    if (optionElectro >= 1 && optionElectro <= 4)
    {
        // Activate the selected electromagnet at full power (active LOW: LOW = on)
        digitalWrite(magnetPins[optionElectro - 1], LOW);
    }
    // Serial.println("Electromagnet activated");
}

// ─────────────────────────────────────────────────────────────────────────────
// deactivateAllMagnets()
// Turns off all four electromagnets and stops the safety timer.
//
// Should be called at the end of every piece move and in all error/safety paths.
// ─────────────────────────────────────────────────────────────────────────────
void deactivateAllMagnets()
{
    // Serial.println("Deactivating all electromagnets");
    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Turn off all electromagnets (active LOW: HIGH = off)
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(magnetPins[i], HIGH); // Set the pin high (off)
    }

    // Disable and stop the timer
    if (timer2H != NULL)
    {
        timerAlarmDisable(timer2H);
        timerDetachInterrupt(timer2H);
        timerEnd(timer2H);
        timer2H = NULL; // Ensure the pointer is NULL after freeing the timer
                        //  Serial.println("==============================================Timer disabled");
    }
    // Serial.println("All electromagnets deactivated")
}
