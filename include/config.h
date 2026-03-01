#include <preferences.h>
#include <BLE.h>

extern Bluetooth BleChess;
extern Preferences preferences;
extern bool testModeSculpture;
extern unsigned long timeMoving;

// #define MODELEFRA // Efra
#define MODELFINAL // Lalo

//------------------------------------------------------------------------------
#define COMPARE_IMPRIME 1 // 001  0 = no print, 1 = print.
#define COMPARE_CENTRA 2  // 010  0 = no specific centering, 1 = specific centering.
#define COMPARE_HOME 4    // 100  0 = no homing, 1 = do homing.

#define magnet1 19 // 19
#define magnet2 22 // 23
#define magnet3 21 // 21
#define magnet4 23 // 22

#define ENABLE_PIN 18
#define BUZZER_PIN 25

//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (32)
#define MOTOR_0_STEP_PIN (33)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (17)
#define MOTOR_1_STEP_PIN (5)

#define CALIB_CURRENT 1000
#define MIN_RMS_CURRENT 800
#define HOLD_CURRENT 400
#define R_SENSE 0.11f
#define TOFFTEST 1

#define SERIAL_PORT2 Serial1 // TMC2208/TMC2224 HardwareSerial port

#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2
#define DRIVER_ADDRESS2 0b11 // TMC2209 Driver address according to MS1 and MS2

// Pins below ares used to select the 8bits muxes
#define mux8_0 12 // 12 4
#define mux8_1 14 // 14 13
#define mux8_2 16 // 4  2
// Pins for the 4 selectors of the 16bits muxes.
#define mux16_0 2  // 13  25
#define mux16_1 15 // 2  14
#define mux16_2 13 // 25  12
// Data IN pins from 16 bits muxes
#define mux16Out_1 35 // 35  39
#define mux16Out_2 34 // 34  34
#define mux16Out_3 39 // 39  35
#define mux16Out_4 36 // 36  36

//Battery Mux
#define batS0 16
#define batS1 2
#define batS2 15