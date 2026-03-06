# PhantomFirmware Refactor Notes

## Overview

The original firmware consisted of three large monolithic source files:

| Original file | Lines | Replaced by |
|---|---|---|
| `src/main.cpp` | 1920 | `src/app/` (4 files) |
| `src/BLE.cpp` | 1919 | `src/ble/` (9 files) |
| `src/scultpureMode.cpp` | 4520 | `src/mech/` + `src/sensors/` + `src/chess/` + `src/common/` (16 files) |

**No logic was changed.** All function bodies, variable names, magic numbers, UUIDs, pin assignments and algorithms are identical to the originals.

---

## Module structure

```
src/
├── app/                      ← Arduino entry points and game-mode dispatch
│   ├── app_globals.h         extern declarations for all globals in setup.cpp
│   ├── setup.cpp             globals, bleTask(), setup(), loop(), utilities
│   ├── modes.cpp             errorMessage(), testMode(), pauseMode(), sculptureMode()
│   └── play_mode.cpp         ChessGameManager, playMode(), detectChangePlus(),
│                             automaticMechanicMovement(), soundEndGame(), stringToMatrix()
│
├── ble/                      ← NimBLE stack, GATT service, OTA
│   ├── ble_state.h           extern declarations for all BLE globals
│   ├── ble_state.cpp         single definition point for all 54 BLE globals
│   ├── ble_callbacks.h       class declarations for all 24 callback classes
│   ├── ble_callbacks.cpp     onWrite/onRead for all 23 non-OTA characteristics
│   ├── ble_ota.cpp           FilesCallbacks_receiveOta (OTA update handler)
│   ├── ble_server.cpp        Bluetooth::Bluetooth(), Bluetooth::init(), bleServerCallback
│   ├── ble_methods.cpp       all Bluetooth:: getter/setter static methods
│   ├── ble_battery.cpp       Bluetooth::batterySim(), Bluetooth::batteryCheck()
│   └── ble_fragment.cpp      Fragment class + resetPlaylist()
│
├── sensors/                  ← Hall-effect sensor multiplexer chain
│   ├── sensor_state.h        extern declarations for sensor globals
│   ├── sensor_state.cpp      definitions: rows, cols, muxesOut, dirMux, sensorMatrixSc, sensorUpdate
│   ├── sensor_io.cpp         sensorsDir(), updateSensors(), readRawSensors(), detectChessBoard()
│   └── sensor_calib.cpp      sensorOffsetCalib(), sensorsCalibration()
│
├── mech/                     ← Stepper drivers, motion, trajectory, electromagnets
│   ├── mech_state.h          extern declarations for mechanical globals
│   ├── mech_state.cpp        TMC2209/AccelStepper objects, timer handles, speed vars
│   ├── drivers.cpp           configDrivers(), configAndCurrentPosManager(), testDrivers()
│   ├── motion.cpp            accelRampV3(), rawMovement(), rawMovementStallGuard(), ISRs
│   ├── trajectory.cpp        generateTrajectory(), Bézier curves, interpolatePoints()
│   ├── electromagnets.cpp    activateElectromagnetV2(), deactivateAllMagnets()
│   └── calibration.cpp       mechanicalCalibration()
│
├── chess/                    ← Board logic, PGN decode, game replay
│   ├── chess_state.h         extern declarations for chess globals
│   ├── chess_state.cpp       timeSculpture, movChess, targetMatrix, games[] array
│   ├── matrix_util.cpp       printGenericMatrix(), initMatrixPlus()
│   ├── movement.cpp          simplifiedMovement(), decodeMovement(), reorderChessboardPlus(),
│   │                         centerInitialPiecesSc(), compareMatrixVsSensorsPlus(),
│   │                         findNearestEmptyPosition(), findNearestPiecePosition()
│   └── pgn.cpp               sculptureMain(), readFromFileSc(), decodeChessMove()
│
└── common/                   ← Shared utilities
    └── utils.cpp             soundHandler()
```

---

## Key invariants

### Global variable ownership

Each global is **defined in exactly one `.cpp` file** and accessed everywhere else via an `extern` declaration (either in the matching `.h` or in `config.h`/`BLE.h`).

| Variable | Defined in |
|---|---|
| `BleChess`, `preferences`, `sensorMatrix`, `mode`, `testFlag`, `previousChessState`, … | `src/app/setup.cpp` |
| All `generalCharacteristic_*` pointers, `globalConnect`, `versionUpdate`, … | `src/ble/ble_state.cpp` |
| `driver`, `driver2`, `stepper1`, `stepper2`, timer handles, … | `src/mech/mech_state.cpp` |
| `muxesOut`, `dirMux`, `sensorMatrixSc`, `sensorUpdate`, `rows`, `cols` | `src/sensors/sensor_state.cpp` |
| `timeSculpture`, `movChess`, `targetMatrix`, `reorderChessboard`, `games[]` | `src/chess/chess_state.cpp` |

### Dual-core architecture

- **Core 0** (`bleTask`): NimBLE stack + `batterySim()` + `updateSensors()`
- **Core 1** (Arduino `loop()`): all motion, chess logic, and mode dispatch

### Board coordinate system

- Physical centre = (0 mm, 0 mm)
- Square pitch = 50 mm
- Full board range: X = −175 to +175 mm, Y = −175 to +175 mm
- Matrix indices: `matrix[col][row]`, 1-indexed for the 8×8 playing field; row/col 0 and 9 are the graveyard border

### 10×10 sensor matrix layout

```
col:  0    1    2    3    4    5    6    7    8    9
row 0: [grave]  a8   b8   c8   d8   e8   f8   g8   h8  [grave]
row 1: a7   ──   ──   ──   ──   ──   ──   ──   ──   h7
...
row 8: a2   ──   ──   ──   ──   ──   ──   ──   ──   h2
row 9: [grave]  a1   b1   c1   d1   e1   f1   g1   h1  [grave]
```
(Outer ring = graveyard for captured pieces; inner 8×8 = active chess board)

### OTA update flow

1. App writes first packet: bytes 0–3 = total firmware size (little-endian uint32).
2. App writes subsequent packets: raw firmware binary chunks.
3. On final chunk: `esp_ota_ops` `end()` is called, `updatingStatus = -1` saved to NVS, then `esp_restart()`.
4. On next boot `setup()` detects `updatingStatus >= 0` → broadcasts "Updating" indefinitely (recovery mode).

### BLE characteristic map (selected)

| UUID constant | Direction | Purpose |
|---|---|---|
| `GENERAL_UUID_STATUS_BOARD` | Board→App | Detected physical move string |
| `GENERAL_UUID_CHECK_MOVE` | App→Board | "1"=legal, "2"=illegal |
| `GENERAL_UUID_RECEIVE_MOVEMENT` | App→Board | Engine move command |
| `UUID_MECHANISM_SPEED` | App→Board | "1"=50 mm/s, "2"=100 mm/s, "3"=200 mm/s |
| `UUID_PAUSE` | App→Board | 1=paused, 0=running |
| `UUID_REPEAT` | App→Board | 0=stop, 1=repeat game, 2=repeat playlist |
| `UUID_BOARD_ROTATION` | App→Board | 0/90/180/270 degrees; triggers reboot |
| `UUID_CALIB_TYPE` | App→Board | 0/1/2 calibration method; triggers reboot |
| `FILE_UUID_RECEIVE_OTA` | App→Board | OTA firmware chunks |
| `UUID_TAKEBACK` | App→Board | "capturedPieces,newFen,side" |
| `UUID_MATRIX_INIT_GAME` | App→Board | 100-char flat board matrix |
