# Elegoo Smart Robot Car V4.0

Firmware for the [Elegoo Smart Robot Car Kit V4.0](https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial), running on an Arduino UNO.

## Project Structure

```
SmartRobotCar/
│
├── SmartRobotCar.ino              # Entry point: setup() and loop(); calls all Application_FunctionSet methods
│
├── ApplicationFunctionSet.h       # ApplicationFunctionSet class declaration: all high-level modes and CMD handlers
├── ApplicationFunctionSet.cpp     # High-level logic: driving modes (tracking, obstacle, follow, rocker),
│                                  # serial JSON command parser, IR remote handler, yaw-based straight-line control
│
├── DeviceDriverSet.h              # Hardware driver class declarations and pin definitions
├── DeviceDriverSet.cpp            # Hardware drivers: RGB LED (FastLED), push button, IR receiver,
│                                  # line tracking sensors (ITR20001), voltage monitor, motors (TB6612),
│                                  # ultrasonic (HC-SR04), servos
│
├── util.h                         # Shared inline utility: delay_wdt() — watchdog-safe blocking delay
│
├── MPU6050_getdata.h              # MPU6050_getdata class declaration
├── MPU6050_getdata.cpp            # MPU6050 gyroscope integration: init, offset calibration, yaw angle computation
│
├── MPU6050.h / MPU6050.cpp        # Low-level MPU6050 I2C register driver (Elegoo-provided)
├── I2Cdev.h  / I2Cdev.cpp         # I2C communication abstraction used by the MPU6050 driver
│
├── IRremote.h / IRremote.cpp      # IR receive/decode library (bundled, NEC protocol)
├── IRremoteInt.h                  # Internal definitions for IRremote
│
├── ArduinoJson-v6.11.1.h          # Single-header ArduinoJson v6 — used to parse serial JSON commands
│
├── addLibrary/                    # Zip archives of libraries to install via arduino-cli
│   ├── FastLED-master.zip         # RGB LED control
│   ├── IRremote.zip               # IR remote (not needed; bundled IRremote.h/.cpp already in sketch folder)
│   ├── NewPing.zip                # Ultrasonic helper (not used; HC-SR04 is driven manually)
│   └── pitches.zip                # Musical note frequency constants (not used in current code)
│
└── .vscode/
    └── arduino.json               # VSCode Arduino extension config: board, port, sketch path
```

## Hardware

| Component       | Part                   |
|-----------------|------------------------|
| Main controller | Arduino UNO            |
| Motor driver    | TB6612FNG              |
| Gyroscope       | GY-521 (MPU6050)       |
| Ultrasonic      | HC-SR04                |
| Line tracking   | ITR20001 (×3 IR)       |
| RGB LED         | NeoPixel (×1)          |
| IR receiver     | 38kHz receiver         |
| Servos          | Pin 10 (z), Pin 11 (y) |

## Features

| Mode               | Description                                                           |
|--------------------|-----------------------------------------------------------------------|
| Standby            | Motors stopped; LED breathes violet                                   |
| Line Tracking      | Follows a black line using three IR sensors; LED green                |
| Obstacle Avoidance | Scans with servo + ultrasonic and steers around obstacles; LED yellow |
| Follow             | Follows an object within 20 cm detected by ultrasonic; LED blue       |
| Rocker (App/IR)    | Remote control via Bluetooth app or IR remote; LED violet             |
| Programming (CMD)  | Accepts JSON commands over serial from the app; see protocol below    |

## Operating Modes — Physical Button

Press the onboard button (pin 2) to cycle through modes:

| Presses | Mode               |
|---------|--------------------|
| 1       | Line Tracking      |
| 2       | Obstacle Avoidance |
| 3       | Follow             |
| 4       | Standby            |

## IR Remote Control

Two IR remote variants are supported (code sets A and B in the `IR_CODE_TABLE` in `DeviceDriverSet.cpp`).

| Button | Action                              |
|--------|-------------------------------------|
| ▲      | Move forward                        |
| ▼      | Move backward                       |
| ◄      | Turn left                           |
| ►      | Turn right                          |
| OK     | Standby                             |
| 1      | Line Tracking mode                  |
| 2      | Obstacle Avoidance mode             |
| 3      | Follow mode                         |
| 4      | Increase line tracking threshold    |
| 5      | Reset line tracking threshold       |
| 6      | Decrease line tracking threshold    |
| 7      | Increase rocker speed               |
| 8      | Reset rocker speed                  |
| 9      | Decrease rocker speed               |

## Serial / App JSON Protocol

Commands are sent as JSON frames terminated with `}`, at **9600 baud**.

| N   | Command                       | Fields                                                        |
|-----|-------------------------------|---------------------------------------------------------------|
| 1   | Motor control (per motor)     | D1: motor (0=both, 1=L, 2=R), D2: speed, D3: dir (1=fwd, 2=bck) |
| 2   | Car direction — timed         | D1: dir, D2: speed, T: ms                                     |
| 3   | Car direction — continuous    | D1: dir, D2: speed                                            |
| 4   | Motor speed (independent L/R) | D1: speed_L, D2: speed_R                                      |
| 5   | Servo control                 | D1: servo (1=z, 2=y, 3=both), D2: angle×10                   |
| 7   | RGB lighting — timed          | D1: position, D2–D4: R/G/B, T: ms                            |
| 8   | RGB lighting — continuous     | D1: position, D2–D4: R/G/B                                   |
| 21  | Query ultrasonic distance     | D1: 1=obstacle bool, 2=distance in cm                        |
| 22  | Query IR tracking sensor      | D1: 0=L, 1=M, 2=R                                            |
| 23  | Query lift-off detection      | —                                                             |
| 100 | Clear all → Standby           | —                                                             |
| 101 | Switch autonomous mode        | D1: 1=Tracking, 2=Avoidance, 3=Follow                        |
| 102 | Rocker control                | D1: dir (1–9), D2: speed                                      |
| 105 | LED brightness                | D1: 1=increase, 2=decrease                                    |
| 106 | Servo step control            | D1: 1–5                                                       |
| 110 | Clear all → Programming mode  | —                                                             |

Direction values for car commands: `1`=Left, `2`=Right, `3`=Forward, `4`=Backward.

## Development Setup

**Requirements:** `arduino-cli` (non-snap), VSCode with [Arduino Community Edition](https://marketplace.visualstudio.com/items?itemName=vscode-arduino-community.vscode-arduino).

### Install arduino-cli

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
mkdir -p ~/.local/bin && mv bin/arduino-cli ~/.local/bin/
# Add ~/.local/bin to PATH in ~/.bashrc if not already present
```

### Install board and libraries

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli config set library.enable_unsafe_install true
arduino-cli lib install --zip-path addLibrary/FastLED-master.zip
arduino-cli lib install --zip-path addLibrary/NewPing.zip
arduino-cli lib install --zip-path addLibrary/pitches.zip
arduino-cli lib install Servo
```

### VSCode settings

Add to your user `settings.json`:
```json
"arduino.useArduinoCli": true,
"arduino.commandPath": "/home/<user>/.local/bin/arduino-cli"
```

`.vscode/arduino.json` is already configured for Arduino UNO on `/dev/ttyUSB0`. Adjust the port if needed (`/dev/ttyACM0` is also common).

### Upload permission

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for the change to take effect
```

### Compile and upload

- **Verify:** `Ctrl+Shift+P` → *Arduino: Verify*
- **Upload:** `Ctrl+Shift+P` → *Arduino: Upload*

## Hardware Diagnostics

Each hardware driver has a built-in diagnostic function that prints results over serial or actuates the hardware. They are compiled out in normal firmware and activated by a single flag.

### Enable

In [DeviceDriverSet.h](DeviceDriverSet.h), line 12, change:
```cpp
#define _Test_DeviceDriverSet 0
```
to:
```cpp
#define _Test_DeviceDriverSet 1
```

The `loop()` in [SmartRobotCar.ino](SmartRobotCar.ino) automatically switches to the test runner when the flag is set — no other file needs editing.

### Run

Upload and open the Serial Monitor (`Ctrl+Shift+P` → *Arduino: Open Serial Monitor*) at **9600 baud**.

By default `RunHardwareTests()` is called every loop iteration. To run each test only once, edit the test loop in `SmartRobotCar.ino`:
```cpp
void loop() {
  wdt_reset();
  static bool ran = false;
  if (!ran) {
    Application_FunctionSet.RunHardwareTests();
    ran = true;
  }
}
```

### What each test does

| Test | Output / behaviour |
|------|--------------------|
| `DeviceDriverSet_RBGLED_Test` | Flashes LED white then red |
| `DeviceDriverSet_Key_Test` | Prints current button press counter to serial |
| `DeviceDriverSet_ITR20001_Test` | Prints raw ADC values for L, M, R IR sensors |
| `DeviceDriverSet_Voltage_Test` | Prints computed battery voltage in volts |
| `DeviceDriverSet_Motor_Test` | Spins both motors forward 1s, pauses, backward 1s, then stops |
| `DeviceDriverSet_ULTRASONIC_Test` | Prints distance in cm to the nearest object |
| `DeviceDriverSet_Servo_Test` | Sweeps servo to 180°, back to 0°, returns to 90° centre |
| `DeviceDriverSet_IRrecv_Test` | Prints raw IR code of any button pressed on the remote |

### Restore normal operation

Set `_Test_DeviceDriverSet` back to `0` and upload.

### Verbose serial output during normal operation

In [ApplicationFunctionSet.cpp](ApplicationFunctionSet.cpp), line 21, setting:
```cpp
#define _Test_print 1
```
enables sensor value logging (line tracking ADC readings, serial frame dump) during normal operation — useful for tuning `TrackingDetection_S` and `TrackingDetection_E` thresholds.

## Changelog

### 2026-04-21 (david)
**Refactoring**
- Removed `_xxx0` suffix from all CMD method names
- Removed unimplemented stub declarations (`CMD_VoiceControl`, `CMD_LEDCustomExpressionControl`, `CMD_LEDNumberDisplayControl`, `CMD_TrajectoryControl`)
- Removed duplicate parameterised CMD overloads (never called; serial parser sets member variables directly)
- Renamed `Application_SmartRobotCarxxx0` → `g_car`
- Renamed `function_xxx` → `inRange`
- Renamed motor direction macros: `direction_just` → `DIR_FORWARD`, `direction_back` → `DIR_BACKWARD`, `direction_void` → `DIR_STOP`
- Extracted shared `delay_wdt()` into `util.h`; removed duplicate definitions from both `.cpp` files
- Removed `volatile` from sensor fields (only accessed from main loop, no ISR involvement)
- Fixed local `ApplicationFunctionSet` instance shadowing the global in motion control
- Removed dead variable `switc_ctrl` in obstacle avoidance
- Removed `/* code */` / `/* constant-expression */` VSCode snippet artifacts from all switch statements
- Removed pre-built binary `SmartRobotCarV4.0_V1_20230201.hex` from the repository

**New features / fixes**
- MPU6050 init failure now prints a warning and disables yaw correction gracefully
- Non-blocking servo: `DeviceDriverSet_Servo_SetAngle()`, `Tick()`, `IsReady()` — obstacle avoidance and follow mode rewritten as `millis()`-based state machines; no more blocking delays in the main loop
- IR remote: raw code constants replaced with a `IRCodeMap` lookup table in `DeviceDriverSet.cpp`

**Comments**
- Added explanatory comments: yaw P-correction loop, blind-scan timing, ultrasonic µs→cm conversion, voltage ADC formula, MPU6050 sensitivity constant, key debounce
- Fixed two pre-existing bugs: `uint8_t c = ""` → `c = 0`; `volatile uint16_t*` cast mismatch
- Removed Chinese comments; translated `README.txt`; replaced with `README.md`
- Updated `.gitignore` for Arduino build artifacts and VSCode generated files

### 2023-02-01 (Zhang Dezhi)
- Command N102 now includes speed control: `Rocker_CarSpeed = doc["D2"]`

### 2022-03-03 (Zhang Dezhi)
- Removed `PinChangeInt` software interrupt library due to compile errors with the latest AVR BOARD package; replaced with hardware interrupt 0
