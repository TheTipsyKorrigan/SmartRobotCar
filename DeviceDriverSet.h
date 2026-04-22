/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#ifndef _DeviceDriverSet_H_
#define _DeviceDriverSet_H_

#define _Test_DeviceDriverSet 0

// ---------------------------------------------------------------------------
// RGB LED
// ---------------------------------------------------------------------------
#include "FastLED.h"
class DeviceDriverSet_RBGLED
{
public:
  // Initialises the FastLED driver and sets the global brightness level.
  void DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness);
  // Sets Traversal_Number LEDs to colour, holding each for Duration ms.
  void DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_RBGLED_Test(void);
#endif
  // Sets a single LED (LED_s index) to the given RGB value.
  void DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b);

public:
private:
#define PIN_RBGLED 4
#define NUM_LEDS 1
public:
  CRGB leds[NUM_LEDS];
};

// ---------------------------------------------------------------------------
// Key Detection
// ---------------------------------------------------------------------------
class DeviceDriverSet_Key
{
public:
  // Attaches the interrupt for the mode-selection button.
  void DeviceDriverSet_Key_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Key_Test(void);
#endif
  // Writes the current debounced key value to *get_keyValue.
  void DeviceDriverSet_key_Get(uint8_t *get_keyValue);

public:
#define PIN_Key 2
#define keyValue_Max 4
public:
  static uint8_t keyValue;
};

// ---------------------------------------------------------------------------
// ITR20001 Line-tracking IR Sensors
// ---------------------------------------------------------------------------
class DeviceDriverSet_ITR20001
{
public:
  // Configures sensor pins; returns false on success.
  bool DeviceDriverSet_ITR20001_Init(void);
  // Returns the raw ADC reading from the left IR sensor.
  int DeviceDriverSet_ITR20001_getAnaloguexxx_L(void);
  // Returns the raw ADC reading from the middle IR sensor.
  int DeviceDriverSet_ITR20001_getAnaloguexxx_M(void);
  // Returns the raw ADC reading from the right IR sensor.
  int DeviceDriverSet_ITR20001_getAnaloguexxx_R(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ITR20001_Test(void);
#endif

private:
// Pin mapping for hardware revision 04 (A0/A1/A2 swapped vs rev 03).
#define PIN_ITR20001xxxL A2
#define PIN_ITR20001xxxM A1
#define PIN_ITR20001xxxR A0
};

// ---------------------------------------------------------------------------
// Voltage Detection
// ---------------------------------------------------------------------------
class DeviceDriverSet_Voltage
{
public:
  // Configures the voltage-divider ADC pin.
  void DeviceDriverSet_Voltage_Init(void);
  // Returns the battery voltage in volts after divider scaling and compensation.
  float DeviceDriverSet_Voltage_getAnalogue(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Voltage_Test(void);
#endif
private:
#define PIN_Voltage A3
};

// ---------------------------------------------------------------------------
// Motor (TB6612 dual H-bridge)
// ---------------------------------------------------------------------------
class DeviceDriverSet_Motor
{
public:
  // Configures all TB6612 control pins as outputs.
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  // Drives both motors: direction and speed for group A (right) and B (left),
  // with controlED gating the STBY enable line.
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, // Group A motor parameters
                                     boolean direction_B, uint8_t speed_B, // Group B motor parameters
                                     boolean controlED                     // AB enable setting (true)
  );
private:
// TB6612 pin assignments
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3
public:
#define speed_Max 255
#define DIR_FORWARD  true
#define DIR_BACKWARD false
#define DIR_STOP     3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};

// ---------------------------------------------------------------------------
// Ultrasonic Sensor (HC-SR04)
// ---------------------------------------------------------------------------
class DeviceDriverSet_ULTRASONIC
{
public:
  // Configures TRIG and ECHO pins.
  void DeviceDriverSet_ULTRASONIC_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ULTRASONIC_Test(void);
#endif
  // Fires a sonar pulse and writes the measured distance in cm to *ULTRASONIC_Get;
  // clamped to MAX_DISTANCE.
  void DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/);

private:
#define TRIG_PIN 13
#define ECHO_PIN 12
// Maximum measurable distance in cm; sensor is rated to 400–500 cm.
#define MAX_DISTANCE 200
};

// ---------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------
#include <Servo.h>
class DeviceDriverSet_Servo
{
public:
  // Centres both servos to Position_angle and then detaches to reduce jitter.
  void DeviceDriverSet_Servo_Init(unsigned int Position_angle);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Servo_Test(void);
#endif
  // Begins a non-blocking move of the Z-axis servo to Position_angle degrees.
  void DeviceDriverSet_Servo_SetAngle(unsigned int Position_angle);
  // Begins a non-blocking move of the selected servo (1=Z, 2=Y, 3=both).
  void DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle);
  // Advances the non-blocking servo state machine; detaches after MOVE_TIME_MS.
  void DeviceDriverSet_Servo_Tick();
  // Returns true once the servo has finished moving (or was never started).
  bool DeviceDriverSet_Servo_IsReady();

private:
#define PIN_Servo_z 10
#define PIN_Servo_y 11
  unsigned long _moveStart = 0;
  bool _moving = false;
  static const uint16_t MOVE_TIME_MS = 500;
};

// ---------------------------------------------------------------------------
// IR Receiver
// ---------------------------------------------------------------------------
#include "IRremote.h"
class DeviceDriverSet_IRrecv
{
public:
  // Enables the IR receiver on RECV_PIN.
  void DeviceDriverSet_IRrecv_Init(void);
  // Decodes the next IR frame and maps it to a button ID; returns true on success.
  bool DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/);
  void DeviceDriverSet_IRrecv_Test(void);

public:
  unsigned long IR_PreMillis;

private:
#define RECV_PIN 9

  static const uint8_t IR_TABLE_SIZE = 14;
  struct IrEntry { unsigned long code; uint8_t buttonId; };
  static const IrEntry IR_TABLE[IR_TABLE_SIZE * 2];
};

#endif
