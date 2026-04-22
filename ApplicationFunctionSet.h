/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#ifndef _ApplicationFunctionSet_H_
#define _ApplicationFunctionSet_H_

#include <Arduino.h>
#include "DeviceDriverSet.h"

class ApplicationFunctionSet
{
public:
  // Initialises all hardware peripherals and sets mode to Standby.
  void ApplicationFunctionSet_Init(void);
  // Resets the car to Standby mode on startup.
  void ApplicationFunctionSet_Bootup(void);
  // Updates the RGB LED to reflect the current mode or battery state.
  void ApplicationFunctionSet_RGB(void);
  // (Reserved for future expression/animation sequences.)
  void ApplicationFunctionSet_Expression(void);
  // Executes rocker (joystick) control when in Rocker mode.
  void ApplicationFunctionSet_Rocker(void);
  // Runs the line-tracking state machine when in TraceBased mode.
  void ApplicationFunctionSet_Tracking(void);
  // Runs the obstacle-avoidance state machine when in ObstacleAvoidance mode.
  void ApplicationFunctionSet_Obstacle(void);
  // Runs the object-following state machine when in Follow mode.
  void ApplicationFunctionSet_Follow(void);
  // Moves the pan/tilt servos by one step in the direction encoded in Set_Servo.
  void ApplicationFunctionSet_Servo(uint8_t Set_Servo);
  // Stops the car and re-calibrates the IMU yaw origin while in Standby mode.
  void ApplicationFunctionSet_Standby(void);
  // Reads the physical mode-select button and updates the active mode.
  void ApplicationFunctionSet_KeyCommand(void);
  // Refreshes all sensor readings used by the active mode this loop cycle.
  void ApplicationFunctionSet_SensorDataUpdate(void);
  // Parses incoming JSON frames from the serial port and dispatches commands.
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  // Reads and dispatches a decoded IR remote button press.
  void ApplicationFunctionSet_IRrecv(void);

#if _Test_DeviceDriverSet
  // Runs all hardware self-tests sequentially; call from loop() when _Test_DeviceDriverSet is 1.
  void RunHardwareTests(void);
#endif

public:
  // Queries the ultrasonic sensor and reports obstacle status or distance.
  void CMD_UltrasoundModuleStatus(uint8_t is_get);
  // Queries the IR tracking sensors and reports their ADC values.
  void CMD_TraceModuleStatus(uint8_t is_get);
  // Reports whether the car is currently detected as airborne.
  void CMD_Car_LeaveTheGround(uint8_t is_get);

  void CMD_inspect(void);
  // Applies individual motor direction and speed from CMD_ variables.
  void CMD_MotorControl(void);
  // Drives the car in a commanded direction for a fixed time then stops.
  void CMD_CarControlTimeLimit(void);
  // Drives the car in a commanded direction indefinitely until superseded.
  void CMD_CarControlNoTimeLimit(void);
  // Sets independent left/right motor speeds from CMD_ variables.
  void CMD_MotorControlSpeed(void);
  // Moves the selected servo to the commanded angle and returns to Programming mode.
  void CMD_ServoControl(void);
  // Activates the RGB LED with the commanded colour for a fixed duration.
  void CMD_LightingControlTimeLimit(void);
  // Activates the RGB LED with the commanded colour indefinitely.
  void CMD_LightingControlNoTimeLimit(void);
  // Stops all motion and LEDs, then transitions to Standby or Programming mode.
  void CMD_ClearAllFunctions(void);

private:
  // Sensor raw values
  float VoltageData_V;        // Battery voltage (V)
  uint16_t UltrasoundData_mm; // Ultrasonic distance (mm)
  uint16_t UltrasoundData_cm; // Ultrasonic distance (cm)
  int TrackingData_L;         // Left IR sensor ADC value
  int TrackingData_M;         // Middle IR sensor ADC value
  int TrackingData_R;         // Right IR sensor ADC value

  // Derived sensor status flags
  boolean VoltageDetectionStatus = false;
  boolean UltrasoundDetectionStatus = false;
  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;
  // true if MPU6050 initialised successfully; gates yaw-correction code paths.
  bool _mpuOk = false;

  // Sensor threshold settings
  const float VoltageDetection = 7.00;  // Low-battery threshold (V)
  const uint8_t ObstacleDetection = 20; // Obstacle detection range (cm)

  String CommandSerialNumber;
  uint8_t Rocker_CarSpeed = 250;
  uint8_t Rocker_temp;

public:
  uint8_t TrackingDetection_S = 250;   // IR sensor lower threshold (line detected)
  uint16_t TrackingDetection_E = 850;  // IR sensor upper threshold (line detected)
  uint16_t TrackingDetection_V = 950;  // IR sensor threshold for leave-the-ground detection

public:
  uint8_t CMD_is_Servo;
  uint8_t CMD_is_Servo_angle;

public:
  uint8_t CMD_is_MotorSelection;
  uint8_t CMD_is_MotorDirection;
  uint8_t CMD_is_MotorSpeed;
  uint32_t CMD_is_MotorTimer;

public:
  uint8_t CMD_is_CarDirection;
  uint8_t CMD_is_CarSpeed;
  uint32_t CMD_is_CarTimer;

public:
  uint8_t CMD_is_MotorSpeed_L;
  uint8_t CMD_is_MotorSpeed_R;

public:
  // LED position: 0=all, 1=left, 2=front, 3=right, 4=back, 5=centre
  uint8_t CMD_is_LightingSequence;
  uint8_t CMD_is_LightingColorValue_R;
  uint8_t CMD_is_LightingColorValue_G;
  uint8_t CMD_is_LightingColorValue_B;
  uint32_t CMD_is_LightingTimer;

private:
  uint8_t CMD_is_FastLED_setBrightness = 20;
};

extern ApplicationFunctionSet Application_FunctionSet;
#endif
