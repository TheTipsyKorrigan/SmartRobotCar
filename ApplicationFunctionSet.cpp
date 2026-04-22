/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet.h"
#include "DeviceDriverSet.h"
#include "util.h"
#include "ArduinoJson-v6.11.1.h"
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

// Hardware device instances
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

// Returns true if s <= x <= e.
static boolean inRange(long x, long s, long e) {
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

enum SmartRobotCarMotionControl {
  Forward,
  Backward,
  Left,
  Right,
  LeftForward,
  LeftBackward,
  RightForward,
  RightBackward,
  stop_it
};

enum SmartRobotCarFunctionalModel {
  Standby_mode,
  TraceBased_mode,
  ObstacleAvoidance_mode,
  Follow_mode,
  Rocker_mode,
  CMD_inspect_mode,
  CMD_Programming_mode,
  CMD_ClearAllFunctions_Standby_mode,
  CMD_ClearAllFunctions_Programming_mode,
  CMD_MotorControl_mode,
  CMD_CarControl_TimeLimit,
  CMD_CarControl_NoTimeLimit,
  CMD_MotorControl_Speed,
  CMD_ServoControl_mode,
  CMD_LightingControl_TimeLimit,
  CMD_LightingControl_NoTimeLimit,
};

// Application state
struct Application_xxx {
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx g_car;

// Forward declarations for internal helpers
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void) {
  bool res_error = true;
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  if (res_error) {
    Serial.println("WARNING: MPU6050 not found — yaw correction disabled");
    _mpuOk = false;
  } else {
    _mpuOk = true;
    AppMPU6050getdata.MPU6050_calibration();
  }
  g_car.Functional_Mode = Standby_mode;
}

// ---------------------------------------------------------------------------
// Internal motion helpers
// ---------------------------------------------------------------------------

// Uses all three ITR20001 sensors to detect whether the car is airborne.
// All three above TrackingDetection_V means wheels are off the ground.
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void) {
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V) {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  } else {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}

/*
  Straight-line motion control: For dual-drive motors, coefficient deviations
  and external interference make true straight-line motion difficult. A yaw
  feedback loop is used to compensate.

  direction      : Forward or Backward only
  directionRecord: Detects first entry into this direction to latch the yaw reference
  speed          : 0–255
  Kp             : Proportional gain for the yaw error (tuned per mode)
  UpperLimit     : Motor output saturation limit
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit) {
  static float Yaw;
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10) {
    AppMotor.DeviceDriverSet_Motor_control(DIR_STOP, 0, DIR_STOP, 0, control_enable);
    if (Application_FunctionSet._mpuOk) {
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    }
    is_time = millis();
  }
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false) {
    en = directionRecord;
    yaw_So = Yaw;
  }
  // Yaw correction: if the car drifts right (Yaw > yaw_So), boost the right motor
  // and reduce the left to straighten. Kp scales the correction; UpperLimit prevents saturation.
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit) {
    R = UpperLimit;
  } else if (R < 10) {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit) {
    L = UpperLimit;
  } else if (L < 10) {
    L = 10;
  }
  if (direction == Forward) {
    AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, R, DIR_FORWARD, L, control_enable);
  } else if (direction == Backward) {
    AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, L, DIR_BACKWARD, R, control_enable);
  }
}

/*
  Movement direction control.
  direction : Forward(1), Backward(2), Left(3), Right(4), LeftForward(5),
              LeftBackward(6), RightForward(7), RightBackward(8), stop_it(9)
  is_speed  : 0–255
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed) {
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  // Modes that need straight-line yaw correction use lower Kp / UpperLimit to
  // avoid over-correction at moderate speeds.
  switch (g_car.Functional_Mode) {
    case Rocker_mode:
      Kp = 10;
      UpperLimit = 255;
      break;
    case ObstacleAvoidance_mode:
      Kp = 2;
      UpperLimit = 180;
      break;
    case Follow_mode:
      Kp = 2;
      UpperLimit = 180;
      break;
    case CMD_CarControl_TimeLimit:
      Kp = 2;
      UpperLimit = 180;
      break;
    case CMD_CarControl_NoTimeLimit:
      Kp = 2;
      UpperLimit = 180;
      break;
    default:
      Kp = 10;
      UpperLimit = 255;
      break;
  }
  switch (direction) {
    case Forward:
      if (g_car.Functional_Mode == TraceBased_mode) {
        AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, speed, DIR_FORWARD, speed, control_enable);
      } else {
        // Enter yaw-correction loop for straight-line forward movement.
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
        directionRecord = 1;
      }
      break;
    case Backward:
      if (g_car.Functional_Mode == TraceBased_mode) {
        AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, speed, DIR_BACKWARD, speed, control_enable);
      } else {
        // Enter yaw-correction loop for straight-line backward movement.
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
        directionRecord = 2;
      }
      break;
    case Left:
      directionRecord = 3;
      AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, speed, DIR_BACKWARD, speed, control_enable);
      break;
    case Right:
      directionRecord = 4;
      AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, speed, DIR_FORWARD, speed, control_enable);
      break;
    case LeftForward:
      directionRecord = 5;
      AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, speed, DIR_FORWARD, speed / 2, control_enable);
      break;
    case LeftBackward:
      directionRecord = 6;
      AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, speed, DIR_BACKWARD, speed / 2, control_enable);
      break;
    case RightForward:
      directionRecord = 7;
      AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, speed / 2, DIR_FORWARD, speed, control_enable);
      break;
    case RightBackward:
      directionRecord = 8;
      AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, speed / 2, DIR_BACKWARD, speed, control_enable);
      break;
    case stop_it:
      directionRecord = 9;
      AppMotor.DeviceDriverSet_Motor_control(DIR_STOP, 0, DIR_STOP, 0, control_enable);
      break;
    default:
      directionRecord = 10;
      break;
  }
}

// ---------------------------------------------------------------------------
// Sensor update
// ---------------------------------------------------------------------------

// Selective sensor update — only reads what the current loop iteration needs.
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void) {
  // Advance non-blocking servo state machine every loop.
  AppServo.DeviceDriverSet_Servo_Tick();

  { // Battery voltage: sampled every 10 ms; low-battery flag latches after 500 consecutive readings.
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10) {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection) {
        VoltageData_number++;
        if (VoltageData_number == 500) {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      } else {
        VoltageDetectionStatus = false;
      }
    }
  }

  { // IR tracking sensors: read every loop for line-tracking and airborne detection.
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = inRange(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = inRange(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = inRange(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }
}

// ---------------------------------------------------------------------------
// Bootup
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void) {
  g_car.Functional_Mode = Standby_mode;
}

// ---------------------------------------------------------------------------
// Internal lighting helper
// ---------------------------------------------------------------------------

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B) {
  switch (is_LightingSequence) {
    case 0:
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 1: // Left
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 2: // Forward
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 3: // Right
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 4: // Back
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 5: // Middle
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// RGB LED mode display
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void) {
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) {
    if ((millis() - getAnalogue_time) > 3000) {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  // Low-battery: rapid red flashing for the first 500 ms of every 3 s window.
  if (inRange((temp), 0, 500) && VoltageDetectionStatus == true) {
    switch (temp) {
      case 0 ... 49:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 50 ... 99:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 100 ... 149:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 150 ... 199:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 200 ... 249:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 250 ... 299:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 300 ... 349:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 350 ... 399:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 400 ... 449:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 450 ... 499:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      default:
        break;
    }
  } else if (((inRange((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false) {
    // Normal operation: colour encodes the active mode.
    switch (g_car.Functional_Mode) {
      case Standby_mode:
        {
          if (VoltageDetectionStatus == true) {
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
            delay(30);
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
            delay(30);
          } else {
            // Breathing violet when idle and battery is good.
            static uint8_t setBrightness = 0;
            static boolean et = false;
            static unsigned long time = 0;
            if ((millis() - time) > 10) {
              time = millis();
              if (et == false) {
                setBrightness += 1;
                if (setBrightness == 100)
                  et = true;
              } else if (et == true) {
                setBrightness -= 1;
                if (setBrightness == 0)
                  et = false;
              }
            }
            AppRBG_LED.leds[0] = CRGB::Violet;
            FastLED.setBrightness(setBrightness);
            FastLED.show();
          }
        }
        break;
      case CMD_Programming_mode:
        break;
      case TraceBased_mode:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Green);
        break;
      case ObstacleAvoidance_mode:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Yellow);
        break;
      case Follow_mode:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Blue);
        break;
      case Rocker_mode:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Violet);
        break;
      default:
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Rocker control
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Rocker(void) {
  if (g_car.Functional_Mode == Rocker_mode) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(g_car.Motion_Control, Rocker_CarSpeed);
  }
}

// ---------------------------------------------------------------------------
// Line tracking
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void) {
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (g_car.Functional_Mode == TraceBased_mode) {
    if (Car_LeaveTheGround == false) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

#if _Test_print
    static unsigned long print_time = 0;
    if (millis() - print_time > 500) {
      print_time = millis();
      Serial.print("ITR20001_getAnaloguexxx_L=");
      Serial.println(TrackingData_L);
      Serial.print("ITR20001_getAnaloguexxx_M=");
      Serial.println(TrackingData_M);
      Serial.print("ITR20001_getAnaloguexxx_R=");
      Serial.println(TrackingData_R);
    }
#endif

    if (inRange(TrackingData_M, TrackingDetection_S, TrackingDetection_E)) {
      // Line centred — go straight.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    } else if (inRange(TrackingData_R, TrackingDetection_S, TrackingDetection_E)) {
      // Line to the right — turn right.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    } else if (inRange(TrackingData_L, TrackingDetection_S, TrackingDetection_E)) {
      // Line to the left — turn left.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    } else {
      // No line detected — execute blind scan.
      if (timestamp == true) {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      // Blind scan: swing right (0–200 ms and 1600–2000 ms), then left (200–1600 ms).
      // If nothing found after 3 s, stop.
      if ((inRange((millis() - MotorRL_time), 0, 200) || inRange((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      } else if (((inRange((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      } else if ((inRange((millis() - MotorRL_time), 3000, 3500))) {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  } else if (false == timestamp) {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

// ---------------------------------------------------------------------------
// Obstacle avoidance — non-blocking state machine
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void) {
  enum ObstacleState {
    OBS_IDLE,
    OBS_WAIT_CENTER,
    OBS_SCAN,
    OBS_MOVING_FORWARD
  };
  static ObstacleState state = OBS_IDLE;
  static uint8_t scanIndex = 0; // 0 = 30°, 1 = 90°, 2 = 150°
  static unsigned long moveStart = 0;

  if (g_car.Functional_Mode != ObstacleAvoidance_mode) {
    state = OBS_IDLE;
    return;
  }

  if (Car_LeaveTheGround == false) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    state = OBS_IDLE;
    return;
  }

  uint16_t get_Distance;

  switch (state) {
    case OBS_IDLE:
      // First entry: centre servo before measuring.
      AppServo.DeviceDriverSet_Servo_SetAngle(90);
      state = OBS_WAIT_CENTER;
      break;

    case OBS_WAIT_CENTER:
      if (!AppServo.DeviceDriverSet_Servo_IsReady())
        return;
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);
      if (inRange(get_Distance, 0, 20)) {
        // Obstacle ahead — start scan at 30°.
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        scanIndex = 0;
        AppServo.DeviceDriverSet_Servo_SetAngle(30);
        state = OBS_SCAN;
      } else {
        state = OBS_MOVING_FORWARD;
      }
      break;

    case OBS_SCAN:
      if (!AppServo.DeviceDriverSet_Servo_IsReady())
        return;
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);
      if (!inRange(get_Distance, 0, 20)) {
        // Path clear at this angle — choose direction based on scan position.
        switch (scanIndex) {
          case 0: // 30° → right
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            break;
          case 1: // 90° → forward
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
            break;
          case 2: // 150° → left
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
            break;
        }
        moveStart = millis();
        state = OBS_IDLE;
      } else {
        // Still blocked — try next scan angle.
        scanIndex++;
        if (scanIndex < 3) {
          uint8_t angles[3] = {30, 90, 150};
          AppServo.DeviceDriverSet_Servo_SetAngle(angles[scanIndex]);
        } else {
          // All three angles blocked — reverse then turn right.
          ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
          delay_wdt(500);
          ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
          delay_wdt(50);
          state = OBS_IDLE;
        }
      }
      break;

    case OBS_MOVING_FORWARD:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
      // Re-check centre distance on the next cycle.
      state = OBS_WAIT_CENTER;
      break;
  }
}

// ---------------------------------------------------------------------------
// Following mode — non-blocking state machine
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void) {
  enum FollowState {
    FOL_IDLE,
    FOL_SCAN
  };
  static FollowState state = FOL_IDLE;
  static uint16_t ULTRASONIC_Get = 0;
  static uint8_t Position_Servo = 1;
  static uint8_t OneCycle = 1;
  static unsigned long time_Servo = 0;
  static uint8_t Position_Servo_xx = 0;
  static uint8_t timestamp = 3;

  if (g_car.Functional_Mode != Follow_mode) {
    ULTRASONIC_Get = 0;
    Position_Servo = 1;
    OneCycle = 1;
    timestamp = 3;
    state = FOL_IDLE;
    return;
  }

  if (Car_LeaveTheGround == false) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    return;
  }

  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ULTRASONIC_Get);

  if (false == inRange(ULTRASONIC_Get, 0, 20)) {
    // No object within 20 cm — scan with servo to find target.
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

    if (timestamp == 3) {
      if (Position_Servo_xx != Position_Servo) {
        Position_Servo_xx = Position_Servo;
        if (Position_Servo == 1) {
          time_Servo = millis();
          AppServo.DeviceDriverSet_Servo_SetAngle(80);
        } else if (Position_Servo == 2) {
          time_Servo = millis();
          AppServo.DeviceDriverSet_Servo_SetAngle(20);
        } else if (Position_Servo == 3) {
          time_Servo = millis();
          AppServo.DeviceDriverSet_Servo_SetAngle(80);
        } else if (Position_Servo == 4) {
          time_Servo = millis();
          AppServo.DeviceDriverSet_Servo_SetAngle(150);
        }
      }
    } else {
      if (timestamp == 1) {
        timestamp = 2;
        time_Servo = millis();
      }
    }
    // Hold the current servo position for 1 s before advancing to the next scan step.
    if (millis() - time_Servo > 1000) {
      timestamp = 3;
      Position_Servo += 1;
      OneCycle += 1;
      if (OneCycle > 4) {
        Position_Servo = 1;
        OneCycle = 5;
      }
    }
  } else {
    // Object detected within range — drive toward it.
    OneCycle = 1;
    timestamp = 1;
    if ((Position_Servo == 1)) {
      // Object ahead — move forward.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
    } else if ((Position_Servo == 2)) {
      // Object to the right — turn right.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
    } else if ((Position_Servo == 3)) {
      // Object slightly right of centre — move forward.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
    } else if ((Position_Servo == 4)) {
      // Object to the left — turn left.
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
    }
  }
}

// ---------------------------------------------------------------------------
// Servo pan/tilt control
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo) {
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo;

  switch (temp_Set_Servo) {
    case 1 ... 2:
      {
        if (1 == temp_Set_Servo) {
          y_angle -= 1;
        } else if (2 == temp_Set_Servo) {
          y_angle += 1;
        }
        if (y_angle <= 3)  // minimum angle
          y_angle = 3;
        if (y_angle >= 11) // maximum angle
          y_angle = 11;
        AppServo.DeviceDriverSet_Servo_controls(2, y_angle);
      }
      break;

    case 3 ... 4:
      {
        if (3 == temp_Set_Servo) {
          z_angle += 1;
        } else if (4 == temp_Set_Servo) {
          z_angle -= 1;
        }
        if (z_angle <= 1)  // minimum angle
          z_angle = 1;
        if (z_angle >= 17) // maximum angle
          z_angle = 17;
        AppServo.DeviceDriverSet_Servo_controls(1, z_angle);
      }
      break;

    case 5:
      AppServo.DeviceDriverSet_Servo_controls(2, 9);
      AppServo.DeviceDriverSet_Servo_controls(1, 9);
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Standby mode
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void) {
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (g_car.Functional_Mode == Standby_mode) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    // Re-calibrate the IMU yaw origin once the car has been stationary on the
    // ground for at least 10 consecutive 20 ms samples (200 ms total).
    if (true == is_ED) {
      static unsigned long timestamp;
      if (millis() - timestamp > 20) {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround()) {
          cout += 1;
        } else {
          cout = 0;
        }
        if (cout > 10) {
          is_ED = false;
          if (_mpuOk) {
            AppMPU6050getdata.MPU6050_calibration();
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// CMD — Graphical programming and command control module
// Elegoo & SmartRobot & 2020-06
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::CMD_inspect(void) {
  if (g_car.Functional_Mode == CMD_inspect_mode) {
    Serial.println("CMD_inspect");
    delay(100);
  }
}

// N1: Unidirectional motor drive command (no time limit).
void ApplicationFunctionSet::CMD_MotorControl(void) {
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (g_car.Functional_Mode == CMD_MotorControl_mode) {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      switch (CMD_is_MotorSelection) {
        case 0: // Both motors
          {
            is_MotorSpeed_A = CMD_is_MotorSpeed;
            is_MotorSpeed_B = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, is_MotorSpeed_A, DIR_FORWARD, is_MotorSpeed_B, control_enable);
            } else if (2 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, is_MotorSpeed_A, DIR_BACKWARD, is_MotorSpeed_B, control_enable);
            } else {
              return;
            }
          }
          break;
        case 1: // Motor A only
          {
            is_MotorSpeed_A = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, is_MotorSpeed_A, DIR_STOP, is_MotorSpeed_B, control_enable);
            } else if (2 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_BACKWARD, is_MotorSpeed_A, DIR_STOP, is_MotorSpeed_B, control_enable);
            } else {
              return;
            }
          }
          break;
        case 2: // Motor B only
          {
            is_MotorSpeed_B = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_STOP, is_MotorSpeed_A, DIR_FORWARD, is_MotorSpeed_B, control_enable);
            } else if (2 == CMD_is_MotorDirection) {
              AppMotor.DeviceDriverSet_Motor_control(DIR_STOP, is_MotorSpeed_A, DIR_BACKWARD, is_MotorSpeed_B, control_enable);
            } else {
              return;
            }
          }
          break;
        default:
          break;
      }
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed) {
  switch (is_CarDirection) {
    case 1:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, is_CarSpeed);
      break;
    case 2:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, is_CarSpeed);
      break;
    case 3:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, is_CarSpeed);
      break;
    case 4:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, is_CarSpeed);
      break;
    default:
      break;
  }
}

// N2: Direction and speed control with a time limit; enters Programming mode when time expires.
void ApplicationFunctionSet::CMD_CarControlTimeLimit(void) {
  static boolean CarControl = false;
  static boolean CarControl_TE = false;    // Time expired flag
  static boolean CarControl_return = false;
  if (g_car.Functional_Mode == CMD_CarControl_TimeLimit) {
    CarControl = true;
    if (CMD_is_CarTimer != 0) {
      if ((millis() - g_car.CMD_CarControl_Millis) > (CMD_is_CarTimer)) {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        g_car.Functional_Mode = CMD_Programming_mode;
        if (CarControl_return == false) {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      } else {
        CarControl_TE = false;
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false) {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  } else {
    if (CarControl == true) {
      CarControl_return = false;
      CarControl = false;
      g_car.CMD_CarControl_Millis = 0;
    }
  }
}

// N3: Direction and speed control without a time limit.
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit(void) {
  static boolean CarControl = false;
  if (g_car.Functional_Mode == CMD_CarControl_NoTimeLimit) {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  } else {
    if (CarControl == true) {
      CarControl = false;
    }
  }
}

// N4: Independent left/right motor speed control.
void ApplicationFunctionSet::CMD_MotorControlSpeed(void) {
  static boolean MotorControl = false;
  if (g_car.Functional_Mode == CMD_MotorControl_Speed) {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      AppMotor.DeviceDriverSet_Motor_control(DIR_FORWARD, CMD_is_MotorSpeed_L, DIR_FORWARD, CMD_is_MotorSpeed_R, control_enable);
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
    }
  }
}

// N5: Move the selected servo to the commanded angle, then return to Programming mode.
void ApplicationFunctionSet::CMD_ServoControl(void) {
  if (g_car.Functional_Mode == CMD_ServoControl_mode) {
    AppServo.DeviceDriverSet_Servo_controls(CMD_is_Servo, CMD_is_Servo_angle / 10);
    g_car.Functional_Mode = CMD_Programming_mode;
  }
}

// N7: RGB lighting control with a time limit; enters Programming mode when time expires.
void ApplicationFunctionSet::CMD_LightingControlTimeLimit(void) {
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false;    // Time expired flag
  static boolean LightingControl_return = false;

  if (g_car.Functional_Mode == CMD_LightingControl_TimeLimit) {
    LightingControl = true;
    if (CMD_is_LightingTimer != 0) {
      if ((millis() - g_car.CMD_LightingControl_Millis) > (CMD_is_LightingTimer)) {
        LightingControl_TE = true;
        FastLED.clear(true);
        g_car.Functional_Mode = CMD_Programming_mode;
        if (LightingControl_return == false) {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      } else {
        LightingControl_TE = false;
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false) {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  } else {
    if (LightingControl == true) {
      LightingControl_return = false;
      LightingControl = false;
      g_car.CMD_LightingControl_Millis = 0;
    }
  }
}

// N8: RGB lighting control without a time limit.
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit(void) {
  static boolean LightingControl = false;
  if (g_car.Functional_Mode == CMD_LightingControl_NoTimeLimit) {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  } else {
    if (LightingControl == true) {
      LightingControl = false;
    }
  }
}

// N100/N110: Stop all motion and LEDs, then switch to Standby or Programming mode.
void ApplicationFunctionSet::CMD_ClearAllFunctions(void) {
  if (g_car.Functional_Mode == CMD_ClearAllFunctions_Standby_mode) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black);
    g_car.Motion_Control = stop_it;
    g_car.Functional_Mode = Standby_mode;
  }
  if (g_car.Functional_Mode == CMD_ClearAllFunctions_Programming_mode) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black);
    g_car.Motion_Control = stop_it;
    g_car.Functional_Mode = CMD_Programming_mode;
  }
}

// N21: Query ultrasonic sensor and report obstacle presence or distance.
void ApplicationFunctionSet::CMD_UltrasoundModuleStatus(uint8_t is_get) {
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm);
  UltrasoundDetectionStatus = inRange(UltrasoundData_cm, 0, ObstacleDetection);
  if (1 == is_get) {
    // Report boolean obstacle status: true = obstacle present.
    if (true == UltrasoundDetectionStatus) {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    } else {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  } else if (2 == is_get) {
    // Report raw distance value.
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}

// N22: Query IR tracking sensors and report the requested channel ADC value.
void ApplicationFunctionSet::CMD_TraceModuleStatus(uint8_t is_get) {
  char toString[10];
  if (0 == is_get) {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  } else if (1 == is_get) {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  } else if (2 == is_get) {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
  g_car.Functional_Mode = CMD_Programming_mode;
}

// ---------------------------------------------------------------------------
// Key command
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void) {
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue) {
    temp_keyValue = get_keyValue;
    switch (get_keyValue) {
      case 1:
        g_car.Functional_Mode = TraceBased_mode;
        break;
      case 2:
        g_car.Functional_Mode = ObstacleAvoidance_mode;
        break;
      case 3:
        g_car.Functional_Mode = Follow_mode;
        break;
      case 4:
        g_car.Functional_Mode = Standby_mode;
        break;
      default:
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// IR remote control
// ---------------------------------------------------------------------------

void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void) {
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button)) {
    IRrecv_en = true;
  }
  if (true == IRrecv_en) {
    switch (IRrecv_button) {
      case 1:
        g_car.Motion_Control = Forward;
        break;
      case 2:
        g_car.Motion_Control = Backward;
        break;
      case 3:
        g_car.Motion_Control = Left;
        break;
      case 4:
        g_car.Motion_Control = Right;
        break;
      case 5:
        g_car.Functional_Mode = Standby_mode;
        break;
      case 6:
        g_car.Functional_Mode = TraceBased_mode;
        break;
      case 7:
        g_car.Functional_Mode = ObstacleAvoidance_mode;
        break;
      case 8:
        g_car.Functional_Mode = Follow_mode;
        break;
      case 9:
        // Raise the line-detection threshold to adapt to a brighter surface.
        if (g_car.Functional_Mode == TraceBased_mode) {
          if (TrackingDetection_S < 600) {
            TrackingDetection_S += 10;
          }
        }
        break;
      case 10:
        // Reset line-detection threshold to the factory default.
        if (g_car.Functional_Mode == TraceBased_mode) {
          TrackingDetection_S = 250;
        }
        break;
      case 11:
        // Lower the line-detection threshold to adapt to a darker surface.
        if (g_car.Functional_Mode == TraceBased_mode) {
          if (TrackingDetection_S > 30) {
            TrackingDetection_S -= 10;
          }
        }
        break;
      case 12:
        {
          if (Rocker_CarSpeed < 255) {
            Rocker_CarSpeed += 5;
          }
        }
        break;
      case 13:
        {
          Rocker_CarSpeed = 250;
        }
        break;
      case 14:
        {
          if (Rocker_CarSpeed > 50) {
            Rocker_CarSpeed -= 5;
          }
        }
        break;
      default:
        g_car.Functional_Mode = Standby_mode;
        break;
    }
    // Motion buttons (1–4) enter Rocker mode with a 300 ms hold timeout;
    // if the button is not repeated within 300 ms the car stops.
    if (IRrecv_button < 5) {
      g_car.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300) {
        IRrecv_en = false;
        g_car.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    } else {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}

// ---------------------------------------------------------------------------
// Serial port JSON command parser
// ---------------------------------------------------------------------------

// See "Communication protocol for Smart Robot Car.pdf" for the full command set.
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void) {
  static String SerialPortData = "";
  uint8_t c = 0;
  if (Serial.available() > 0) {
    while (c != '}' && Serial.available() > 0) {
      c = Serial.read();
      SerialPortData += (char)c;
    }
  }
  if (c == '}') {
#if _Test_print
    Serial.println(SerialPortData);
#endif
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, SerialPortData);
    SerialPortData = "";
    if (error) {
      Serial.println("error:deserializeJson");
    } else if (!error) {
      int control_mode_N = doc["N"];
      char *temp = doc["H"];
      CommandSerialNumber = temp;

      switch (control_mode_N) {
        case 1: // N1: motor control mode
          g_car.Functional_Mode = CMD_MotorControl_mode;
          CMD_is_MotorSelection = doc["D1"];
          CMD_is_MotorSpeed = doc["D2"];
          CMD_is_MotorDirection = doc["D3"];
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 2: // N2: car direction control, time-limited
          g_car.Functional_Mode = CMD_CarControl_TimeLimit;
          CMD_is_CarDirection = doc["D1"];
          CMD_is_CarSpeed = doc["D2"];
          CMD_is_CarTimer = doc["T"];
          g_car.CMD_CarControl_Millis = millis();
          break;

        case 3: // N3: car direction control, no time limit
          g_car.Functional_Mode = CMD_CarControl_NoTimeLimit;
          CMD_is_CarDirection = doc["D1"];
          CMD_is_CarSpeed = doc["D2"];
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 4: // N4: independent motor speed control
          g_car.Functional_Mode = CMD_MotorControl_Speed;
          CMD_is_MotorSpeed_L = doc["D1"];
          CMD_is_MotorSpeed_R = doc["D2"];
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 5: // N5: servo angle control
          g_car.Functional_Mode = CMD_ServoControl_mode;
          CMD_is_Servo = doc["D1"];
          CMD_is_Servo_angle = doc["D2"];
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 7: // N7: lighting control, time-limited
          g_car.Functional_Mode = CMD_LightingControl_TimeLimit;
          CMD_is_LightingSequence = doc["D1"];
          CMD_is_LightingColorValue_R = doc["D2"];
          CMD_is_LightingColorValue_G = doc["D3"];
          CMD_is_LightingColorValue_B = doc["D4"];
          CMD_is_LightingTimer = doc["T"];
          g_car.CMD_LightingControl_Millis = millis();
          break;

        case 8: // N8: lighting control, no time limit
          g_car.Functional_Mode = CMD_LightingControl_NoTimeLimit;
          CMD_is_LightingSequence = doc["D1"];
          CMD_is_LightingColorValue_R = doc["D2"];
          CMD_is_LightingColorValue_G = doc["D3"];
          CMD_is_LightingColorValue_B = doc["D4"];
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 21: // N21: ultrasonic obstacle query
          CMD_UltrasoundModuleStatus(doc["D1"]);
          break;

        case 22: // N22: IR tracking sensor query
          CMD_TraceModuleStatus(doc["D1"]);
          break;

        case 23: // N23: airborne detection query
          if (true == Car_LeaveTheGround) {
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_false}");
#endif
          } else if (false == Car_LeaveTheGround) {
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_true}");
#endif
          }
          break;

        case 110: // N110: clear all functions, enter Programming mode
          g_car.Functional_Mode = CMD_ClearAllFunctions_Programming_mode;
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          break;

        case 100: // N100: clear all functions, enter Standby mode
          g_car.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
#if _is_print
          Serial.print("{ok}");
#endif
          break;

        case 101: // N101: switch active autonomous mode
          if (1 == doc["D1"]) {
            g_car.Functional_Mode = TraceBased_mode;
          } else if (2 == doc["D1"]) {
            g_car.Functional_Mode = ObstacleAvoidance_mode;
          } else if (3 == doc["D1"]) {
            g_car.Functional_Mode = Follow_mode;
          }
#if _is_print
          Serial.print("{ok}");
#endif
          break;

        case 105: // N105: adjust FastLED global brightness
          if (1 == doc["D1"] && (CMD_is_FastLED_setBrightness < 250)) {
            CMD_is_FastLED_setBrightness += 5;
          } else if (2 == doc["D1"] && (CMD_is_FastLED_setBrightness > 0)) {
            CMD_is_FastLED_setBrightness -= 5;
          }
          FastLED.setBrightness(CMD_is_FastLED_setBrightness);
#if _Test_print
          Serial.print("{ok}");
#endif
          break;

        case 106: // N106: pan/tilt servo step control
          {
            uint8_t temp_Set_Servo = doc["D1"];
            if (temp_Set_Servo > 5 || temp_Set_Servo < 1)
              return;
            ApplicationFunctionSet_Servo(temp_Set_Servo);
          }
#if _is_print
          Serial.print("{ok}");
#endif
          break;

        case 102: // N102: rocker (joystick) direction and speed command
          g_car.Functional_Mode = Rocker_mode;
          Rocker_temp = doc["D1"];
          Rocker_CarSpeed = doc["D2"];
          switch (Rocker_temp) {
            case 1:
              g_car.Motion_Control = Forward;
              break;
            case 2:
              g_car.Motion_Control = Backward;
              break;
            case 3:
              g_car.Motion_Control = Left;
              break;
            case 4:
              g_car.Motion_Control = Right;
              break;
            case 5:
              g_car.Motion_Control = LeftForward;
              break;
            case 6:
              g_car.Motion_Control = LeftBackward;
              break;
            case 7:
              g_car.Motion_Control = RightForward;
              break;
            case 8:
              g_car.Motion_Control = RightBackward;
              break;
            case 9:
              g_car.Motion_Control = stop_it;
              g_car.Functional_Mode = Standby_mode;
              break;
            default:
              g_car.Motion_Control = stop_it;
              break;
          }
          break;

        default:
          break;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Hardware test runner
// ---------------------------------------------------------------------------

#if _Test_DeviceDriverSet
void ApplicationFunctionSet::RunHardwareTests(void) {
  AppRBG_LED.DeviceDriverSet_RBGLED_Test();
  AppKey.DeviceDriverSet_Key_Test();
  AppITR20001.DeviceDriverSet_ITR20001_Test();
  AppVoltage.DeviceDriverSet_Voltage_Test();
  AppMotor.DeviceDriverSet_Motor_Test();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Test();
  AppServo.DeviceDriverSet_Servo_Test();
  AppIRrecv.DeviceDriverSet_IRrecv_Test();
}
#endif
