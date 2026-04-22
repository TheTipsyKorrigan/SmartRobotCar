/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"

void setup() {
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
}

#if _Test_DeviceDriverSet
void loop() {
  wdt_reset();
  Application_FunctionSet.RunHardwareTests();
}
#else
void loop() {
  wdt_reset();
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_Follow();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  Application_FunctionSet.ApplicationFunctionSet_Rocker();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  Application_FunctionSet.ApplicationFunctionSet_IRrecv();
  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

  Application_FunctionSet.CMD_ServoControl();
  Application_FunctionSet.CMD_MotorControl();
  Application_FunctionSet.CMD_CarControlTimeLimit();
  Application_FunctionSet.CMD_CarControlNoTimeLimit();
  Application_FunctionSet.CMD_MotorControlSpeed();
  Application_FunctionSet.CMD_LightingControlTimeLimit();
  Application_FunctionSet.CMD_LightingControlNoTimeLimit();
  Application_FunctionSet.CMD_ClearAllFunctions();
}
#endif
