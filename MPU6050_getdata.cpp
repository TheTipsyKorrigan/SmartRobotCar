/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2026-04-21
 * @LastEditors: david
 * @Description: MPU6050 Data solution
 * @FilePath:
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

bool MPU6050_getdata::MPU6050_dveInit(void) {
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout;
  // Poll until a valid chip ID is returned, giving up after 10 attempts.
  do {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10) {
      return true;
    }
  } while (chip_id == 0x00 || chip_id == 0xFF);
  accelgyro.initialize();
  return false;
}

bool MPU6050_getdata::MPU6050_calibration(void) {
  unsigned short times = 100; // Number of samples averaged for offset calculation
  for (int i = 0; i < times; i++) {
    gz = accelgyro.getRotationZ();
    gzo += gz;
  }
  gzo /= times;
  return false;
}

bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw) {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;  // Elapsed time in seconds
  lastTime = now;
  gz = accelgyro.getRotationZ();
  // 131 LSB/(°/s) is MPU6050 sensitivity at ±250°/s range; subtract offset to remove bias.
  float gyroz = -(gz - gzo) / 131.0 * dt;
  // Dead zone: discard noise below 0.05°/s to prevent yaw drift while stationary.
  if (fabs(gyroz) < 0.05) {
    gyroz = 0.00;
  }
  agz += gyroz;
  *Yaw = agz;
  return false;
}
