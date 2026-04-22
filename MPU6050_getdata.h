/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 17:22:13
 * @LastEditors: Changhua
 * @Description: MPU6050 Data solution
 * @FilePath:
 */
#ifndef _MPU6050_getdata_H_
#define _MPU6050_getdata_H_
#include <Arduino.h>

class MPU6050_getdata
{
public:
  // Initialises the MPU6050 over I2C; returns true on failure (device not found).
  bool MPU6050_dveInit(void);
  // Samples the Z-axis gyro 100 times to compute and store the zero-rate offset.
  bool MPU6050_calibration(void);
  // Integrates the Z-axis gyro rate to update the accumulated yaw angle in *Yaw.
  bool MPU6050_dveGetEulerAngles(float *Yaw);

public:
  int16_t gz;
  unsigned long now, lastTime = 0;
  float dt;      // Derivative time (seconds between samples)
  float agz = 0; // Accumulated yaw angle (degrees)
  long gzo = 0;  // Gyro Z zero-rate offset (raw LSB units)
};

extern MPU6050_getdata MPU6050Getdata;
#endif
