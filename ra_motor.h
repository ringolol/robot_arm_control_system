#ifndef RA_MOTOR_H
#define RA_MOTOR_H

#include <Arduino.h>
#include <math.h>

#include <Wire.h>

#include "ra_utils.h"
#include "ra_kalman.h"
#include "ra_pid.h"


/* Motor class. It includes motor, hall sensor, accelerator, gyroscope, 
   speed PID regulator and position PID regulator. */
class RaMotor {
private:
  /* h-bridge */
  short hArm1;
  short hArm2;
  short pwm;

  /* gyro/accel sensor */
  short mpu;
  float pos_sign;
  float spd_gain;

  /* hall sensor */
  #ifdef USE_HALL
  short hall;
  int *pulces = NULL;
  int *spdBuff = NULL;
  #endif
  
  /* voltage */
  float U = 0.0, u0 = 0;

public:
  /* acceleremeter data */
  int16_t accelerometer_x, accelerometer_y, accelerometer_z;

  /* gyroscope data */
  int16_t gyro_x, gyro_y, gyro_z;

  /* temperature data */
  int16_t temperature;

  /* Kalman Filter */
  RaKalman *kalman = NULL;

  /* PIDs */
  RaPID *SpeedPID, *PosPID;
  
private:
  /* set rotation direction clockwise or conter-clockwise */
  void setDirection(bool cw=true);

public:
  RaMotor(short a1, short a2, short p, 
          short m, float sg, float u0_);

  void setPIDPosPar(float p, float i, float d=0, float s=0, int n=3);
  void PIDPos(float pos_r, float pos_m);

  void setPIDSpdPar(float p, float i, float d=0, float s=0, int n=3);
  void PIDSpd(float spd_r, float spd_m);

  void setVoltage(float s);

  void fetchSensor();
  void fetch_upd();
  float getPosition();
  float getSpeed();
  
  void stop();

  float getU() {return U;}

  #ifdef USE_HALL
  RaMotor(short a1, short a2, short p, 
          short m, float sg, float u0_, 
          short h, int *sb, void (*fun)(void));
  float getHallSpeed();
  #endif
};

#endif
