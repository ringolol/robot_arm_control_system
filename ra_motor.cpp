#include "ra_motor.h"

RaMotor::RaMotor(short a1, short a2, short p, short m, float sg, float u0_) {
  hArm1 = a1;
  hArm2 = a2;
  pwm = p;
  mpu = m;
  pos_sign = sg;
  spd_gain = pos_sign * ACC_RES * PI / 180.0f / -32768.0f;
  
  u0 = u0_;

  short outputPins[] = {
    hArm1, hArm2, pwm
  };

  for(auto pin: outputPins) {
    pinMode(pin, OUTPUT);
  };
  
  setDirection(true);

  analogWrite(pwm, 0);
  
  Wire.begin();
  // Begins a transmission to the I2C slave (GY-521 board)
  Wire.beginTransmission(mpu);
  // PWR_MGMT_1 register
  Wire.write(0x6B); 
  // set to zero (wakes up the MPU-6050)
  Wire.write(0); 
  Wire.endTransmission(true);

  // set acc resolution
  Wire.beginTransmission(mpu);
  Wire.write(0x1B);
  switch (ACC_RES)
  {
  case 500:
    Wire.write(0x08);
    break;

  case 1000:
    Wire.write(0x10);
    break;
  
  default:
  Wire.write(0x00);
    break;
  }
  Wire.endTransmission(true);

  delay(100);

  fetchSensor();
  float kal_dt = PID_wait_sec;
  float plant_k = .762f, plant_T = .0625f;
  float f[] = {
    1.0f, kal_dt, 
    .0f, 1.0f - kal_dt/plant_T
  };
  float b[] = {
    .0f, 
    plant_k * kal_dt / plant_T
  };
  float q[] = {
    .03f, .0f, 
    .0f, .05f
  };
  float r[] = {
    .5f, .0f, 
    .0f, .5f
  };
  float x0[] = {
    getPosition(), 
    .0f
  };
  float p0[] = {
    .0f, .0f, 
    .0f, .0f
  };
  kalman = new RaKalman(
    RaMatrix(f, 2, 2), 
    RaMatrix(b, 2, 1), 
    RaMatrix(q, 2, 2), 
    RaMatrix(r, 2, 2),
    RaMatrix(x0, 2, 1),
    RaMatrix(p0, 2, 2)
  );

  SpeedPID = new RaPID();
  PosPID = new RaPID();
}

void RaMotor::setDirection(bool cw) {
  digitalWrite(cw?hArm1:hArm2, LOW);
  digitalWrite(cw?hArm2:hArm1, HIGH);
}

void RaMotor::setVoltage(float u) {
  // +/- 10.5 V

  setDirection(u<0);
  float ur = abs(u);
  ur += u0;
  if(ur > 10.5) {
    ur = 10.5;
  }
  U = sign(u) * ur;
  analogWrite(pwm, ur * VOL_TO_BIT);
}

void RaMotor::fetchSensor() {
  Wire.beginTransmission(mpu);
  // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.write(0x3B); 
  // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.endTransmission(false); 
  // request a total of 7*2=14 registers
  Wire.requestFrom(mpu, 7*2, true); 
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_x = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  temperature = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); 
  // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); 
}

void RaMotor::setPIDPosPar(float p, float i, float d, float s, int n) {
  PosPID->set_pars(p, i, d, s, n);
}

void RaMotor::fetch_upd() {
  fetchSensor();
  float z[] = {getPosition(), getSpeed()};
  kalman->pred_upd(RaMatrix(z, 2, 1), 0);
  // kalman->pred_upd(RaMatrix(z, 2, 1), U);
}

void RaMotor::PIDPos(float pos_r, float pos_m) {
  float u = PosPID->calc(pos_r, pos_m);
  PIDSpd(u, kalman->getSpeed());
}

void RaMotor::setPIDSpdPar(float p, float i, float d, float s, int n) {
  SpeedPID->set_pars(p, i, d, s, n);
}

void RaMotor::PIDSpd(float spd_r, float spd_m) {
  float u = SpeedPID->calc(spd_r, spd_m);
  setVoltage(u);
}

float RaMotor::getPosition() {
  float q = atan2(accelerometer_y, pos_sign * accelerometer_x);
  return q >= -PI/2 ? q : q + 2 * PI;
}

float RaMotor::getSpeed() {
  return spd_gain * gyro_z;
}

void RaMotor::stop() {
  analogWrite(pwm, 0);
}

#ifdef USE_HALL
RaMotor::RaMotor(short a1, short a2, short p, 
                 short m, float sg, float u0_,
                 short h, int *sb, void (*fun)(void))
                 :RaMotor::RaMotor(a1, a2, p, m, sg, u0_) {
  hall = h;
  spdBuff = sb;
  pinMode(hall, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hall), fun, RISING);
}

float RaMotor::getHallSpeed() {
  int spd_sum = 0;
  for(int i = 0; i < SPD_BUFF_LEN; i++) {
    spd_sum += spdBuff[i];
  }
  return spd_sum * 2 * PI * 1000.0 / (1.0 * SPD_BUFF_LEN) / (1.0 * SPD_MEASURE_RATE) / 110.0 / 2.0;
}
#endif