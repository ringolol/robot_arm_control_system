#include "ra_exp.h"

#ifdef USE_HALL
void freq_char(RaMotor *motor) {
  float ws[] = {2.0, 4.0, 16.0, 32.0, 64.0, 128.0, 256.0};
  long tlim = 2*PI*1000;
  for(auto w: ws) {
    long t1 = millis();
    long dt = 0;
    while(dt < tlim) {
      long t2 = millis();
      dt = t2 - t1;
      int u = 10.5 * sin(w * dt / 1000.0);
      motor->setVoltage(u);
      motor->fetchSensor();
      Named fields[] = {
        Named("w", w), 
        Named("t", t2), 
        Named("u", u), 
        Named("spd_hall", motor->getHallSpeed()),
        Named("spd", motor->getSpeed()), 
      };
      Serial.println(toSerial(fields, 5, true));
      wait(PID_WAIT_MIL, tic);
    }
    motor->setVoltage(0);
    wait(2000, tic);
  }
  stopFlag = true;
}

void min_voltage(RaMotor *motor) {
  for(float u = 0.0; u <= 10.5; u += 0.5) {
    motor->setVoltage(u);
    long t1 = millis();
    long dt = 0;
    while(dt < 3000) {
      long t2 = millis();
      dt = t2 - t1;
      Named fields[] = {
        Named("u", u), 
        Named("t", t2), 
        Named("spd_hall", motor->getHallSpeed()),
      };
      Serial.println(toSerial(fields, 3, true));
      wait(PID_WAIT_MIL, tic);
    }
  }
  stopFlag = true;
}

void backlash(RaMotor *motor1, RaMotor *motor2) {
  motor1->fetch_upd();
  motor2->fetch_upd();
  Serial.print(motor1->getPosition(), 2);
  Serial.print('\t');
  Serial.println(motor2->getPosition());
}
#endif