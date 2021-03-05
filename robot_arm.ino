#include <arduino-timer.h>

#include "ra_motor.h"
#include "ra_consts.h"
#include "ra_utils.h"
#include "ra_man.h"
#include "ra_kalman.h"
#include "ra_exp.h"


RaMotor *motor1 = NULL;
RaMotor *motor2 = NULL;

RaMan *man = NULL;


void setup() {
  motor1 = new RaMotor(M1_ARM1, M1_ARM2, M1_PWM, MPU_ADDR1, -1.f, 0.f); // 2.5
  motor2 = new RaMotor(M2_ARM1, M2_ARM2, M2_PWM, MPU_ADDR2, 1.f, 0.f); // 1
  
  // disable pull-up resistors for I2C
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  // set I2C speed to 400kHz 
  Wire.setClock(400000UL);

  // high speed RS-232
  Serial.begin(2000000UL);

  #ifdef USE_HALL
  // measure speed
  pulces1 = pulces2 = 0;
  timer.every(SPD_MEASURE_RATE, updSpeed);
  #endif

  // set PIDs parameters
  motor1->setPIDSpdPar(15.0f, 0.f, 0.0f, 0.0f, 0);
  motor1->setPIDPosPar(1.f, 1.f, 0.0f, 0.0f, 0); // 0.5f, 4.f 
  motor2->setPIDSpdPar(15.0f, 0.f, 0.0f, 0.0f, 0);
  motor2->setPIDPosPar(1.f, 1.f, 0.0f, 0.0f, 0);

  // manipulator object
  man = new RaMan(.065f, .065f, motor1, motor2);

  // set trajectory and move into init position
  man->choose_traj(2);
}

void loop() {
  man->coor_control();
  // man->US();

  if(stopFlag) {
    Serial.println("Program is stopped.");
    motor1->stop();
    motor2->stop();
    while(true);
  }
}
