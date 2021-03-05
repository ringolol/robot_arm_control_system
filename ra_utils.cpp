#include "ra_utils.h"

// PID
const float PID_wait_sec = PID_WAIT_MIL / 1000.0; 

bool stopFlag = false;

// do smthing while waiting
void wait(unsigned long del, void (*callback)(void)) {
  unsigned long t0 = millis();
  while(millis() - t0 < del) {
    if(callback) callback();
  }
}

float sign(float x) {
  return x >= 0 ? 1.0 : -1.0;
}

String toSerial(Named *fields, int n, bool json) {
  char buff[128];
  String s = "";
  if(json) s += "{";
  for(int i = 0; i < n; i++) {
    if(json) s += "\"" + fields[i].name + "\":";
    s += dtostrf(fields[i].val, -1, 4, buff);
    if(json) s += i < n - 1 ? "," : "";
    else s += '\t';
  }
  if(json) s += "}";
  return s;
}

#ifdef USE_HALL
measure speed vars
int pulces1 = 0, pulces2 = 0;
short inxb = 0;
int spdBuff1[SPD_BUFF_LEN];
int spdBuff2[SPD_BUFF_LEN];

Timer<> timer = timer_create_default();

void hallHandler1() {
  pulces1++;
}

void hallHandler2() {
  pulces2++;
}

// handler for timer, which upd speeds
bool updSpeed(void* argument) {
  if(inxb >= SPD_BUFF_LEN) {
    inxb = 0;
  }
  spdBuff1[inxb] = pulces1;
  pulces1 = 0;
  spdBuff2[inxb] = pulces2;
  pulces2 = 0;
  inxb += 1;
  return true;
}

void tic() {
  timer.tick();
}
#endif