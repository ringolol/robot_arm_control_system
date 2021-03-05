#ifndef RA_UTILS_H
#define RA_UTILS_H

#include <Arduino.h>
#include <arduino-timer.h>
#include "ra_consts.h"


// PID step in seconds
extern const float PID_wait_sec;

// stop program flag
extern bool stopFlag; 

struct Point {
    float x;
    float y;

    Point(float x_, float y_) {
        x = x_;
        y = y_;
    }
};

// named data
struct Named {
    String name;
    float val;

    Named(String n, float v) {
        name = n;
        val = v;
    }
};

// do smthing while waiting
void wait(unsigned long del, void (*callback)(void));

// sign of a number
float sign(float x);

// print fields to serial, either separated by tabs or in json
String toSerial(Named *fields, int n, bool json=false);

#ifdef USE_HALL
// measure speed vars
extern int pulces1, pulces2;
extern short inxb;
extern int spdBuff1[SPD_BUFF_LEN];
extern int spdBuff2[SPD_BUFF_LEN];
extern Timer<> timer;

// measuring speed
void hallHandler1();
void hallHandler2();
bool updSpeed(void* argument);

// tick for spd timer
void tic();
#endif

#endif
