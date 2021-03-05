#ifndef RA_MAN_H
#define RA_MAN_H

#include "ra_motor.h"
#include "ra_utils.h"
#include <Arduino.h>

class RaMan {
private:
    // kinematic parameters
    float l1, l2;
    // motors
    RaMotor *motor1, *motor2;

    // trajectory parameters
    float alf, da;
    short traj_type;

public:
    RaMan(float l1_, float l2_, RaMotor *m1, RaMotor *m2);
    Point inverseDynamics(Point p);
    void setPosition(Point p);
    void goto_init_pos(int tlim, float x0, float y0);
    void reset_regulators();

    void coor_control();

    void US();
    void circle_US();
    void line_US();
    void para_US();

    static float circle_ek(float xr, float yr);
    static float circle_alf(float xr, float yr);
    static float line_ek(float xr, float yr);
    static float line_alf(float xr, float yr);
    static float para_ek(float xr, float yr);
    static float para_alf(float xr, float yr);

    void choose_traj(short tr_type);
    void circle_setup();
    void line_setup();
    void para_setup();
    
};


#endif