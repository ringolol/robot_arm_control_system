#include "ra_man.h"

RaMan::RaMan(float l1_, float l2_, RaMotor *m1, RaMotor *m2)
{
    l1 = l1_;
    l2 = l2_;
    motor1 = m1;
    motor2 = m2;
}

Point RaMan::inverseDynamics(Point p)
{
    float q2 = acos((sq(p.x) + sq(p.y) - (sq(l1) + sq(l2))) / (2 * l1 * l2));
    float q1 = atan2(p.y, p.x) - atan2(l2 * sin(q2), l1 + l2 * cos(q2));
    return Point(q1, q2);
}

void RaMan::setPosition(Point p)
{
    Point q = inverseDynamics(p);

    motor1->fetch_upd();
    float q1 = motor1->kalman->getPos();
    motor1->PIDPos(q.x, q1);

    motor2->fetch_upd();
    float q12 = motor2->kalman->getPos();
    motor2->PIDPos(q.x + q.y, q12);
}

void RaMan::coor_control()
{
    static RaPID EkPID(COOR_REG_P, COOR_REG_I, COOR_REG_D, 0.0f, COOR_REG_N);
    motor1->fetch_upd();
    motor2->fetch_upd();

    float q1 = motor1->kalman->getPos();
    float q12 = motor2->kalman->getPos();
    float dq1 = motor1->kalman->getSpeed();
    float dq12 = motor2->kalman->getSpeed();

    float c1 = cos(q1), c12 = cos(q12);
    float s1 = sin(q1), s12 = sin(q12);
    float xr = l1 * c1 + l2 * c12;
    float yr = l1 * s1 + l2 * s12;

    // stop for line
    if(traj_type == 0 && xr > LINE_XMAX) {
        stopFlag = true;
        return;
    } else if(traj_type == 2 && xr > PARA_XMAX) {
        stopFlag = true;
        return;
    }

    float ek, alf, vk;
    switch (traj_type)
    {
    default:
    case 0:
        ek = line_ek(xr, yr);
        alf = line_alf(xr, yr);
        vk = LINE_VK;
        break;
    case 1:
        ek = circle_ek(xr, yr);
        alf = circle_alf(xr, yr);
        vk = CIRCLE_VK;
        break;
    case 2:
        ek = para_ek(xr, yr);
        alf = para_alf(xr, yr);
        vk = PARA_VK;
        break;
    }
    ek = EkPID.calc(ek, 0.f);
    
    float ekx = -ek * sin(alf), eky = ek * cos(alf);
    float vtx = vk * cos(alf), vty = vk * sin(alf);

    float dxz = vtx - ekx;
    float dyz = vty - eky;

    float dq1z = -(dxz * c12 + dyz * s12) / (l1 * c12 * s1 - l1 * s12 * c1);
    float dq2z = (dxz * l1 * c1 + dyz * l1 * s1 + dxz * l2 * c12 + dyz * l2 * s12) / (l1 * c12 * s1 - l1 * s12 * c1) / l2;

    motor1->PIDSpd(dq1z, dq1);
    motor2->PIDSpd(dq1z + dq2z, dq12);

    Named fields[] = {
        Named("time", millis()),
        Named("dx*", dxz),
        Named("dy*", dyz),
        Named("q1", motor1->kalman->getPos()),
        Named("q12", motor2->kalman->getPos()),
        Named("dq1", motor1->kalman->getSpeed()),
        Named("dq12", motor2->kalman->getSpeed())};
    Serial.println(toSerial(fields, 7, true));
}

void RaMan::US() {
    switch (traj_type)
    {
    default:
    case 0:
        line_US();
        break;
    case 1:
        circle_US();
        break;
    case 2:
        para_US();
        break;
    }
}

void RaMan::line_US()
{
    static unsigned long t0 = millis();
    unsigned long tnew = millis();
    float xz = LINE_VK * cos(alf) * (tnew - t0) / 1000.0f + LINE_X0;
    float yz = LINE_K * xz + LINE_B;
    if(xz > LINE_XMAX || xz < LINE_XMIN) {
        stopFlag = true;
        return;
    }
    setPosition(Point(xz, yz));

    Named fields[] = {
        Named("time", millis()),
        Named("x*", xz),
        Named("y*", yz),
        Named("q1", motor1->kalman->getPos()),
        Named("q12", motor2->kalman->getPos()),
        Named("dq1", motor1->kalman->getSpeed()),
        Named("dq12", motor2->kalman->getSpeed())};
    Serial.println(toSerial(fields, 7, true));

    delay(PID_WAIT_MIL);
}

void RaMan::circle_US()
{
    static unsigned long tnew = millis();
    unsigned long told = tnew;
    tnew = millis();
    alf += da * (tnew - told) / 1000.0f;
    float xz = CIRCLE_R * cos(alf) + CIRCLE_XC;
    float yz = CIRCLE_R * sin(alf) + CIRCLE_YC;
    setPosition(Point(xz, yz));

    Named fields[] = {
        Named("time", millis()),
        Named("x*", xz),
        Named("y*", yz),
        Named("q1", motor1->kalman->getPos()),
        Named("q12", motor2->kalman->getPos()),
        Named("dq1", motor1->kalman->getSpeed()),
        Named("dq12", motor2->kalman->getSpeed())};
    Serial.println(toSerial(fields, 7, true));

    delay(PID_WAIT_MIL);
}

void RaMan::para_US()
{
    static unsigned long tnew = millis();
    unsigned long told = tnew;
    tnew = millis();
    
    static float xz = PARA_X0;
    xz += PARA_VK * (tnew - told) / 1000.0f / sqrt(4 * sq(PARA_A) * sq(xz) + 1);
    float yz = PARA_A * sq(xz - PARA_XC) + PARA_B;
    if(xz > PARA_XMAX || xz < PARA_XMIN) {
        stopFlag = true;
        return;
    }
    setPosition(Point(xz, yz));

    Named fields[] = {
        Named("time", millis()),
        Named("x*", xz),
        Named("y*", yz),
        Named("q1", motor1->kalman->getPos()),
        Named("q12", motor2->kalman->getPos()),
        Named("dq1", motor1->kalman->getSpeed()),
        Named("dq12", motor2->kalman->getSpeed())};
    Serial.println(toSerial(fields, 7, true));

    delay(PID_WAIT_MIL);
}

void RaMan::goto_init_pos(int tlim, float x0, float y0)
{
    // goto start position
    int start = millis();
    while (millis() - start < tlim)
    {
        setPosition(Point(x0, y0));
        delay(PID_WAIT_MIL);
    }
    reset_regulators();
}

void RaMan::reset_regulators()
{
    // prepare regulators
    motor1->PosPID->reset();
    motor1->SpeedPID->reset();
    motor2->PosPID->reset();
    motor2->SpeedPID->reset();
}

float RaMan::circle_ek(float xr, float yr)
{
    return sq(xr - CIRCLE_XC) + sq(yr - CIRCLE_YC) - sq(CIRCLE_R);
}

float RaMan::circle_alf(float xr, float yr)
{
    return atan2(yr - CIRCLE_YC, xr - CIRCLE_XC) - PI / 2;
}

float RaMan::line_ek(float xr, float yr)
{
    return (yr - (LINE_K * xr + LINE_B)) / sqrt(1 + sq(LINE_K));
}

float RaMan::line_alf(float xr, float yr)
{
    return atan(LINE_K);
}

float RaMan::para_ek(float xr, float yr) {
    return yr - (PARA_A * sq(xr - PARA_XC) + PARA_B);
}

float RaMan::para_alf(float xr, float yr) {
    return atan(2 * PARA_A * (xr - PARA_XC));
}

void RaMan::choose_traj(short tr_type) {
    traj_type = tr_type;
    switch (traj_type)
    {
    default:
    case 0:
        line_setup();
        break;
    case 1:
        circle_setup();
        break;
    case 2:
        para_setup();
        break;
    }
}

void RaMan::circle_setup()
{
    da = -CIRCLE_VK / CIRCLE_R;
    alf = atan2(CIRCLE_Y0 - CIRCLE_YC, CIRCLE_X0 - CIRCLE_XC);
    goto_init_pos(INIT_SETTLING_TIME, CIRCLE_X0, CIRCLE_Y0);
}

void RaMan::line_setup()
{
    alf = atan(LINE_K);
    goto_init_pos(INIT_SETTLING_TIME, LINE_X0, LINE_Y0);
}

void RaMan::para_setup() {
    // ...
    goto_init_pos(INIT_SETTLING_TIME, PARA_X0, PARA_Y0);
}