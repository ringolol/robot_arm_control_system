#ifndef RA_EXP_H
#define RA_EXP_H

#include "ra_motor.h"
#include "ra_utils.h"

#ifdef USE_HALL
void freq_char(RaMotor *motor);
void min_voltage(RaMotor *motor);
#endif

#endif