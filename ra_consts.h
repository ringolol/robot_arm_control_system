#ifndef RA_CONSTS_H
#define RA_CONSTS_H

// Motor 1
#define M1_HALL 2
// M1 H-bridge
#define M1_ARM1 7
#define M1_ARM2 8
// M1 PWM
#define M1_PWM 9

// Motor 2
#define M2_HALL 3
// M2 H-bridge
#define M2_ARM1 11 
#define M2_ARM2 12
// M2 PWM
#define M2_PWM 10

// PID
#define PID_WAIT_MIL 10

// gyro and acc sensors
#define MPU_ADDR1 0x69
#define MPU_ADDR2 0x68
#define ACC_RES 1000

#define VOL_TO_BIT 24.29 // 10.5V to 255 bit

#define INIT_SETTLING_TIME 10000

// hall sensor
// #define USE_HALL
// #define SPD_BUFF_LEN 0
// #define SPD_MEASURE_RATE 0

#define CIRCLE_XC .0f
#define CIRCLE_YC .09f
#define CIRCLE_R .03f
#define CIRCLE_VK .03f
#define CIRCLE_X0 .0f
#define CIRCLE_Y0 (CIRCLE_YC+CIRCLE_R)

#define LINE_K .3125f
#define LINE_B .09125f
#define LINE_X0 -.1f
#define LINE_Y0 .06f
#define LINE_VK .03f
#define LINE_XMAX 0.06f
#define LINE_XMIN -0.1f

#define PARA_A -4.f
#define PARA_XC 0.f
#define PARA_B .1f
#define PARA_VK 0.03f
#define PARA_X0 -0.1f
#define PARA_Y0 .06f
#define PARA_XMAX 0.1f
#define PARA_XMIN -0.1f

// coordinated contorl regulator parameters
#define COOR_REG_P 1.0f 
#define COOR_REG_I 10.0f
#define COOR_REG_D 0.0f
#define COOR_REG_N 0

#endif
