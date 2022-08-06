//2022-1 Semester Yonsei-RoboIN 3 Axis Gymbal Project. 
//@author: ga06033@yonsei.ac.kr
//         mango3354@naver.com
//@date: 2022-06-23

#ifndef __DC_MOTOR_CONTROL__
#define __DC_MOTOR_CONTROL__

#define PITCH_1 8 
#define PITCH_2 9
#define PITCH_PWM 5

#define ROLL_1 4
#define ROLL_2 7
#define ROLL_PWM 3

#define YAW_1 10 
#define YAW_2 11
#define YAW_PWM 6

#define P_THR 180
#define I_THR 300
#define D_THR 3

#define PWM_THR 45
#define RUN_MOTOR_DELAY 2000
#define RUN_MOTOR_DELAY_PWM 55

// pid constant
#define kp 5
#define ki 0
#define kd 0

#define MAX_PWM 150
#include <Arduino.h>

enum MOTOR_CLASS {YAW=0, PITCH, ROLL};
void roll_motor_attach();
void run_roll_motor_dt(int pwm);
void run_roll_motor_delay(int pwm);
// void run_roll_motor_complex(int pwm);

void DELAY(uint16_t milsec);
void run_roll_motor(int pwm);
int computePID(volatile double inp, int i);
double SDYfunc(double out);
void get_min_delay(MOTOR_CLASS motor_type, double (*pwm_delay)[3]);

#endif
// or