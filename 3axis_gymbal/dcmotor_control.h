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

// pid constant
#define kp 10
#define ki 0
#define kd 3

#define MAX_PWM 200

void roll_motor_attach();
void run_roll_motor_dt(int pwm);
void run_roll_motor_delay(int pwm);
void run_roll_motor_complex(int pwm);


void run_roll_motor(int pwm);
int computePID(volatile double inp, int i);
double SDYfunc(double out);

#endif