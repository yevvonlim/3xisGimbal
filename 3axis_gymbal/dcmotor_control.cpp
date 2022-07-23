//2022-1 Semester Yonsei-RoboIN 3 Axis Gymbal Project. 
//@author: ga06033@yonsei.ac.kr
//         mango3354@naver.com
//@date: 2022-06-23

#include "dcmotor_control.h"
#include <Arduino.h>

double setPoint[3]={0,0,0};

unsigned long currentTime[3]={0,0,0}, previousTime[3]={0,0,0};
double elapsedTime;

double error[3]={0,0,0};
double lastError[3]={0,0,0};
double input[3]={0,0,0}, output;
double cumError[3]={0,0,0}, rateError[3]={0,0,0};
double degree=30;
int motor_spd[3]={0,0,0};


void roll_motor_attach(){
    pinMode(ROLL_1, OUTPUT);
    pinMode(ROLL_2, OUTPUT);
    pinMode(ROLL_PWM, OUTPUT);
}
void run_roll_motor(int pwm){
    if (pwm > 0){
        digitalWrite(ROLL_1, HIGH);
        digitalWrite(ROLL_2, LOW);
        analogWrite(ROLL_PWM, pwm); 
    }
    else if (pwm < 0){
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, -pwm); 
    }
    return;
}

void run_roll_motor_dt(int pwm){
    // pwm: -255 ~ 255
    // if pwm < 0, run ROLL motor cw
    // if pwm > 0, run ROLL motor ccw
    if (pwm > 0){
        digitalWrite(ROLL_1, HIGH);
        digitalWrite(ROLL_2, LOW);
        analogWrite(ROLL_PWM, pwm); 
        delay(5);
        analogWrite(ROLL_PWM, 0); 
    }
    else if (pwm < 0){
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, -pwm); 
        delay(5);
        analogWrite(ROLL_PWM, 0); 
    }
    return ;
}

void run_roll_motor_delay(int pwm){
    // pwm: -255 ~ 255
    // if pwm < 0, run ROLL motor cw
    // if pwm > 0, run ROLL motor ccw
    if (pwm > 0){
        uint16_t delay_milsec = pwm*1.5;
        digitalWrite(ROLL_1, HIGH);
        digitalWrite(ROLL_2, LOW);
        analogWrite(ROLL_PWM, 55); 
        delay(delay_milsec);
        analogWrite(ROLL_PWM, 0); 
    }
    else if (pwm < 0){
        uint16_t delay_milsec = -pwm*1.5;
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, 55); 
        delay(delay_milsec);
        analogWrite(ROLL_PWM, 0); 
    }
    return ;
}

void run_roll_motor_complex(int pwm){
    if (pwm > 0) {
        if (pwm < 45){
            run_roll_motor_delay(pwm);
        }
        else{
            run_roll_motor_dt(pwm);
        }
    }
    else if (pwm < 0) {
        if(pwm > -45){
            run_roll_motor_delay(pwm);
        } 
        else{
            run_roll_motor_dt(pwm);
        }
    }
    return;
}

int computePID(volatile double inp, int i)
{     
        currentTime[i] = millis();                //get current time
        elapsedTime = (double)(currentTime[i] - previousTime[i]);        //compute time elapsed from previous computation
        
        error[i] = setPoint[i]-inp;                         // determine error
        cumError[i] += error[i] * elapsedTime;                // compute integral
        rateError[i] = (error[i] - lastError[i])/elapsedTime;   // compute derivative

        double out = kp*error[i] + ki*cumError[i] + kd * rateError[i];                //PID output               
        Serial.print("out | "); Serial.println(out); Serial.println("");
        motor_spd[i]=(int)SDYfunc(out);
        
        lastError[i] = error[i];                                //remember current error
        previousTime[i] = currentTime[i];                        //remember current time
 
        return motor_spd[i];                                        //have function return the PID output
}
//출처: https://throwexception.tistory.com/851 [집밖은 위험해:티스토리]

double SDYfunc(double out){
    double thr = 180*kp + 80*kd + 450*ki;
    double spd=0;
    
    if (out < thr && out >= 0){
        spd = MAX_PWM / thr * out;
    }
    else if (out >= -thr && out < 0){
        spd = MAX_PWM / thr * out;
    }
    
    else if (out < -thr){
        spd = -MAX_PWM;
    }
    else{
        spd = MAX_PWM;
    }
    return spd;
}