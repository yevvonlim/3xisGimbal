//2022-1 Semester Yonsei-RoboIN 3 Axis Gymbal Project. 
//@author: ga06033@yonsei.ac.kr
//         mango3354@naver.com
//@date: 2022-06-23

#include "dcmotor_control.h"
#include "mpu6050_wrapper.h"
// #include <Arduino.h>

double setPoint[3]={0,0,0};
extern MPU6050 mpu;
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
// void run_roll_motor(int pwm){
//     if (pwm >= 0){
//         digitalWrite(ROLL_1, HIGH);
//         digitalWrite(ROLL_2, LOW);
//         analogWrite(ROLL_PWM, pwm); 
//     }
//     else if (pwm < 0){
//         digitalWrite(ROLL_1, LOW);
//         digitalWrite(ROLL_2, HIGH);
//         analogWrite(ROLL_PWM, -pwm); 
//     }
//     return;
// }

void DELAY(uint16_t milsec){
    uint16_t st_time = millis();
    while(abs(millis()-st_time) < milsec) ;
    return;
}

void run_roll_motor(int pwm){
    // pwm: -255 ~ 255
    // if pwm < 0, run ROLL motor cw
    // if pwm > 0, run ROLL motor ccw
    if (pwm > 0){
        digitalWrite(ROLL_1, HIGH);
        digitalWrite(ROLL_2, LOW);
        analogWrite(ROLL_PWM, pwm); 
        // uint16_t run_motor_delay = RUN_MOTOR_DELAY/pwm;
        // Serial.print("DELAY: "); Serial.println(run_motor_delay);
        // uint16_t st_time = millis();
        
        // DELAY(run_motor_delay);
        // analogWrite(ROLL_PWM, 0); 
    }
    else if (pwm < 0){
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, -pwm); 
        // uint16_t run_motor_delay = RUN_MOTOR_DELAY/(-pwm);
        // Serial.print("DELAY: "); Serial.println(run_motor_delay);
        // DELAY(run_motor_delay);
        // analogWrite(ROLL_PWM, 0); 
    }
    else if (pwm == 0){
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, pwm); 
    }
    return ;
}

void run_roll_motor_delay(int pwm){
    // pwm: -255 ~ 255
    // if pwm < 0, run ROLL motor cw
    // if pwm > 0, run ROLL motor ccw
    if (pwm > 0){
        uint16_t delay_milsec = pwm;
        digitalWrite(ROLL_1, HIGH);
        digitalWrite(ROLL_2, LOW);
        analogWrite(ROLL_PWM, RUN_MOTOR_DELAY_PWM); 
        delay(delay_milsec);
        analogWrite(ROLL_PWM, 0); 
    }
    else if (pwm < 0){
        uint16_t delay_milsec = -pwm;
        digitalWrite(ROLL_1, LOW);
        digitalWrite(ROLL_2, HIGH);
        analogWrite(ROLL_PWM, RUN_MOTOR_DELAY_PWM); 
        delay(delay_milsec);
        analogWrite(ROLL_PWM, 0); 
    }
    return ;
}


// void run_roll_motor_dt(int pwm){
//     // pwm: -255 ~ 255
//     // if pwm < 0, run ROLL motor cw
//     // if pwm > 0, run ROLL motor ccw
//     if (pwm > 0){
//         digitalWrite(ROLL_1, HIGH);
//         digitalWrite(ROLL_2, LOW);
//         analogWrite(ROLL_PWM, pwm); 
//         delay(RUN_MOTOR_DT_DELAY);
//         analogWrite(ROLL_PWM, 0); 
//     }
//     else if (pwm < 0){
//         digitalWrite(ROLL_1, LOW);
//         digitalWrite(ROLL_2, HIGH);
//         analogWrite(ROLL_PWM, -pwm); 
//         delay(RUN_MOTOR_DT_DELAY);
//         analogWrite(ROLL_PWM, 0); 
//     }
//     return ;
// }

void get_min_delay(MOTOR_CLASS motor_type, double (*pwm_delay)[3]){
    /*
    Params]
        MOTOR_CLASS motor_type: motor type (YAW=0, PITCH=1, ROLL=2)
        double* pwm_delay: pwm_delay -> 이차배열 포인터. (pwm, delay) 순서쌍을 저장함.
    */
    float ypr[3] = {0,};
    int index = 0;
    for(int pwm=30; pwm <= 200; pwm+=10){
        for(int _delay=20; _delay <= 100; _delay+=10){
            get_ypr(ypr);
            mpu.resetFIFO();
            mpu.getIntStatus();
            double init_sensor_value = ypr[motor_type];

            run_roll_motor(pwm);
            delay(_delay);
            run_roll_motor(0);
            delay(500);
            mpu.resetFIFO();
            mpu.getIntStatus();
            
            get_ypr(ypr);
            mpu.resetFIFO();
            mpu.getIntStatus();

            double new_sensor_value = ypr[motor_type];
            run_roll_motor(-pwm);
            delay(_delay);
            run_roll_motor(0);
            /*
            if(abs(init_sensor_value-new_sensor_value) >= 5){
                pwm_delay[index][0] = pwm;
                pwm_delay[index][1] = _delay;
                pwm_delay[index][2] = abs(init_sensor_value-new_sensor_value);
                Serial.print("pwm: "); Serial.print(pwm); Serial.print(", ");Serial.print("delay: ");Serial.print(_delay);
                Serial.print("| DT_ANGLE:"); Serial.println(abs(init_sensor_value-new_sensor_value));
                break;
            }*/
            Serial.print("pwm: "); Serial.print(pwm); Serial.print(", ");Serial.print("delay: ");Serial.print(_delay);
            Serial.print("| DT_ANGLE:"); Serial.println(abs(init_sensor_value-new_sensor_value));
            delay(500);
            Serial.println("------------------------------------------------");
            mpu.resetFIFO();
            mpu.getIntStatus();
        }
        index++;
    }
}

void run_roll_motor_complex(int pwm){
    if (pwm > 0) {
        if (pwm < PWM_THR){
            run_roll_motor_delay(pwm);
        }
        else{
            run_roll_motor_dt(pwm);
        }
    }
    else if (pwm < 0) {
        if(pwm > -PWM_THR){
            run_roll_motor_delay(pwm);
        } 
        else{
            run_roll_motor_dt(pwm);
        }
    }
    return;
}
// qnpr
int computePID(volatile double inp, int i)
{     
        inp = inp > 30? 30 : inp;
        currentTime[i] = millis();                //get current time
        elapsedTime = (double)(currentTime[i] - previousTime[i]);        //compute time elapsed from previous computation
        
        error[i] = setPoint[i]-inp;                         // determine error
        cumError[i] += error[i] * elapsedTime;                // compute integral
        rateError[i] = (error[i] - lastError[i])/elapsedTime;   // compute derivative

        double out = kp*error[i] + ki*cumError[i] + kd * rateError[i];                //PID output               
        // Serial.print("out | "); Serial.println(out); Serial.println("");
        motor_spd[i]=(int)SDYfunc(out);
        
        lastError[i] = error[i];                                //remember current error
        previousTime[i] = currentTime[i];                        //remember current time
 
        return motor_spd[i];                                        //have function return the PID output
}
//출처: https://throwexception.tistory.com/851 [집밖은 위험해:티스토리]

double SDYfunc(double out){
    double thr = P_THR*kp + D_THR*kd + I_THR*ki;
    double spd=0;
    
    if (out < thr && out >= 0){
        // spd = MAX_PWM / thr * out;
        spd = (MAX_PWM / pow(thr, 3)) * pow(out-thr, 3) + MAX_PWM;
    }
    else if (out >= -thr && out < 0){
        // spd = MAX_PWM / thr * out;
        spd = (MAX_PWM / pow(thr, 3)) * pow(out+thr, 3) - MAX_PWM;
    }
    
    else if (out < -thr){
        spd = -MAX_PWM;
    }
    else{
        spd = MAX_PWM;
    }
    return spd;
}