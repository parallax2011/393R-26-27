// #include "main.h"
#include "config.h"
#include "pid.h"

// #include <iostream>


using namespace pros;
using namespace std; 



float vKp;
float vKi;
float vKd;
float error;
float viewvol;
float prevError;
int integral;
float power;

int derivative;


void setConstants(float kp, float ki, float kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
}

void chasMove(int voltageLeft,  int voltageRight) {
    l.move(voltageLeft);
    r.move(voltageRight);

}

void resetEncoders() {
    l.tare_position_all();
    r.tare_position_all();
}
 
float calcPID(int target, float input, int integralKi, int maxIntegral) { //basically tuning i here
    int integral;
    prevError = error;
    error = target - input;
   
    if(std::abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }


    if(integral >= 0) {
        integral = std::min(integral, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral = std::max(integral, -maxIntegral); //same thing but negative max
    }
   
    derivative = (prevError - error);


    // if (target < 0){
    //     derivative *= -1;
    // }


    power = (vKp * error) + (vKi * integral) - (vKd * derivative); //+ (vKd * derivative);

    return power;
}