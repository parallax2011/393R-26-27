#include "pros/adi.hpp"
#include "pros/motors.h"


extern void resetEncoders();
extern void setConstants(float kp, float ki, float kd);
extern void chasMove(int voltageLeft,  int voltageRight);
extern float calcPID(int target, float input, int integralKi, int maxIntegral);
extern float error;
extern float viewvol;


//Tuning

#define STRAIGHT_KP 1.5
#define STRAIGHT_KI 0.001
#define STRAIGHT_KD 10

#define TURN_KP 7.5
#define TURN_KI 0.0001
#define TURN_KD 80

// Other Settings

#define STRAIGHT_INTEGRAL_KI 430
#define STRAIGHT_MAX_INTEGRAL 2000

#define TURN_INTERGAL_KI 30
#define TURN_MAX_INTERGAL 25



