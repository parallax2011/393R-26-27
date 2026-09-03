#include "pros/adi.hpp"
#include "pros/motors.h"



extern void distanceDriveBack(int target, int timein);

extern float distanceconfigure;

#define DISTANCE_KP 0.4
#define DISTANCE_KI 0.00001
#define DISTANCE_KD 2.4

#define OFFSET 3