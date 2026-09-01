#include "pros/adi.hpp"
#include "pros/motors.h"


extern void distanceDriveFront(int target, int timein);
extern void distanceDriveBack(int target, int timein);
extern void upDistanceBack(int target, int timein);
extern float distanceconfigure;

#define DISTANCE_KP 0.7
#define DISTANCE_KI 0.00001
#define DISTANCE_KD 3.2

#define OFFSET 3