#include "config.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pid.h"
#include "distance.h"

using namespace pros;
using namespace std; 

float distanceconfigure = 0;



void distanceDriveBack(int target, int timein) {

    setConstants(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);

    int timeout = timein;

    const float stablecoder = backdist.get();
    float reversecoder;                                                                                                   
    float voltage2;
    float encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    int time = 0;

   
    cont.clear();
    // double error_range_time = 0;
    cont.print(0, 0, "ERROR: %f", error);


    resetEncoders();
   

    reversecoder = float(backdist.get());
    while(true) {
        encoderAvg = float(backdist.get());
        voltage2 = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(voltage2 > 127) {
            voltage2 = 127;
        } else if(voltage2 < -127) {
            voltage2 = -127;
        } else{
            voltage2 = voltage2;
        }


        if(init_heading > 180) {
            init_heading = (360 - init_heading);
        }

        if(imu.get_heading() < 180) {
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error = ((360 - imu.get_heading()) - init_heading);
        }

        
        heading_error *= 6;

        chasMove((voltage2 + heading_error), (voltage2 - heading_error));
        
       
        if (abs(target - encoderAvg) <= 2.5) count++;
        if (count >= 20){    
            break;
        }

        if (count >= 20 || time > timeout) {
            l.brake();
            r.brake();
    break;
        }



        delay(10);
    

        time += 10;
    }
    l.brake();
    r.brake();
}