#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"

pros::Controller cont(pros::E_CONTROLLER_MASTER);

//motors
pros::MotorGroup l({-1, -2, -3}, pros::MotorGears::blue);
pros::MotorGroup r({5, 9, 10}, pros::MotorGears::blue);
pros::MotorGroup lift({-16, 17}, pros::MotorGears::red);
pros::Motor intake(-6, pros::MotorGears::blue);

pros::adi::DigitalOut claw('F');
pros::adi::DigitalOut pivoter('G');
pros::adi::DigitalOut rotator('H');

// drivetrain settings
lemlib::Drivetrain dt(&l, // left motor group
                              &r, // right motor group
                              14.625, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(11); // imu
pros::Rotation rot(19); // lift rotation sensor
pros::Rotation horizEnc(20); // horizontal tracking wheel encoder
pros::Rotation vertEnc(21); // vertical tracking wheel encoder

// horizontal tracking wheel
lemlib::TrackingWheel horizTrackWheel(&horizEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertTrackWheel(&vertEnc, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertTrackWheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizTrackWheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateralCont(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularCont(6.5, // proportional gain (kP)
                                              0.0001, // integral gain (kI)
                                              32, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(dt, // drivetrain settings
                        lateralCont, // lateral PID settings
                        angularCont, // angular PID settings
                        sensors // odometry sensors
);