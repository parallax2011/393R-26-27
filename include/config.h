#include "lemlib/api.hpp" // IWYU pragma: keep

extern pros::Controller cont;

extern pros::MotorGroup l;
extern pros::MotorGroup r;
extern pros::MotorGroup lift;
extern pros::Motor intake;
extern pros::adi::DigitalOut claw;
extern pros::adi::DigitalOut rotator;
extern pros::adi::DigitalOut pivoter;
extern lemlib::Drivetrain dt;

extern pros::Imu imu;
extern pros::Rotation rot;
extern pros::Rotation horizEnc;
extern pros::Rotation vertEnc;
extern lemlib::TrackingWheel horizTrackWheel;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateralCont;
extern lemlib::ControllerSettings angularCont;

extern lemlib::Chassis chassis;