#include "main.h"
#include "config.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include <cstdint>

//using namespace pros;
//using namespace lemlib;

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    claw.set_value(false);
    pivoter.set_value(true);

    // chassis.setPose(-70, 0, 270);

    //rotator.set_value(false);
    // print position to brain screen
    // pros::Task screen_task([&]() {
    //     while (true) {
    //         //print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // // delay to save resources
    //         pros::delay(20);
    //     }
    // });
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void moveLift(float target) {
    lift.set_brake_mode(pros::MotorBrake::hold);
    float pos = rot.get_position();
    float error = target - std::abs(pos);
    float deriv = 0;
    float prevError = 0;
    int timeRunning = 0; int timeout = 5000;
    float kp = 1; float kd = 6;
    while (std::abs(error) > 100 and timeRunning < timeout) {
        pos = rot.get_position();
        error = target - std::abs(pos);
        deriv = error - prevError;
        float power = kp * error + kd * deriv;
        lift.move(power);
        //cont.clear();
        //cont.print(1, 0, "Y: %f", error); // y
        timeRunning += 10;
        prevError = error;
        pros::delay(10);
    }
    lift.brake();
}

void telemetry() {
    cont.print(0, 0, "X: %f", chassis.getPose().x);
    cont.print(1, 0, "Y: %f", chassis.getPose().y);
    cont.print(2, 0, "Theta: %f", chassis.getPose().theta);
}

void test() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -8, 4000, {.forwards = false} );
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 3000, 
                           {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    // chassis.setPose(0, 0, 90);
    // cont.print(0, 0, "X: %f", chassis.);
    // cont.print(1, 0, "Y: %f", chassis.getPose().y);
    // cont.print(2, 0, "Theta: %f", chassis.getPose().theta);
    chassis.moveToPoint(-10, -12, 4000, {.forwards = false});

    // lemlib::Pose pose = chassis.getPose();
    // // pros::lcd::print(0, "X: %f", pose.x);
    // // pros::lcd::print(1, "Y: %f", pose.y);
    // // pros::lcd::print(2, "Theta: %f", pose.theta);


    // chassis.swingToHeading(60, lemlib::DriveSide::LEFT, 3000, 
    //                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE} );
    // moveLift(2000);
    // chassis.moveToPoint(-4, -2.5, 3000, {.forwards = false, .maxSpeed = 50});
}

void skills_auton() {
    claw.set_value(false);
    chassis.setPose(-70, 0, 270);
    chassis.moveToPoint(-64, 0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 100});
    chassis.moveToPoint(-69, 0, 1000, {.maxSpeed = 100, .minSpeed = 100});
    chassis.moveToPoint(-64, 0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 100});
    chassis.moveToPoint(-69, 0, 1000, {.maxSpeed = 100, .minSpeed = 100});

    chassis.moveToPoint(-50, 0, 3000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.turnToHeading(0, 1500, {}, false);
    moveLift(-700);
    pivoter.set_value(false);
    pros::delay(999999);
}

void autonomous() {

    int auton = 1;

    if(auton == 0) test();
    if(auton == 1) skills_auton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



bool cState = false;
bool pState = false;
bool detect = true;
int distToClamp = 80.0;

void controls() {
    //chassis
    int forward = cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(forward, turn, false);

    //intake
    if (cont.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.move(127);
    else if (cont.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.move(-127);
    else intake.move(0);
    
    //lift
    if (cont.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        lift.move(127);
    } else if (cont.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        lift.move(-85);
    } else {
        lift.move(0);
        lift.set_brake_mode_all(pros::MotorBrake::hold);
    }

    //claw
    if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        cState = !cState;
        claw.set_value(cState);
        if (cState) { //if opened claw
            detect = false;
        } else if (!cState) { //if closed claw
            detect = false;
        }
    }
    //detection window is open and claw is open
    if (!detect and cState and dist.get() > distToClamp) {
        detect = true;
    }
    if (detect and dist.get() < distToClamp) {
        claw.set_value(false); //close claw once object < 50mm
        detect = false;
        cState = false;
    }

    //pivoter
    if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        pState = !pState;
        pivoter.set_value(pState);
    }

    // pid tuning buttons
    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        // moveLift(7000);
        claw.set_value(1);
    }
    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        chassis.turnToHeading(90, 4000);
    }
    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        chassis.moveToPoint(0, 30, 4000);
    }

    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        skills_auton();
    }
}

void opcontrol() {
    rot.set_position(0);
    chassis.setPose(0, 0, 0);

	while (true) {
        controls();
        float rotation_pos = rot.get_position();
         
        cont.print(0, 0, "X: %f", chassis.getPose().x);
        cont.print(1, 0, "Y: %f", chassis.getPose().y);
        cont.print(2, 0, "T: %f", rotation_pos);

        cont.clear();
            // delay to save resources
        // pros::delay(1);
	}
}