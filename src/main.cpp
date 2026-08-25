#include "main.h"
#include "config.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

//using namespace pros;
//using namespace lemlib;

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    claw.set_value(true);
    rotator.set_value(false);
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
void autonomous() {}

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

void moveLift(float position) {
    float rotation_pos = rot.get_position();

    float error = position - std::abs(rotation_pos);
    float prevError = 0;
    int maxtime = 5000;
    int timeout = 0;
    while (std::abs(error) > 100 && timeout < maxtime) {
        rotation_pos = rot.get_position();
        float kp = 1;
        float kd = 6;
        float error = position - std::abs(rotation_pos);
        float deriv = error - prevError;
        float power = error*kp + deriv*kd;
        lift.move(power);
        timeout += 10;
        cont.clear();
        cont.print(1, 0, "Y: %f", error); // y

        prevError = error;
        pros::delay(10);
    }
    lift.set_brake_mode(pros::MotorBrake::hold);
    lift.brake();
}

bool cState = true;
bool rState = false;
bool pState = false;

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
        lift.move(-70);
    } else {
        lift.move(0);
        lift.set_brake_mode_all(pros::MotorBrake::hold);
    }

    //claw
    if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        cState = !cState;
        claw.set_value(cState);
    }
    //rotator
    if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        rState = !rState;
        rotator.set_value(rState);
    }
    //pivoter
    if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        pState = !pState;
        pivoter.set_value(pState);
    }

    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        moveLift(7000);
    }

    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        chassis.turnToHeading(90, 4000);
    }

    if(cont.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        chassis.moveToPoint(0, 30, 4000);
    }

}

void opcontrol() {
    rot.set_position(0);
	while (true) {
        controls();
        float rotation_pos = rot.get_position();
 
        cont.print(0, 0, "X: %f", chassis.getPose().x);
        cont.print(1, 0, "Y: %f", chassis.getPose().y);
        cont.print(2, 0, "Angle: %f", chassis.getPose().theta);
        cont.clear();
            // delay to save resources
        //pros::delay(1);
	}
}