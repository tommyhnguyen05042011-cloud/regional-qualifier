#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "func_add.hpp"
#include <csignal>
#include <cstdio>
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

using namespace pros;
using namespace lemlib;

void initialize() {
	lcd::initialize();
	loading = true;
	// rotation reset
	lift_rotation.reset_position();
	// piston state
	intake_pistion_front.set_value(false);
	intake_piston_back.set_value(false);
	arm.set_value(false);

	lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	//chassis initilize
	chassis.calibrate();
	chassis.setPose(-58.85, -6.5, 270, false);

	Task([&] {
		while (true) {
			lcd::print(0, "X: %.3f", chassis.getPose().x);
            lcd::print(1, "Y: %.3f", chassis.getPose().y);
            lcd::print(2, "Theta: %.3f", chassis.getPose().theta);

			if (manual) {
				lcd::print(3, "Manual");
			} else {
				lcd::print(3, "Macro");
			}

			lcd::print(4, "Lift Level: %d", level);
			lcd::print(5, "Lift Rotation: %d", lift_rotation.get_position());

			lcd::print(6, "Left Drive Temp: %d", left_motor_group.get_temperature());
			lcd::print(7, "Right Drive Temp: %d", right_motor_group.get_temperature());

			delay(200);
    	}
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// start auton lift macro task
	Task auto_lift_task(lift_auto);

	// toggle
	chassis.tank(-60, -60);
	delay(400);
	chassis.tank(35, 35);
	delay(750);
	chassis.tank(0, 0);
	delay(200);
	chassis.tank(-40, -40);
	delay(200);

	// alliance goal 1
	chassis.moveToPose(-45, -20, 0, 2000, {.forwards = false});
	liftTarget = 500;
	arm.set_value(true);
	chassis.waitUntilDone();
	chassis.tank(-30, -30); // push into goal
	liftTarget = -1100;
	delay(1000);
	chassis.tank(0, 0);
	delay(500);
	delay(500);
	liftTarget = 1000;

	// stack up
	chassis.setPose(-47, -16.67, 0); // reset pose to minimise error
	intake.move(127);
	chassis.moveToPose(-37.7, -14, 83, 2000, {.minSpeed = 70, .earlyExitRange = 4}); // motion chain
	delay(500);
	arm.set_value(false);
	liftTarget = 0;
	chassis.moveToPose(-29.1, -19.7, 137, 1800);
	chassis.waitUntilDone();
	intake_piston_back.set_value(true);
	delay(1500);
	delay(200);

	// stack up
	liftTarget = 1900;
	arm.set_value(true);
	delay(200);
	intake_piston_back.set_value(false);
	chassis.moveToPose(-46, -22, 90, 2000, {.forwards = false});
	arm.set_value(true);
	chassis.waitUntilDone();
	chassis.tank(-50, -50);
	delay(500);
	chassis.cancelAllMotions();

	chassis.setPose(-40.17, -23.5, 90);
	delay(500);
	chassis.moveToPose(-37.1, -32.9, 188, 1500, {.minSpeed = 70, .earlyExitRange = 4});
	delay(200);
	liftTarget = 0;
	arm.set_value(false);
	chassis.moveToPose(-47, -46.5, 225, 1800);
	chassis.waitUntilDone();
	intake_piston_back.set_value(true);
	delay(1300);
	delay(300);
	intake_piston_back.set_value(false);

	liftTarget = 4500;
	arm.set_value(true);
	chassis.turnToPoint(-47, -26, 1500, {.forwards = false});
	chassis.waitUntilDone();
	chassis.moveToPose(-47, -26, 180, 2000, {.forwards = false});
	chassis.waitUntilDone();
	chassis.tank(-50, -50);
	delay(200);
	delay(800);

	// midfield
	chassis.moveToPoint(-41.9, -36, 1500, {.minSpeed = 70, .earlyExitRange = 4});
	delay(500);
	liftTarget = 0;
	chassis.moveToPose(-3.5, -5.6, 45, 2000);

	// exit auton lift macro task
	auto_lift_task.remove();
}

Controller master(E_CONTROLLER_MASTER);

void opcontrol() {

	Task macro_lift_task(macro_lift);
	Task macro_intake_task(macro_intake);
	manual = false;

	while (true) {
		// get left y and right x positions
        int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX, false, 0.5);

		// intake control
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake.move(127);
		} else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			intake.move(-127);
		} else {
			intake.move(0);
		}

		// intake pneumatics control
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			intake1Retract = !intake1Retract;
			intake2Retract = !intake2Retract;
		}
		intake_pistion_front.set_value(!intake1Retract); //flip both variable bc our pistons are flipped
		intake_piston_back.set_value(!intake2Retract);

		// claw control
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			claw.move(-127);
		} else {
			claw.move(127);
		}

		// manual toggle
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			manual = !manual;
		}

		// what to do if manual or not
		if (manual) {
			if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
				lift.move(127);
			} else if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
				lift.move(-80);
			} else {
				lift.move(0);
				lift.brake();
			}
		} else {
			if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
				scoring = false;
				if (level < 4) {
					level++;
				}
			} else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
				scoring = false;
				if (level > 0) {
					level--;
				}
			}
		}

		// arm control at lv0
		if (level == 0) {
			if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
				loading = !loading;
			}
			arm.set_value(!loading); /*in the macro lift, lv0 does not control the arm
									   and this will control it instead*/ 
		}

		//reseting lift
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP)) {
			scoring = false;
			level = 0;
		}

		delay(20);
	}
}