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

	lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	//chassis initilize
	chassis.calibrate();
	chassis.setPose(-58.8, -15.75, 310, false);

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
	// start macro tasks
	Task auto_lift_task(lift_auto);
}

void opcontrol() {
	Task macro_lift_task(macro_lift);
	// Task macro_toggle_task(macro_toggle);
	manual = false;

	while (true) {
		// get left y and right x positions
        int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX, false, 0.5);

		// intake control
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			if (flipperState) {
			} else {
				intake.move(127);
			}
		} else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			intake.move(-127);
		} else {
			intake.move(0);
		}

		// flipper control
		if (lift_rotation.get_position() < 20000 || level == 0) { /*if lift is lower than a certain height/level
																	then set to retracted position*/
			flipperState = false;
		} else {
			if (master.get_digital(E_CONTROLLER_DIGITAL_A)) { // else can be changed freely
				flipperState = !flipperState;
			}
		}
		flipper.set_value(flipperState);

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
		}

		//reseting lift
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP)) {
			scoring = false;
			level = 0;
		}

		delay(20);
	}
}