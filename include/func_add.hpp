#include "pros/misc.h"
#include "subsystem.hpp"

inline int level = 0; /* keeps track of lift's current level */

inline bool loading = true; /* state of arm, loading (taking stacks from intake, only on lv0)
                               or scoring (parallel to ground)*/

inline bool scoring = false; /* is robot's next scoring system going to score or not
                                only happens higher than lv0*/

inline bool manual = false; /* state of control, manual or macroed.
                               when manual, driver will be able
                               to move the lift to whatever degree of movement he needs */

inline bool flipperState = false;
inline int liftTarget;
inline int liftError;
inline float liftkP = 0.02;

inline void macro_lift() {
	while (true) {
        if (!scoring && !manual) {
            if (level == 0) {
                liftTarget = -200;
            } else if (level == 1) {
                liftTarget = 26000;
            } else if (level == 2) {
                liftTarget = 47000;
            } else if (level == 3) {
                liftTarget = 72000;
            } else if (level == 4) {
                liftTarget = 94000;
            }
            
            liftError = liftTarget - lift_rotation.get_position();
            if (liftError * liftkP < -80) {
                lift.move(-80);
            } else {
                lift.move(liftError * liftkP);
            }
        }
		pros::delay(20);
	}
}

inline void macro_toggle() {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            while (optical.get_hue() > 25) {
                intake.move(-127);
            }
            pros::delay(100);
            intake.move(0);
        }
        pros::delay(20);
    }
}

inline void lift_auto() {
    while (true) {
        liftError = liftTarget - lift_rotation.get_position();
        if (liftError * liftkP < -80) {
            lift.move(-80);
        } else {
            lift.move(liftError * liftkP);
        }
        pros::delay(20);
    }
}