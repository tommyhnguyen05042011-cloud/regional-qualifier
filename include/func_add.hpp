#include "subsystem.hpp"

inline int level = 0; /* keeps track of lift's current level */
inline bool hold_score = false; /* determine whether arm should stay
                                   at score or at load*/

inline bool loading = true; /* state of arm, loading (taking stacks from intake, only on lv0)
                               or scoring (parallel to ground)*/

inline bool scoring = false; /* is robot's next scoring system going to score or not
                                only happens higher than lv0*/

inline bool manual = false; /* state of control, manual or macroed.
                               when manual, driver will be able
                               to move the lift to whatever degree of movement he needs */

inline bool clawIn = true;
inline bool intake1Retract = true;
inline bool intake2Retract = true;
inline bool currentOpticalDetect = false;
inline bool lastOpticalDetect = false;
inline int liftTarget;
inline int liftError;
inline float liftkP = 0.02;

inline void macro_intake() {
    while (true) {
        currentOpticalDetect = (optical.get_proximity() > 50 && optical.get_proximity() < 255);

        if (!intake1Retract) {
            intake2Retract = false;
        } else if (currentOpticalDetect && !lastOpticalDetect) {
            intake2Retract = false;
            pros::delay(1500);
        } else if (!currentOpticalDetect) {
            intake2Retract = true;
        }

        lastOpticalDetect = currentOpticalDetect;
        pros::delay(20);
    }
}

inline void macro_lift() {
	while (true) {
        if (!scoring && !manual) {
            if (level == 0) {
                liftTarget = -200;
            } else if (level == 1) {
                loading = false;
                arm.set_value(true);
                liftTarget = 24000;
            } else if (level == 2) {
                arm.set_value(true);
                liftTarget = 45000;
            } else if (level == 3) {
                arm.set_value(true);
                liftTarget = 70000;
            } else if (level == 4) {
                arm.set_value(true);
                liftTarget = 92000;
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