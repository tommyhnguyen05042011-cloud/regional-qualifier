#include "subsystem.hpp"

inline int level = 0; /* keeps track of lift's current level */
inline bool hold_score = false; /* determine whether arm should stay
                                   at score or at load*/

inline bool loading = true; /* state of arm, loading (taking stacks from intake, only on lv0)
                               or scoring (parallel to ground to score)*/

inline bool scoring = false; /* is robot's next scoring system going to score or not
                                only happens higher than lv0*/

inline bool manual = false; /* state of control, manual or macroed.
                               when manual, driver will be able
                               to move the lift to whatever degree of movement he needs */

inline bool armIn = true;
inline bool armLower = true;
inline bool intake1Retract = true;
inline bool intake2Retract = true;
inline bool currentOpticalDetect = false;
inline bool lastOpticalDetect = false;
inline int liftTarget;
inline int armTarget;
inline int liftError;
inline int armError;
inline float liftkP = 0.45;
inline float armkP = 0.03;

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
            if (level > 0) {
                hold_score = armIn;
            }
            if (level == 0) {
                if (!armIn) {
                    hold_score = false;
                }
                if (hold_score && armIn) {
                    armTarget = 9000;
                } else {
                    armTarget = 0;
                }
                liftTarget = 0;
            } else if (level == 1) {
                armTarget = 9000;
                liftTarget = 1900;
            } else if (level == 2) {
                armTarget = 9000;
                liftTarget = 4500;
            } else if (level == 3) {
                armTarget = 9000;
                liftTarget  = 7100;
            }
            armError = armTarget - arm_rotation.get_position();
            liftError = liftTarget - lift_rotation.get_position()/100;

            arm.move(armError * armkP);
            lift.move(liftError * liftkP);
        }
		pros::delay(20);
	}
}

inline void lift_auto() {
    while (true) {
        liftError = liftTarget - lift_rotation.get_position()/100;
        armError = armTarget - arm_rotation.get_position();
        lift.move(liftError * liftkP);
        arm.move(armError * armkP);
        pros::delay(20);
    }
}

/* scoring command */
inline void score_stack() {
    if (level != 0) {
        pros::delay(200);
        arm.move((10000 - arm_rotation.get_position()) * 0.03);
        pros::delay(1000);
        armIn = false;
        scoring = false;
    }
}