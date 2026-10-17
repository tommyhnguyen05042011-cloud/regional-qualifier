#include "subsystem.hpp"

inline int level = 0; /* keeps track of lift's current level */
inline bool hold_score = false; /* determine whether Claw should stay
                                   at score or at load*/

inline bool loading = true; /* state of claw, loading (taking stacks from intake, only on lv0)
                               or scoring (parallel to ground to score)*/

inline bool scoring = false; /* is robot's next scoring system going to score or not
                                only happens higher than lv0*/

inline bool manual = false; /* state of control, manual or macroed.
                               when manual, driver will be able
                               to move the lift to whatever degree of movement he needs */

inline bool clawClose = true;
inline bool clawLower = true;
inline bool intake1Retract = true;
inline bool intake2Retract = true;
inline bool currentOpticalDetect = false;
inline bool lastOpticalDetect = false;
inline int liftTarget;
inline int clawTarget;
inline int liftError;
inline int clawError;
inline float liftkP = 0.45;
inline float clawkP = 0.03;

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
                hold_score = clawClose;
            }
            if (level == 0) {
                if (!clawClose) {
                    hold_score = false;
                }
                if (hold_score && clawClose) {
                    clawTarget = 9000;
                } else {
                    clawTarget = 0;
                }
                liftTarget = 0;
            } else if (level == 1) {
                clawTarget = 9000;
                liftTarget = 1900;
            } else if (level == 2) {
                clawTarget = 9000;
                liftTarget = 4500;
            } else if (level == 3) {
                clawTarget = 9000;
                liftTarget  = 7100;
            }
            clawError = clawTarget - claw_rotation.get_position();
            liftError = liftTarget - lift_rotation.get_position()/100;

            claw.move(clawError * clawkP);
            lift.move(liftError * liftkP);
        }
		pros::delay(20);
	}
}

inline void lift_auto() {
    while (true) {
        liftError = liftTarget - lift_rotation.get_position()/100;
        clawError = clawTarget - claw_rotation.get_position();
        lift.move(liftError * liftkP);
        claw.move(clawError * clawkP);
        pros::delay(20);
    }
}

/* scoring command */
inline void score_stack() {
    if (level != 0) {
        claw_piston.set_value(true);
        pros::delay(200);
        claw.move((10000 - claw_rotation.get_position()) * 0.03);
        pros::delay(1000);
        clawClose = false;
        scoring = false;
    }
}