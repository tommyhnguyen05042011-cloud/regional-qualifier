#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// left motor group
inline pros::MotorGroup left_motor_group({-2, -3, -4}, pros::MotorGears::blue);
// right motor group
inline pros::MotorGroup right_motor_group({5, 6, 7}, pros::MotorGears::blue);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                                     &right_motor_group, // right motor group
                                     10.71, // 10 inch track width
                                     lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                                     450, // drivetrain rpm is 450
                                     2 // horizontal drift is 2 (for now)
);

// imu
inline pros::Imu imu(14);

// vertical tracking wheel
inline pros::Rotation vertical_encoder(1);
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.39);

// odometry settings
inline lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                                   nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                                   nullptr,
                                   nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                   &imu // inertial sensor
);

// lateral PID controller
inline lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                                     0, // integral gain (kI)
                                                     3, // derivative gain (kD)
                                                     0, // anti windup
                                                     0, // small error range, in inches
                                                     0, // small error range timeout, in milliseconds
                                                     0, // large error range, in inches
                                                     0, // large error range timeout, in milliseconds
                                                     0 // maximum acceleration (slew)
);

// angular PID controller
inline lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                                     0, // integral gain (kI)
                                                     20, // derivative gain (kD)
                                                     3, // anti windup
                                                     1, // small error range, in degrees
                                                     100, // small error range timeout, in milliseconds
                                                     3, // large error range, in degrees
                                                     500, // large error range timeout, in milliseconds
                                                     0 // maximum acceleration (slew)
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                               lateral_controller, // lateral PID settings
                               angular_controller, // angular PID settings
                               sensors // odometry sensors
);

inline pros::Motor intake(13);
inline pros::MotorGroup lift({11, -12});
inline pros::Motor claw(16);

inline pros::Rotation claw_rotation(15);
inline pros::Rotation lift_rotation(-17);
inline pros::Optical optical(8);

inline pros::ADIDigitalOut claw_piston('H');
inline pros::ADIDigitalOut intake_pistion_front('A');
inline pros::ADIDigitalOut intake_piston_back('B');