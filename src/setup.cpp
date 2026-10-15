#include "setup.hpp"

// MotorGroups are {front, middle, back}
pros::MotorGroup leftDrivetrain(
  {-16, 1, 18},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::MotorGroup rightDrivetrain(
  {2, -11, -10},
  pros::v5::MotorGears::blue,
  pros::v5::MotorUnits::degrees
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// {right, left}
pros::MotorGroup dr4bMech({-4, -5});

pros::Motor claw(
  -20,
  pros::v5::MotorGears::green,
  pros::v5::MotorUnits::degrees
);

pros::Rotation horizontalOdom(-17);
pros::Rotation verticalOdom(12);
pros::IMU imu1(15);
pros::IMU imu2(16);


lemlib::TrackingWheel verticalTrackingWheel(
  &verticalOdom,
  lemlib::Omniwheel::NEW_2,
  0.915 // 0.915
);

lemlib::TrackingWheel horizontalTrackingWheel(
  &horizontalOdom,
  lemlib::Omniwheel::NEW_2,
  0.49 // 0.49
);

lemlib::OdomSensors sensors(
  &verticalTrackingWheel,
  nullptr,
  &horizontalTrackingWheel,
  nullptr,
  &imu1
);

const double trackWidth = 10.75;
const double wheelbase = 10.5;

lemlib::Drivetrain drivetrain(
  &leftDrivetrain,
  &rightDrivetrain,
  trackWidth,
  lemlib::Omniwheel::NEW_275,
  24.0/36.0 * 600,
  2
);

lemlib::ControllerSettings lateralPID(
  11.75,  // proportional gain (kP)
  0,   // integral gain (kI)
  8,   // derivative gain (kD)
  3,         // anti windup
  1,          // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,          // large error range, in inches
  500, // large error range timeout, in milliseconds
  0                 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularPID(
  3.5,   // proportional gain (kP)
  0.0,   // integral gain (kI)
  20,  // derivative gain (kD)
  3,         // anti windup
  1,          // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,          // large error range, in inches
  500, // large error range timeout, in milliseconds
  0                // maximum acceleration (slew)
);

lemlib::Chassis chassis(
  drivetrain,
  lateralPID,
  angularPID,
  sensors
);
