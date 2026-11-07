#pragma once

#include "main.h"
#include "drive.hpp"
#include "pros/adi.hpp"
#include "screen.hpp"
#include "lemlib/api.hpp"

#define MVLIB_USE_SIMPLES
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"

extern pros::MotorGroup leftDrivetrain;
extern pros::MotorGroup rightDrivetrain;

extern pros::Rotation horizontalOdom;
extern pros::Rotation verticalOdom;
extern pros::IMU imu1;
extern pros::IMU imu2;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Controller controller;

extern pros::Motor toggle;
void handleToggle();

extern pros::MotorGroup dr4bMech;
extern pros::Motor claw;
extern pros::Distance clawDistance;

void dr4bHandle();
void initializeLiftHeight();
void liftScoreMacro();

void openClaw();
void closeClaw();
void relaxClaw();
void controllerClawHandle();

enum class ClawState {
  OPEN,
  CLOSED,
  RELAXED
};

enum class LiftHeightSteps {
  MatchLoad, /* Height to pick up from match load */
  Floor = MatchLoad, /* Height to pick up from floor */
  AllianceGoalEmpty, /* Height to drop pins into alliance goal */
  NeutralGoalEmpty,  /* Height to drop pins into neutral goal */
  NeutralGoal1,      /* Height to drop pins into neutral/alliance goal with 1 pin */
  NeutralGoal2,      /* Height to drop pins into neutral/alliance goal with 2 pins */
  NeutralGoal3,      /* Height to drop pins into neutral/alliance goal with 3 pins */
  NeutralGoal4       /* Height to drop pins into neutral/alliance goal with 4 pins */
};

// Encoder degrees after initializeLiftHeight() homes and tares at the TOP.
constexpr float dr4bStartingHeight = -1160;
constexpr float dr4bMaxHeight = 0;

// Manual lift tuning. Slow down over the final 400 motor degrees of travel.
constexpr float dr4bSlowdownDistance = 400;
constexpr float dr4bRaiseMaxPower = 120;
constexpr float dr4bRaiseMinPower = 60;
constexpr float dr4bLowerPower = -80;
// Motor power units per second (0 disables slew for that direction).
// At the normal 10 ms loop, these allow changes of 4 up / 6 down.
constexpr float dr4bRaiseSlew = 400;
constexpr float dr4bLowerSlew = 600;

constexpr std::pair<float, LiftHeightSteps> dr4bHeightSteps[] = {
  { -1160, LiftHeightSteps::MatchLoad },
  { -1160, LiftHeightSteps::Floor },

  { -1050, LiftHeightSteps::AllianceGoalEmpty },
  { NULL, LiftHeightSteps::NeutralGoalEmpty },
  { -800, LiftHeightSteps::NeutralGoal1 },
  { -650, LiftHeightSteps::NeutralGoal2 },
  { -530, LiftHeightSteps::NeutralGoal3 },
  { -445, LiftHeightSteps::NeutralGoal4 }
};

// Lift is only ever lowered using PID, so only one is needed
extern lemlib::PID liftPID;

void setClawState(ClawState state);
ClawState getClawState();
void setClawAction(ClawState action);

extern void MotionViewAuton();
