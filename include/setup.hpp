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

extern pros::MotorGroup dr4bMech;
extern pros::Motor claw;

void dr4bHandle();

void openClaw();
void closeClaw();
void relaxClaw();
void controllerClawHandle();

enum class ClawState {
  OPEN,
  OPEN_FAILED,
  CLOSED,
  RELAXED
};

void setClawState(ClawState state);
ClawState getClawState();
void setClawAction(ClawState action);

extern void MotionViewAuton();
