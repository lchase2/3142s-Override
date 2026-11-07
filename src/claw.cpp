#include "./../include/setup.hpp"
#include "pros/rtos.h"
#include <cstdlib>

ClawState currentClawState = ClawState::CLOSED;
ClawState getClawState() {
  return currentClawState;  
}

void setClawState(ClawState state) {
  currentClawState = state;
}

constexpr int kClawOpenVelocity = 200;
constexpr int kClawKeepOpenPower = 2000;

constexpr int kClawCloseVoltage = 12000;

void openClaw() {
  auto& logger = mvlib::Logger::getInstance();

  claw.move_velocity(-kClawOpenVelocity);
  pros::delay(150);
  claw.move_voltage(-kClawKeepOpenPower);
  
  setClawState(ClawState::OPEN);
}

void closeClaw() {
  // Normal closing remains strong and continuous to retain objects.
  claw.move_voltage(kClawCloseVoltage);
  setClawState(ClawState::CLOSED);
}

void relaxClaw() {
  claw.move_velocity(0);
  setClawState(ClawState::RELAXED);
}

void setClawAction(ClawState action) {
  switch (action) {
    case ClawState::OPEN:
      openClaw();
      break;
    case ClawState::CLOSED:
      closeClaw();
      break;
    case ClawState::RELAXED:
      relaxClaw();
      break;
    default:
      break;
  }
}

void controllerClawHandle() {
  ClawState currentState = getClawState();

  if (controller.get_digital_new_press(DIGITAL_L2)) {
    if (getClawState() == ClawState::CLOSED) {
      setClawAction(ClawState::OPEN);
      return;
    }
    setClawAction(ClawState::OPEN);
        
    pros::Task closeClawTask{[=] {
      while (controller.get_digital(DIGITAL_L2)) pros::delay(10);
      setClawAction(ClawState::CLOSED);
    }};

  } else if (controller.get_digital_new_press(DIGITAL_L1)) {
    setClawAction(currentState == ClawState::CLOSED ? ClawState::RELAXED : ClawState::CLOSED);
  }
}
