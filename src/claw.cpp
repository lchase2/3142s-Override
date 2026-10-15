#include "./../include/setup.hpp"
#include <cstdlib>

namespace {
bool getMotorStalled(pros::Motor* motor, uint32_t mAThreshold = 2500,
                     float velocityThreshold = 5) {
  return (std::fabs(motor->get_actual_velocity()) < velocityThreshold &&
          motor->get_current_draw() > mAThreshold);
}
} // namespace

ClawState currentClawState = ClawState::CLOSED;
ClawState getClawState() {
  return currentClawState;  
}

void setClawState(ClawState state) {
  currentClawState = state;
}

// Position commands can initially use full torque to accelerate. Limit both
// their speed and current so a reversal cannot skip the claw's gears.
constexpr int kClawOpenVelocity = 5000;
constexpr int kClawOpenTimeout = 300;
constexpr int kClawOpenCurrentLimit = 1000;

constexpr int kClawStalledmA = 900;
constexpr int kClawStalledVel = 10;

constexpr int kClawCloseCurrentLimit = 2500;
constexpr int kClawCloseVelocity = 200;

constexpr int kClawKeepOpenPower = 30;

void openClaw() {
  auto& logger = mvlib::Logger::getInstance();
  claw.set_current_limit(kClawOpenCurrentLimit);

  uint32_t startTime = pros::millis();
  claw.move_velocity(-kClawOpenVelocity);
  
  while (!getMotorStalled(&claw, kClawStalledmA, kClawStalledVel) &&
         (pros::millis() - startTime) < kClawOpenTimeout) {
    // Wait until timedout or stalled (signifying end of path)
    pros::delay(15);
  }

  claw.set_current_limit(2500); // Unset current limit
  claw.move_voltage(-kClawKeepOpenPower);

  setClawState(ClawState::OPEN);
}

void closeClaw() {
  // Normal closing remains strong and continuous to retain objects.
  claw.set_current_limit(kClawCloseCurrentLimit);
  claw.move_velocity(kClawCloseVelocity);
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
    // Currently open, relax. Currently closed, open. Failed open, try again.
    setClawAction(currentState ==
          ClawState::OPEN ?
          ClawState::RELAXED :
            currentState == ClawState::OPEN_FAILED ?
            ClawState::OPEN :
            ClawState::OPEN);

  } else if (controller.get_digital_new_press(DIGITAL_L1)) {
    setClawAction(currentState == ClawState::CLOSED ? ClawState::RELAXED : ClawState::CLOSED);
  }
}
