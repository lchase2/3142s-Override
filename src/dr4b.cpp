#include "./../include/setup.hpp"
#include "pros/motors.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>

namespace {
float dr4bManualPower = 0;
uint32_t dr4bLastUpdateMs = 0;
bool dr4bManualActive = false;

float slewLiftPower(float target, float elapsedSeconds) {
  // Stop immediately on release; never let the ramp defeat a lower height cap.
  if (target == 0) return dr4bManualPower = 0;

  const bool reversing = dr4bManualPower * target < 0;
  const float direction = reversing ? dr4bManualPower : target;
  const float rate = direction > 0 ? dr4bRaiseSlew : dr4bLowerSlew;
  const float step = rate <= 0 ? 254.0f : rate * elapsedSeconds;
  // On a reversal, ramp to zero before accelerating in the other direction.
  const float nextTarget = reversing ? 0 : target;
  dr4bManualPower += std::clamp(nextTarget - dr4bManualPower, -step, step);
  if (!reversing && target > 0) dr4bManualPower = std::min(dr4bManualPower, target);
  if (!reversing && target < 0) dr4bManualPower = std::max(dr4bManualPower, target);
  return dr4bManualPower;
}

std::optional<uint32_t> lastScoreMs;
constexpr uint32_t kScoreCooldownMs = 200; // Debounce between successful scores
constexpr double kScoreMinLiftHeight = -1075; // Min height for macro to engage

constexpr int kDistanceReadingMax = 180; // Highest valid reading to detect pins
constexpr int kDistanceReadingReleaseMax = 45; // Highest claw open reading
constexpr int kDistanceReadingReleaseMin = 15; // Lowest claw open reading
constexpr int kDistanceReadingReleaseTarget =
    (kDistanceReadingReleaseMax - kDistanceReadingReleaseMin) /
    2;                                  // Target open reading
constexpr int kDistanceReadingMin = 10; // Lowest valid reading to detect pins
constexpr int kDistanceNoDetect = 9999; // No object detected (from PROS)

// Update the lift once and report whether it is in position to release the score.
bool updateScoreLift() {
  static auto& logger = mvlib::Logger::getInstance();

  if (lastScoreMs) {
    if (pros::millis() - *lastScoreMs < kScoreCooldownMs) return false;
    lastScoreMs.reset();
  }

  const auto sensorValue = clawDistance.get_distance();
  const double liftHeight = (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2;
  // A nearby reading at the bottom of travel can look like a scoring target.
  if (liftHeight < kScoreMinLiftHeight && sensorValue >= 1 && sensorValue <= kDistanceReadingMax) {
    dr4bMech.move_velocity(0);
    return false;
  }

  bool liftWithinRange = sensorValue > kDistanceReadingMin &&
                         sensorValue < kDistanceReadingMax &&
                         sensorValue != kDistanceNoDetect;

  bool liftWithinReleaseRange = sensorValue > kDistanceReadingReleaseMin &&
                                sensorValue < kDistanceReadingReleaseMax &&
                                liftWithinRange;
  if (liftWithinReleaseRange) {
    // Hold the lift in place while the claw releases the score.
    dr4bMech.move_velocity(0);
    dr4bMech.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    return true;
  }

  if (sensorValue < kDistanceReadingMax) {
    // Lift is in detectable range of target, slow down if overspeed
    dr4bMech.move_velocity(dr4bMech.get_actual_velocity() > 0 ? -10 : 20);
    logger.info("Score macro in range, slowing down");
    return false;
  }

  if (!liftWithinRange && sensorValue > kDistanceReadingMax) {
    // Lift is above target, lower
    dr4bMech.move_velocity(-40);
    logger.info("Score macro lowering fullspeed");
    return false;
  }

  dr4bMech.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  return false;
}
} // namespace

void liftScoreMacro(uint32_t timeoutMs) {
  if (timeoutMs == 0 && !controller.get_digital(DIGITAL_A)) {
    dr4bMech.move_velocity(0);
    return;
  }

  const auto start = pros::millis();
  if (timeoutMs != 0) dr4bMech.move_velocity(0);
  do {
    const bool readyToScore = updateScoreLift();
    const auto elapsed = pros::millis() - start;
    // Opening the claw takes time and should fit within the autonomous deadline.
    const bool canOpenClaw = timeoutMs == 0 ||
        (elapsed < timeoutMs && kClawOpenTimeMs <= timeoutMs - elapsed);
    if (readyToScore && canOpenClaw) {
      openClaw();
      lastScoreMs = pros::millis();
      mvlib::Logger::getInstance().info("Score macro finished scoring");
      return;
    }

    if (timeoutMs == 0) return;
    if (elapsed >= timeoutMs) break;
    pros::delay(std::min<uint32_t>(10, timeoutMs - elapsed));
  } while (pros::millis() - start < timeoutMs);

  dr4bMech.move_velocity(0);
}

void dr4bHandle() {
  const auto now = pros::millis();
  // Reset after inactivity, and cap dt so a delayed loop cannot skip the ramp.
  const bool freshStart = !dr4bManualActive || now - dr4bLastUpdateMs > 100;
  const float elapsedSeconds = freshStart ? 0.01f :
      std::min(now - dr4bLastUpdateMs, uint32_t{50}) / 1000.0f;
  if (freshStart) dr4bManualPower = 0;
  dr4bLastUpdateMs = now;

  const bool raising = controller.get_digital(DIGITAL_R1);
  const bool lowering = controller.get_digital(DIGITAL_R2);
  const bool slowLower = controller.get_digital(DIGITAL_B);

  if (!raising && !lowering && !slowLower) {
    dr4bManualActive = false;
    dr4bManualPower = 0;
    dr4bMech.move(0);
    liftScoreMacro();
    return;
  }
  dr4bManualActive = true;

  float targetPower = dr4bLowerPower;
  dr4bMech.set_current_limit_all(2500);
  if (raising) {
    // Homing sets the top to zero, so distance remaining is -position.
    const double position = (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2;
    if (!std::isfinite(position)) {
      dr4bManualPower = 0;
      dr4bMech.move(0);
      return;
    }
    const float distanceToTop = dr4bMaxHeight - position;
    const float speedFactor = std::clamp(distanceToTop / dr4bSlowdownDistance, 0.0f, 1.0f);
    targetPower = dr4bRaiseMinPower +
        (dr4bRaiseMaxPower - dr4bRaiseMinPower) * speedFactor;
  } else if (slowLower) {
    dr4bMech.set_current_limit_all(1100);
    dr4bMech.move(slewLiftPower(-127, elapsedSeconds));
    return;
  }
  dr4bMech.move(slewLiftPower(targetPower, elapsedSeconds));
}

void initializeLiftHeight() {
  dr4bMech.move_voltage(7000);
  pros::delay(100); // let lift start moving

  while (dr4bMech.get_actual_velocity(0) > 5) {
    // Wait until lift is done moving up
    pros::delay(20);
  }
  pros::delay(100); // Let motors relax before taring
  dr4bMech.tare_position_all();
  dr4bMech.move_voltage(0);
}
