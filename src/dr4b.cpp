#include "./../include/setup.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

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

uint32_t lastScoreMacroReleaseMs = 0;
bool scoreMacroPrevRunning = false;
bool scoreMacroPrevFinished = false;

constexpr int kDistanceReadingMax = 180; // Highest valid reading to detect pins
constexpr int kDistanceReadingReleaseMax = 45; // Highest claw open reading
constexpr int kDistanceReadingReleaseMin = 15; // Lowest claw open reading
constexpr int kDistanceReadingReleaseTarget =
    (kDistanceReadingReleaseMax - kDistanceReadingReleaseMin) /
    2;                                  // Target open reading
constexpr int kDistanceReadingMin = 10; // Lowest valid reading to detect pins
constexpr int kDistanceNoDetect = 9999; // No object detected (from PROS)
} // namespace

/*
 * Controls:
 * One button lowers the lift, watching distance sensor until it reaches a
 * threshold where it then stops, opens claw, and then raises lift.
 *
 *
 *
 *
 *
 *
 *
 */
void liftScoreMacro() {
  static auto& logger = mvlib::Logger::getInstance();

  // Not macro key
  if (!controller.get_digital(DIGITAL_A)) {
    if (scoreMacroPrevRunning)
      lastScoreMacroReleaseMs = pros::millis();
    scoreMacroPrevRunning = false;

    if (pros::millis() - lastScoreMacroReleaseMs > 1000) {

    }
    dr4bMech.move_velocity(0);
    return;
  }
  scoreMacroPrevRunning = true;

  // Lower lift
  float sensorValue = clawDistance.get_distance();
  bool liftWithinRange = sensorValue > kDistanceReadingMin &&
                         sensorValue < kDistanceReadingMax &&
                         sensorValue != kDistanceNoDetect;

  bool liftWithinReleaseRange = sensorValue > kDistanceReadingReleaseMin &&
                                sensorValue < kDistanceReadingReleaseMax &&
                                liftWithinRange;
  if (liftWithinReleaseRange) {
    // In target range, stop moving and open claw
    dr4bMech.move_velocity(0);
    openClaw();
    logger.info("Score macro finished scoring");
    scoreMacroPrevFinished = true;
    return;
  }

  if (sensorValue < kDistanceReadingMax) {
    // Lift is in detectable range of target, slow down if overspeed
    dr4bMech.move_velocity(dr4bMech.get_actual_velocity() > 0 ? -10 : 20);
    logger.info("Score macro in range, slowing down");
    scoreMacroPrevFinished = false;
    return;
  }

  if (!liftWithinRange && sensorValue > kDistanceReadingMax) {
    // Lift is above target, lower
    dr4bMech.move_velocity(-40);
    logger.info("Score macro lowering fullspeed");
    scoreMacroPrevFinished = false;
    // Exit and wait for next call
    return;
  }
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
  if (!raising && !lowering) {
    dr4bManualActive = false;
    dr4bManualPower = 0;
    dr4bMech.move(0);
    liftScoreMacro();
    return;
  }
  dr4bManualActive = true;

  float targetPower = dr4bLowerPower;
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
