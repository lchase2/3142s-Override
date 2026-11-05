#include "./../include/setup.hpp"
#include "drive.hpp"
#include <cmath>
#include <cstdlib>

namespace {
uint32_t dr4bLastStallTime = 0;
double dr4bLastPosition = 0;
enum class StallDir {
  NONE,
  UP,
  DOWN
};

StallDir stallDir = StallDir::NONE;

bool getMotorStalled(pros::MotorGroup* motor, uint32_t mvThreshold = 2500,
                     float velocityThreshold = 5, uint8_t index = 0) {
  return (std::fabs(motor->get_actual_velocity(index)) < velocityThreshold &&
          motor->get_current_draw(index) > mvThreshold);
}
constexpr float dr4bStartingHeight = 0;
constexpr float dr4bEndingHeight = 1000;
} // namespace

/**
 * @brief dr4bHandle
 * Instead of using buttons for up/down, we want to use a joystick.
 * If R1 is held, we transform the analog right Y into the movement of the dr4b
 * We use a combination of the velocity and current to detect if the dr4b has reached the end of it's motion
 * (!true && !true)
 * (true || true)
 */
void dr4bHandle() {
  static auto& logger = mvlib::Logger::getInstance();
  if (!controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) {
    if (dr4bLastPosition < dr4bStartingHeight + 100 /*if dr4b is basically fully lowered*/) {
      dr4bMech.move(0);
    } else {
      dr4bMech.move(0);
      // dr4bMech.move_absolute(dr4bLastPosition, 90);
    }
    dr4bLastStallTime = 0;
    return;
  }

  float power = 0;
  if (controller.get_digital(DIGITAL_R2)) power = -80;
  if (controller.get_digital(DIGITAL_R1)) {
    // 0-1 value of how high dr4b is
    const float dr4bRelativeHeight = dr4bLastPosition / dr4bEndingHeight;
    logger.info("Last: %f", dr4bLastPosition);
    logger.info("Relative: %f", dr4bRelativeHeight);
    power = [&]() -> float {
      // Invert height: 1.0 at the bottom, decreasing toward 0 at the top
      float heightFactor = pow(1.0f - dr4bRelativeHeight, 3);
      logger.info("Height Factor: %.1f", heightFactor);
      // so the lift doesn't crawl to a complete halt at the very top.
      heightFactor = std::max(heightFactor, 0.5f);

      // Scale back up to 0-127 motor units
      return heightFactor * 127.0f;
    }();
    
    logger.info("Power: %.1f", power);
    power = std::clamp(power, 50.0f, 120.0f);
  }

  if ((stallDir == StallDir::UP && power <= 0) ||
      (stallDir == StallDir::DOWN && power >= 0)) {
    stallDir = StallDir::NONE;
  }

  // if ((stallDir == StallDir::UP && power > 0) ||
  //     (stallDir == StallDir::DOWN && power < 0)) {
  //   dr4bMech.move(0);
  //   return;
  // }

  bool dr4bStalled = false;
  for (uint8_t i = 0; i < dr4bMech.size(); i++) {
    if (getMotorStalled(&dr4bMech, 2000, 5, i)) {
      dr4bStalled = true;
      break;
    }
  }


  if (dr4bStalled) {
    logger.info("Dr4b Stalled");
    if (dr4bLastStallTime == 0) dr4bLastStallTime = pros::millis();

    if (pros::millis() - dr4bLastStallTime > 50) {
      dr4bMech.move(0);
      stallDir = (power > 0) ? StallDir::UP : StallDir::DOWN;
      return;
    }
  } else {
    dr4bLastStallTime = 0;
  }

  dr4bMech.move(power);

  dr4bLastPosition = (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2;
}
