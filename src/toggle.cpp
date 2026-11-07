#include "./../include/setup.hpp"
#include "pros/rtos.h"
#include <cstdlib>

namespace {
bool currentlySpinning = false;
} // namespace

void handleToggle() {
  if (controller.get_digital_new_press(DIGITAL_X)) {
    currentlySpinning = !currentlySpinning;
    toggle.move(currentlySpinning ? 127 : 0);
  }
}
