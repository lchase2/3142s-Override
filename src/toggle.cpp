#include "./../include/setup.hpp"
#include "pros/rtos.h"
#include <cstdlib>

namespace {
ToggleState currentToggleState = ToggleState::OFF;
} // namespace

void handleToggle() {
  if (controller.get_digital_new_press(DIGITAL_X)) {
    currentToggleState = currentToggleState == ToggleState::SPINNING ? ToggleState::OFF : ToggleState::SPINNING;
    setToggleState(currentToggleState);
  }
}

ToggleState getToggleState() {
  return currentToggleState;
}

void setToggleState(ToggleState state) {
  currentToggleState = state;
  toggle.move(currentToggleState == ToggleState::SPINNING ? 90 : 0);
}
