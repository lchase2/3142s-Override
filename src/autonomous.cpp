#include "./../include/setup.hpp"
void MotionViewAuton() {
chassis.setPose(0, 64, 180);
chassis.moveToPoint(10, 47, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0});
chassis.turnToHeading(90, 2000, {.maxSpeed = 127, .minSpeed = 0});
dr4bMech.move(100);
pros::delay(400);
dr4bMech.move(0);
chassis.moveToPoint(19, 46.5, 4000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0}, false);
chassis.waitUntilDone();
liftScoreMacro(3000);
setClawAction(ClawState::OPEN);
pros::delay(1000);
chassis.moveToPoint(9.5, 47, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(150, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.waitUntilDone();
dr4bMech.move(-50);
chassis.moveToPoint(16.5, 35.5, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 10}, false);
chassis.moveToPoint(21.5, 26.5, 4000, {.forwards = true, .maxSpeed = 50, .minSpeed = 0}, false);
chassis.waitUntilDone();
setClawAction(ClawState::CLOSED);
chassis.moveToPoint(23, 33.5, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
dr4bMech.move(100);
pros::delay(400);
dr4bMech.move(0);
chassis.moveToPoint(23.5, 42.5, 4000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0}, false);
liftScoreMacro(1500);
setClawAction(ClawState::OPEN);
}