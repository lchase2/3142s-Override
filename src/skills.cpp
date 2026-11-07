#include "./../include/setup.hpp"
void MotioasdasdnViewAuton() {
chassis.setPose(6, 64, 180);
chassis.moveToPoint(8, 18, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 30});
dr4bMech.move(100);
pros::delay(400);
dr4bMech.move(0);
chassis.waitUntilDone();
chassis.moveToPoint(1.5, 4.5, 4000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0}, false);
chassis.waitUntilDone();
liftScoreMacro(3000);
setClawAction(ClawState::OPEN);
pros::delay(1000);
dr4bMech.move(100);
pros::delay(200);
chassis.moveToPoint(13, 14.5, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(50, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.waitUntilDone();
dr4bMech.move(-50);
chassis.moveToPoint(20, 20.5, 4000, {.forwards = true, .maxSpeed = 50, .minSpeed = 0}, false);
setClawAction(ClawState::CLOSED);
pros::delay(500);
dr4bMech.move(100);
chassis.moveToPoint(16, 0.5, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.turnToHeading(270, 2000, {.maxSpeed = 127, .minSpeed = 0}, false);
chassis.waitUntilDone();
dr4bMech.move(0);
chassis.moveToPoint(5.5, -0.5, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0}, false);
chassis.waitUntilDone();
liftScoreMacro(3000);
setClawAction(ClawState::OPEN);
pros::delay(1000);
dr4bMech.move(100);
pros::delay(500);
setClawAction(ClawState::RELAXED);
}