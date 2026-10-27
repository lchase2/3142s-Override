#include "setup.hpp"
void MotionViewAuton() {
auto& logger = mvlib::Logger::getInstance();

chassis.setPose(62, 0, 270);
chassis.moveToPoint(19.5, 0, 4000, {.forwards = true, .maxSpeed = 127});
chassis.turnToHeading(20, 1000);
chassis.moveToPoint(42, 55, 2000, {.forwards = true, .maxSpeed = 127});
chassis.turnToHeading(170, 1000);
chassis.moveToPoint(46.5, 28.5, 2000, {.forwards = true, .maxSpeed = 51});
chassis.turnToHeading(180, 1000, {}, false);
chassis.waitUntilDone();
pros::delay(1000);
chassis.moveToPoint(50.5, 58.5, 4000, {.forwards = false, .maxSpeed = 127});
chassis.turnToHeading(90, 800);
chassis.moveToPoint(63.25, 58.5, 1000, {.forwards = true, .maxSpeed = 127}, false);
chassis.waitUntilDone();
logger.info("Auton complete!");
pros::delay(1000);
logger.pause();
//Testing git commit
}