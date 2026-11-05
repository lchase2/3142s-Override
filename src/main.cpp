#include "./../include/setup.hpp"
#include "lemlib/chassis/chassis.hpp"

void initializeDr4bDebug() {
  mvlib::Logger& logger = mvlib::Logger::getInstance();
  logger.watch("DR4B Temp", LogLevel::INFO, WatchMode::onInterval, 5_mvS, 
  [&]() { return (dr4bMech.get_temperature(0) + dr4bMech.get_temperature(1)) / 2; },
  mvlib::LevelOverride<double>{
    .elevatedLevel = LogLevel::WARN,
    .predicate = PREDICATE(v > 45)
  });

  logger.watch("DR4B Watts", LogLevel::INFO, WatchMode::onInterval, 250_mvMs, 
  [&]() { return dr4bMech.get_power(0) + dr4bMech.get_power(1); });

  logger.watch("DR4B height", LogLevel::INFO, WatchMode::onInterval, 250_mvMs,
  [&]() { return (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2; });
}

void captureBoomerang(const char* runId,
                      lemlib::Pose p,
                      int timeoutMs,
                      lemlib::MoveToPoseParams params);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  leftDrivetrain.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  rightDrivetrain.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  claw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  dr4bMech.tare_position_all(); // Dr4b MUST be lowered completely before starting
  auto& logger = mvlib::Logger::getInstance();

  logger.setRobot({
    .leftDrivetrain = &leftDrivetrain,
    .rightDrivetrain = &rightDrivetrain
  }, false);

  logger.setTimings({
    .terminalPollingRate = 60_mvMs,
  });
  mvlib::setOdom(&chassis);
  logger.setBuildDate(__DATE__);
  logger.setMinLogLevel(LogLevel::DEBUG);
  // logger.setDefaultWatches({true, true, true});
  // initializeDr4bDebug();
  logger.setLoggingLocation("/DemoRouteFromRobot.log", 
    MissingFolderPolicy::useRoot, 
    ExistingFilePolicy::overwrite);

  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  logger.start();

  // logger.watch("Throttle", LogLevel::INFO, WatchMode::onInterval, 50_mvMs, 
  // [&]() { return controller.get_analog(ANALOG_LEFT_Y); } );
  // logger.watch("Turn", LogLevel::INFO, WatchMode::onInterval, 50_mvMs, 
  // [&]() { return controller.get_analog(ANALOG_RIGHT_X); } );

  logger.watch("Claw Motor Temp", LogLevel::INFO, WatchMode::onInterval, 500_mvMs, 
    [&]() { return claw.get_temperature(); },
    mvlib::LevelOverride<double>{
      .elevatedLevel = LogLevel::WARN,
      .predicate = PREDICATE(v > 45),
      .label = "Claw Motor Overheating"
    }).setActive(false);
  logger.watch("Claw Motor Watts", LogLevel::INFO, WatchMode::onInterval, 150_mvMs, 
    [&]() { return claw.get_power(); }).setActive(false);

    logger.watch("Claw Motor Position", LogLevel::INFO, WatchMode::onChange, 150_mvMs, 
    [&]() { return claw.get_position(); }).setActive(false);


  logger.watch("Claw State", LogLevel::INFO, WatchMode::onChange, 250_mvMs, 
    [&]() { 
      // Convert the ClawState enum to a string
      switch (getClawState()) {
        case ClawState::CLOSED: return "Closed";
        case ClawState::OPEN: return "Open";
        case ClawState::RELAXED: return "Relaxed";
        default: return "Unknown";
      }
    }).setActive(false); 
  logger.watch("DR4B height", LogLevel::INFO, WatchMode::onInterval, 100_mvMs,
  [&]() { return (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2; }).setActive(false);
  // logger.watch("Angular PID", LogLevel::INFO, WatchMode::onInterval, 1000_mvMs, 
  // [&]() { return chassis.angularPID.getOutput(); });
  // logger.setPrintWatches(false);

  chassis.setPose(0, 0, 0);
  logger.info("Strarts");
  captureBoomerang("Backwards boomerang #1",
    {0, 48, 230}, 4000,
    {.forwards = true, .lead = 0.99, .maxSpeed = 127});
}

// | # | Movement | Physical starting pose `(x, y, θ)` | Destination | Lead | Direction |
// |---|---|---|---|---|---|
// | 1 | Straight control | `(0, 0, 0°)` | Point `(0, 36)` | — | Forwards |
// | 2 | In-place turn control | `(0, 0, 0°)` | Heading `90°` | — | — |
// | 3 | Baseline boomerang | `(0, 0, 0°)` | Pose `(24, 24, 90°)` | `0.6` | Forwards |
// | 4 | Smaller lead | `(0, 0, 0°)` | Pose `(24, 24, 90°)` | `0.3` | Forwards |
// | 5 | Larger lead | `(0, 0, 0°)` | Pose `(24, 24, 90°)` | `0.8` | Forwards |
// | 6 | Different starting heading | `(0, 0, 90°)` | Pose `(24, 24, 90°)` | `0.6` | Forwards |
// | 7 | Backwards boomerang | `(0, 0, 180°)` | Pose `(24, 24, 270°)` | `0.6` | Backwards |

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
chassis.setPose(62, 7.5, 297);
auto& logger = mvlib::Logger::getInstance();

closeClaw();
chassis.moveToPoint(54.5, 13.5, 4000, {.forwards = true, .maxSpeed = 50});
chassis.turnToHeading(318, 1000);
chassis.turnToHeading(318, 1000);
 dr4bMech.move(120);
pros::delay(100);
 dr4bMech.move(0);
chassis.moveToPoint(49, 20, 4000, {.forwards = true, .maxSpeed = 50});
chassis.waitUntilDone();
chassis.waitUntilDone();
 dr4bMech.move(-80);
pros::delay(100);
 dr4bMech.move(0);
openClaw();
pros::delay(1000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

control::Slew slew{
  20,
  20,
  5,
  7.5
};

control::Slew lowSlew{
  20,
  20,
  20,
  20
};

control::ExpoTurnConfig expoTurnConfig{
  1.75,
  90,
  90,
  127
};

control::Drivetrain controlDrive{
  &leftDrivetrain,
  &rightDrivetrain,
  &controller
};

control::DriveConfig config{
  .slew = slew,
  .expoTurnConfig = expoTurnConfig,
  .driveMode = control::DriveMode::CURVATURE,
  .drivetrain = controlDrive,
  .expoThrottle = 1.8,
  .deadband = 10,
  .desaturateBias = 0.625
};

screen::Manager ui;



// Gippity test
void captureBoomerang(const char* runId,
                      lemlib::Pose p,
                      int timeoutMs,
                      lemlib::MoveToPoseParams params) {
    auto& logger = mvlib::Logger::getInstance();

    auto lPID = logger.watch("Linear PID raw", LogLevel::INFO, WatchMode::onInterval, 40_mvMs,
    [] { return chassis.lateralPID.getOutput(); });

    auto aPID = logger.watch("Angular PID raw", LogLevel::INFO, WatchMode::onInterval, 40_mvMs,
        [] { return chassis.angularPID.getOutput(); });

    auto lmV = logger.watch("Left motor mV", LogLevel::INFO, WatchMode::onInterval, 40_mvMs,
        [] { return leftDrivetrain.get_voltage(0); });

    auto rmV = logger.watch("Right motor mV", LogLevel::INFO, WatchMode::onInterval, 40_mvMs,
        [] { return rightDrivetrain.get_voltage(0); });

    auto bmV = logger.watch("Battery mV", LogLevel::INFO, WatchMode::onInterval, 200_mvMs,
        [] { return pros::battery::get_voltage(); });


    pros::delay(1000); // Capture a stationary baseline.

    const auto start = chassis.getPose();
    const auto started = pros::millis();

    logger.info(
        "TEST_START id=%s start=(%.3f,%.3f,%.2f) "
        "target=(%.3f,%.3f,%.2f) lead=%.3f forwards=%d "
        "max=%.1f min=%.1f drift=%.2f earlyExit=%.2f timeout=%d",
        runId, start.x, start.y, start.theta,
        p.x, p.y, p.theta, params.lead, int(params.forwards),
        params.maxSpeed, params.minSpeed,
        params.horizontalDrift, params.earlyExitRange, timeoutMs
    );

    chassis.moveToPose(p.x, p.y, p.theta, timeoutMs, params, false);

    const auto end = chassis.getPose();
    logger.info(
        "TEST_END id=%s elapsedMs=%lu end=(%.3f,%.3f,%.2f)",
        runId, static_cast<unsigned long>(pros::millis() - started),
        end.x, end.y, end.theta
    );
    logger.info("A:KDUHQ:IDH");
    pros::delay(1000); // Capture any remaining drift.

    lPID.setActive(false);
    aPID.setActive(false);
    lmV.setActive(false);
    rmV.setActive(false);
    bmV.setActive(false);
}


void opcontrol() {
  auto& logger = mvlib::Logger::getInstance();

  // pros::delay(1000);
  // MotionViewAuton();
  // chassis.turnToHeading(180, 15000, {.direction = AngularDirection::CW_CLOCKWISE}, false);
  // ui.printToScreen("Touch screen to continue...");
  // ui.waitForScreenTouch(5000);
  // ui.clearScreen();
  // chassis.turnToHeading(0, 15000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);

  while (true) {
    dr4bHandle();
    controllerClawHandle();
    float dr4bHeight = (dr4bMech.get_position(0) + dr4bMech.get_position(1)) / 2;

    config.slew = dr4bHeight < 100 ? lowSlew : slew;
    control::updateDrive(config);

    // print pose to screen
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_SMALL, 10, 10, "X: %.2f | Y: %.2f | Theta: %.2f", pose.x, pose.y, pose.theta);
    pros::delay(10);

  }
}
