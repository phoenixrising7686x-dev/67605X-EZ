#include "main.h"

bool sees_color(SortColor c) {
  if (optical.get_proximity() < 30) return false;

  double hue = optical.get_hue();
  if (c == SORT_BLUE) return (hue > 200 && hue < 250);
  return (hue < 20 || hue > 340);   // RED
}

// Spins the roller until the color is seen, then stops.
// Times out so auton can't get stuck.
void sensor(SortColor c, int timeout_ms) {
  optical.set_led_pwm(100);
  int elapsed = 0;

  while (!sees_color(c) && elapsed < timeout_ms) {
    roller.move(127);
    pros::delay(10);
    elapsed += 10;
  }
  roller.brake();
}

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-2, -3, -4},     // Left Chassis Ports (negative port will reverse it!)
    {7, 8, 9},  // Right Chassis Ports (negative port will reverse it!)

    11,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  clawPiston.set(false);

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(3, 3);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"Close Alliance" , close},
    {"Far Alliance" , far},
    {"Toggle Only", toggle},
    {"Skills" , skills},
    //{"PID" , pid},
    //{"Height Testing" , height},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
 
  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void opcontrol() {
  //Below is the thing for skills macro
  //ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  lift.set_brake_mode_all(pros::MotorBrake::hold);
  //cflip.set(false);
  //chassis.opcontrol_joystick_practicemode_toggle(true);

  // Show which color the auton set
  master.set_text(0, 0, target == SORT_BLUE ? "Target: BLUE" : "Target: RED ");
  clawPiston.set(true);
  bool l2waspressed = false;
  bool limittriggeredwhileholding = false;
  bool l2pressed = false;

  optical.set_integration_time(20);   // in ms, valid range is 3 to 712

  while (true) {
    optical.set_led_pwm(100);

    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    if (master.get_digital(DIGITAL_R2))
      lift.move(120);
    else if (master.get_digital(DIGITAL_L2))
      lift.move(-127);
    else
      lift.move_velocity(0);

    // B = manually flip the target color
    if (master.get_digital_new_press(DIGITAL_B)) {
      target = (target == SORT_BLUE) ? SORT_RED : SORT_BLUE;
      master.rumble(".");
      master.set_text(0, 0, target == SORT_BLUE ? "Target: BLUE" : "Target: RED ");
    }

    if (master.get_digital(DIGITAL_LEFT)) {
      // OVERRIDE: spins no matter what the sensor says
      roller.move(127);
    } else if (master.get_digital(DIGITAL_R1)) {
      // Sensor mode: spin until target color is seen
      if (sees_color(target)) roller.brake();
      else roller.move(127);
    } else {
      roller.brake();
    }

    
    if (master.get_digital_new_press(DIGITAL_L1)){
      clawPiston.set(!clawPiston.get());
    } else if (clawSwitch.get_value() && lift.get_position() < 400) {
      clawPiston.set(false);
    }



    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }


}
 