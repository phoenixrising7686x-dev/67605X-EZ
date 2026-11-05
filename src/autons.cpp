#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void close() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  target = SORT_RED;

  //sensor(target);

  roller.move_relative(2750, 200);
  pros::delay(1000);
  pros::Task lift_sequence_task([]() {
    pros::delay(200);
    lift.move_absolute(600, 80);
  });
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 10);
  pros::delay(900);
  chassis.pid_drive_set(5, 110);
  pros::delay(400);
  lift.move_absolute(0, 65);
  pros::delay(550);
  clawPiston.set(true);
  pros::delay(200);
  lift.move_absolute(400, 80);
  pros::delay(400);
  chassis.pid_swing_set(ez::LEFT_SWING, -58_deg, -90, -20);
  pros::delay(600);
  pros::Task lift_sequence_task_2([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_drive_set(16.5, 110);
  pros::delay(600);
  chassis.pid_drive_set(2.5, 20);
  pros::delay(280);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_3([]() {
    lift.move_absolute(800, 80);
  });
  chassis.pid_turn_set(65, 90);
  pros::delay(700);
  chassis.pid_drive_set(7, 110);
  pros::delay(400);
  lift.move_absolute(450, 45);
  pros::delay(250);
  clawPiston.set(true);
  pros::delay(350);
  chassis.pid_drive_set(-4, 90);
  pros::delay(300);
  chassis.pid_swing_set(ez::LEFT_SWING, 25_deg, -90, -85);
  pros::delay(750);
  pros::Task lift_sequence_task_4([]() {
    lift.move_absolute(50, 80);
  });
  chassis.pid_drive_set(21, 110);
  pros::delay(800);
  chassis.pid_drive_set(4, 20);
  pros::delay(700);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_5([]() {
    lift.move_absolute(950, 80);
  });
  chassis.pid_turn_set(162, 90);
  pros::delay(700);
  chassis.pid_drive_set(8, 75);
  pros::delay(700);
  lift.move_absolute(785, 80);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-22, 75);
  pros::delay(700);
  pros::Task lift_sequence_task_6([]() {
    lift.move_absolute(50, 80);
  });
  chassis.pid_turn_set(90, 90);
  pros::delay(700);
  chassis.pid_drive_set(8, 90);
  
}
void far() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  target = SORT_RED;

  //sensor(target);

  roller.move_relative(2750, 200);
  pros::delay(1000);
  pros::Task lift_sequence_task([]() {
    pros::delay(200);
    lift.move_absolute(600, 80);
  });
  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 90, 10);
  pros::delay(900);
  chassis.pid_drive_set(5, 110);
  pros::delay(400);
  lift.move_absolute(0, 65);
  pros::delay(550);
  clawPiston.set(true);
  pros::delay(200);
  lift.move_absolute(400, 80);
  pros::delay(400);
  chassis.pid_swing_set(ez::LEFT_SWING, -55_deg, -90, -20);
  pros::delay(600);
  pros::Task lift_sequence_task_2([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_drive_set(12, 110);
  pros::delay(500);
  clawPiston.set(true);
  pros::delay(100);
  chassis.pid_drive_set(8, 30);
  pros::delay(600);
  chassis.pid_swing_set(ez::RIGHT_SWING, -75_deg, 110, -30);
  pros::delay(400);
  clawPiston.set(false);
  pros::delay(350);
  pros::Task lift_sequence_task_3([]() {
    lift.move_absolute(900, 80);
  });
  chassis.pid_swing_set(ez::RIGHT_SWING, -55_deg, -110, 30);
  pros::delay(500);
  chassis.pid_drive_set(-16, 100);
  pros::delay(600);
  chassis.pid_turn_set(-25, 90);
  pros::delay(400);
  chassis.pid_drive_set(9, 100);
  pros::delay(300);
  lift.move_absolute(500, 80);
  pros::delay(400);
  clawPiston.set(true);
  pros::delay(300);
  chassis.pid_drive_set(-17, 100);
  pros::delay(900);
  chassis.pid_turn_set(-75, 90);
  pros::delay(500);
  chassis.pid_swing_set(ez::LEFT_SWING, 49_deg, 75, 30);
  pros::delay(1000);
  pros::Task lift_sequence_task_4([]() {
    lift.move_absolute(0, 90);
  });
  pros::delay(1000);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_5([]() {
    lift.move_absolute(1100, 90);
  });
  chassis.pid_turn_set(165, 90);
  pros::delay(500);
  chassis.pid_drive_set(10, 70);
  pros::delay(400);
  lift.move_absolute(750, 90);
  pros::delay(700);
  clawPiston.set(true);


  /*
  lift.move_absolute(400, 80);
  pros::delay(400);
  chassis.pid_swing_set(ez::RIGHT_SWING, -58_deg, -90, -20);
  pros::delay(600);
  pros::Task lift_sequence_task_2([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_drive_set(16.5, 110);
  pros::delay(600);
  chassis.pid_drive_set(2.5, 20);
  pros::delay(280);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_3([]() {
    lift.move_absolute(800, 80);
  });
  chassis.pid_turn_set(65, 90);
  pros::delay(700);
  chassis.pid_drive_set(7, 110);
  pros::delay(400);
  lift.move_absolute(450, 45);
  pros::delay(250);
  clawPiston.set(true);
  pros::delay(350);
  chassis.pid_drive_set(-4, 90);
  pros::delay(300);
  chassis.pid_swing_set(ez::RIGHT_SWING, 25_deg, -90, -85);
  pros::delay(750);
  pros::Task lift_sequence_task_4([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_drive_set(21, 110);
  pros::delay(800);
  chassis.pid_drive_set(3, 20);
  pros::delay(700);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_5([]() {
    lift.move_absolute(950, 80);
  });
  chassis.pid_turn_set(162, 90);
  pros::delay(700);
  chassis.pid_drive_set(6, 75);
  pros::delay(700);
  lift.move_absolute(785, 80);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-22, 75);
  pros::delay(700);
  pros::Task lift_sequence_task_6([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_turn_set(90, 90);
  pros::delay(700);
  chassis.pid_drive_set(8, 75);
  */
}
void toggle() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  target = SORT_RED;

  sensor(target);

}

void skills(){
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  target = SORT_RED;

  //sensor(target);

  roller.move_relative(2750, 200);
  pros::delay(1000);
  //pin
  pros::Task lift_sequence_task([]() {
    pros::delay(200);
    lift.move_absolute(600, 80);
  });
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 10);
  pros::delay(900);
  chassis.pid_drive_set(5, 110);
  pros::delay(400);
  lift.move_absolute(0, 65);
  pros::delay(550);
  clawPiston.set(true);
  pros::delay(200);
  //cup 1
  lift.move_absolute(400, 80);
  pros::delay(400);
  chassis.pid_swing_set(ez::LEFT_SWING, -58_deg, -90, -20);
  pros::delay(600);
  pros::Task lift_sequence_task_2([]() {
    lift.move_absolute(0, 80);
  });
  chassis.pid_drive_set(16.5, 110);
  pros::delay(600);
  chassis.pid_drive_set(3.5, 20);
  pros::delay(280);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_3([]() {
    lift.move_absolute(800, 80);
  });
  chassis.pid_turn_set(65, 90);
  pros::delay(700);
  chassis.pid_drive_set(7, 110);
  pros::delay(400);
  lift.move_absolute(450, 45);
  pros::delay(250);
  clawPiston.set(true);
  pros::delay(350);
  //cup 2
  chassis.pid_drive_set(-4, 90);
  pros::delay(300);
  chassis.pid_swing_set(ez::LEFT_SWING, 25_deg, -90, -85);
  pros::delay(750);
  pros::Task lift_sequence_task_4([]() {
    lift.move_absolute(50, 80);
  });
  chassis.pid_drive_set(21, 110);
  pros::delay(800);
  chassis.pid_drive_set(3.5, 20);
  pros::delay(800);
  clawPiston.set(false);
  pros::delay(200);
  pros::Task lift_sequence_task_5([]() {
    lift.move_absolute(1050, 80);
  });
  chassis.pid_turn_set(162, 90);
  pros::delay(700);
  chassis.pid_drive_set(8, 75);
  pros::delay(700);
  lift.move_absolute(785, 80);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-22, 75);
  pros::delay(700);
  pros::Task lift_sequence_task_6([]() {
    lift.move_absolute(50, 80);
  });
  chassis.pid_turn_set(90, 90);
  pros::delay(700);
  chassis.pid_drive_set(20, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
  chassis.pid_drive_set(-11, 90);
  pros::delay(600);
  chassis.pid_turn_set(180, 90);
  pros::delay(500);
  pros::Task lift_sequence_task_7([]() {
    lift.move_absolute(1200, 127);
  });
  chassis.pid_drive_set(30, 80);
  pros::delay(1500);
  lift.move_absolute(1000, 90);
  pros::delay(300);
  clawPiston.set(true);
  pros::delay(200);
  chassis.pid_drive_set(-30, 80);
  pros::delay(200);
  pros::Task lift_sequence_task_8([]() {
    lift.move_absolute(50, 90);
  });
  pros::delay(700);
  chassis.pid_turn_set(90, 90);
  pros::delay(600);
  chassis.pid_drive_set(14, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
//pin 2
  chassis.pid_drive_set(-11, 90);
  pros::delay(700);
  chassis.pid_turn_set(180, 90);
  pros::Task lift_sequence_task_9([]() {
    lift.move_absolute(1550, 127);
  });
  pros::delay(600);
  chassis.pid_drive_set(30, 70);
  pros::delay(1900);
  lift.move_absolute(1200, 90);
  pros::delay(400);
  clawPiston.set(true);
  pros::delay(400);
  chassis.pid_drive_set(-30, 80);
  pros::delay(400);
  pros::Task lift_sequence_task_10([]() {
    lift.move_absolute(50, 90);
  });
  pros::delay(700);
  chassis.pid_turn_set(90, 90);
  pros::delay(600);
  chassis.pid_drive_set(14, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
//pin 3
  chassis.pid_drive_set(-12, 90);
  pros::delay(800);
  chassis.pid_turn_set(180, 90);
  pros::Task lift_sequence_task_11([]() {
    lift.move_absolute(1900, 127);
  });
  pros::delay(600);
  chassis.pid_drive_set(30, 70);
  pros::delay(1900);
  chassis.pid_drive_set(-2.5, 90);
  pros::delay(600);
  lift.move_absolute(1500, 90);
  pros::delay(400);
  clawPiston.set(true);
  pros::delay(400);
  chassis.pid_drive_set(-30, 80);
  pros::delay(400);
//Other Alliance stake
  pros::Task lift_sequence_task_12([]() {
    lift.move_absolute(0, 90);
  });
  pros::delay(700);
  chassis.pid_turn_set(90, 90);
  pros::delay(650);
  chassis.pid_drive_set(14, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
  chassis.pid_drive_set(-9, 90);
  pros::delay(700);
  chassis.pid_turn_set(252, 90);
  pros::Task lift_sequence_task_13([]() {
    lift.move_absolute(500, 90);
  });
  pros::delay(700);
  chassis.pid_drive_set(20, 90);
  pros::delay(700);
  lift.move_absolute(0, 90);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-5, 90);
  pros::delay(700);
  chassis.pid_turn_set(230, 90);
  pros::delay(300);
  chassis.pid_drive_set(-15, 90);
  pros::delay(900);
  chassis.pid_turn_set(90, 90);
  pros::delay(700);
  chassis.pid_drive_set(14, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
  chassis.pid_drive_set(-9, 90);
  pros::delay(700);
  chassis.pid_turn_set(252, 90);
  pros::Task lift_sequence_task_14([]() {
    lift.move_absolute(800, 90);
  });
  pros::delay(700);
  chassis.pid_drive_set(20, 90);
  pros::delay(700);
  lift.move_absolute(0, 90);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-5, 90);
  pros::delay(700);
  chassis.pid_turn_set(230, 90);
  pros::delay(300);
  chassis.pid_drive_set(-15, 90);
  pros::delay(900);
  chassis.pid_turn_set(90, 90);
  pros::delay(700);
  chassis.pid_drive_set(13, 90);
  pros::delay(900);
  clawPiston.set(false);
  pros::delay(400);
  chassis.pid_drive_set(-9, 90);
  pros::delay(700);
  chassis.pid_turn_set(252, 90);
  pros::Task lift_sequence_task_15([]() {
    lift.move_absolute(1100, 90);
  });
  pros::delay(700);
  chassis.pid_drive_set(23, 90);
  pros::delay(700);
  lift.move_absolute(900, 90);
  pros::delay(700);
  clawPiston.set(true);
  pros::delay(700);
  //Final Roller + Park
  chassis.pid_drive_set(-18, 90);
  pros::delay(700);
  pros::Task lift_sequence_task_16([]() {
    lift.move_absolute(0, 90);
  });
  chassis.pid_turn_set(270, 90);
  pros::delay(700);
  chassis.pid_drive_set(43, 127);
  pros::delay(2000);
  chassis.pid_turn_set(180, 90);
  pros::delay(700);
  chassis.pid_drive_set(-8, 90);
  pros::delay(700);
  roller.move_relative(1800, 200);
  pros::delay(1100);
  chassis.pid_drive_set(50, 120);
  pros::delay(5000);
}

void height(){
  lift.move_absolute(1800, 90);
  pros::delay(3000);
  lift.move_absolute(1500, 90);
  pros::delay(2000);
  clawPiston.set(true);
  pros::delay(700);
  chassis.pid_drive_set(-5, 90);
}

void pid() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(10, 110);
  pros :: delay(1000);
  chassis.pid_turn_set(90, 90);
  pros :: delay(1000);
  chassis.pid_drive_set(10, 110);
  pros :: delay(1000);
  chassis.pid_turn_set(180, 90);
  pros :: delay(1000);
  chassis.pid_drive_set(10, 110);
  pros :: delay(1000);
  chassis.pid_turn_set(270, 90);
  pros :: delay(1000);
  chassis.pid_drive_set(10, 110);
  pros :: delay(1000);
  chassis.pid_turn_set(0, 90);
  pros :: delay(1000);
}




//pushback

/*void rsplit() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  chassis.pid_drive_set(33, 110);
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(180, 90);
  pros :: delay(500);
  chassis.pid_drive_set(17, 60);
  pros :: delay(900);
  chassis.pid_drive_set(-30, 90);
  pros :: delay(900);
  hood.set(true);
  stick.move(80);
  pros :: delay(700);
  loader.set(false);
  stick.move(-80);
  chassis.pid_turn_set(-74, 90);
  pros :: delay (775);
  chassis.pid_drive_set(16, 90);
  pros :: delay (200);
  hood.set(false);
  loader.set(true);
  pros :: delay (400);
  chassis.pid_turn_set(-45, 90);
  pros :: delay (50);
  loader.set(false);  
  chassis.pid_drive_set(16.5, 90);
  pros :: delay (1000);
  chassis.pid_drive_set(-2.75, 90);
  intake.move(-70);
  pros :: delay (900);
  chassis.pid_drive_set(-15, 90);
  pros :: delay (600);
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 90, 0);
  wing.set(false);
  pros :: delay (800);
  chassis.pid_drive_set(-18, 80);  
  stick.move_absolute(-10, 90);
}

void lsplit() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  chassis.pid_drive_set(34, 110);
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(-180, 90);
  pros :: delay(500);
  chassis.pid_drive_set(17.5, 60);
  pros :: delay(980);
  chassis.pid_drive_set(-30, 90);
  pros :: delay(900);
  hood.set(true);
  stick.move_absolute(200, 200);  
  pros :: delay(100);
  stick.move_absolute(800, 110);
  pros :: delay(600);
  loader.set(false);
  stick.move_absolute(-10, 110);
  chassis.pid_turn_set(74, 90);
  pros :: delay (725);
  chassis.pid_drive_set(16, 90);
  pros :: delay (200);
  hood.set(false);
  loader.set(true);
  pros :: delay (500);
  chassis.pid_turn_set(-140, 90);
  lift.set(false);
  pros :: delay (500);
  chassis.pid_drive_set(-18, 90);
  pros :: delay (500);
  hood.set(true);
  stick.move_absolute(200, 200);  
  pros :: delay(100);
  stick.move_absolute(1000, 80);
  pros :: delay (900);
  stick.move_absolute(-10,80);
  wing.set(false);
  loader.set(false);
  chassis.pid_drive_set(13, 90);
  pros :: delay (600);
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 10);
  pros :: delay (800);
  chassis.pid_drive_set(21, 90);
  pros :: delay (1000);
  chassis.pid_swing_set(ez::LEFT_SWING, 5_deg, 90, 0);
}


void slowLsplit(){
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  chassis.pid_drive_set(34, 110);
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(-180, 90);
  pros :: delay(500);
  chassis.pid_drive_set(17.5, 60);
  pros :: delay(980);
  chassis.pid_drive_set(-30, 90);
  pros :: delay(900);
  hood.set(true);
  stick.move_absolute(200, 200);  
  pros :: delay(100);
  stick.move_absolute(800, 110);
  pros :: delay(600);
  loader.set(false);
  stick.move_absolute(-10, 110);
  chassis.pid_turn_set(74, 90);
  pros :: delay (725);
  chassis.pid_drive_set(16, 90);
  pros :: delay (200);
  hood.set(false);
  loader.set(true);
  pros :: delay (500);
  chassis.pid_turn_set(-140, 90);
  lift.set(false);
  pros :: delay (500);
  chassis.pid_drive_set(-18, 90);
  pros :: delay (5000);
  hood.set(true);
  stick.move_absolute(200, 200);  
  pros :: delay(100);
  stick.move_absolute(1000, 125);
  pros :: delay (900);
  stick.move_absolute(-10,125);
  wing.set(false);
  loader.set(false);
  chassis.pid_drive_set(13, 90);
  pros :: delay (600);
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 10);
  pros :: delay (800);
  chassis.pid_drive_set(21, 90);
  pros :: delay (1000);
  chassis.pid_swing_set(ez::LEFT_SWING, 5_deg, 90, 0);
}
void r4wing(){
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  chassis.pid_drive_set(32, 110);
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(180, 90);
  pros :: delay(500);
  chassis.pid_drive_set(14, 90);
  pros :: delay(800);
  chassis.pid_drive_set(-30, 127);
  pros :: delay(800);
  hood.set(true);
  stick.move(80);
  pros :: delay(675);
  loader.set(false);
  stick.move(-80);
  chassis.pid_turn_set(230, 50, ez::right_turn);
  pros :: delay(600);
  chassis.pid_drive_set(7, 90);
  pros :: delay(400);
  chassis.pid_turn_set(180, 90);
  wing.set(false);
  pros :: delay(200);
  chassis.pid_drive_set(-27, 127);
}

void l4wing(){
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  chassis.pid_drive_set(31.5, 90);
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(-180, 90);
  pros :: delay(500);
  chassis.pid_drive_set(9, 85);
  pros :: delay(800);
  chassis.pid_drive_set(-30, 90);
  pros :: delay(800);
  hood.set(true);
  stick.move(80);
  pros :: delay(675);
  loader.set(false);
  stick.move(-80);
  chassis.pid_turn_set(230, 50, ez::right_turn);
  pros :: delay(600);
  chassis.pid_drive_set(7, 90);
  pros :: delay(400);
  chassis.pid_turn_set(180, 90);
  wing.set(false);
  pros :: delay(200);
  chassis.pid_drive_set(-27, 127);
}

void rsawp(){
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  chassis.pid_drive_set(32, 90);
  // matchloader
  loader.set(true);
  intake.move(127);
  pros :: delay(1000);
  chassis.pid_turn_set(180, 90);
  pros :: delay(575);
  chassis.pid_drive_set(18, 60);
  pros :: delay(875);
  // high goal
  chassis.pid_drive_set(-30, 90);
  pros :: delay(1000);
  hood.set(true);
  stick.move(80);
  pros :: delay(700);
  loader.set(false);
  stick.move(-80);
  // 3 balls
  chassis.pid_turn_set(-70, 90);
  pros :: delay (725);
  chassis.pid_drive_set(16, 90);
  pros :: delay (200);
  hood.set(false);
  loader.set(true);
  pros :: delay (600);
  loader.set(false);
  chassis.pid_turn_set(-90, 90);
  pros :: delay (300);
  chassis.pid_drive_set(46, 90);
  intake.move(127);
  // intake 3 balls 
  pros :: delay (750);
  loader.set(true);
  pros :: delay (700);
  chassis.pid_turn_set(-140, 90);
  pros :: delay (300);
  chassis.pid_drive_set(39, 90);
  pros :: delay (1025);
  chassis.pid_turn_set(180, 90);
  pros :: delay (400);
  chassis.pid_drive_set(-29, 90);
  pros :: delay (700);
  hood.set(true);
  stick.move(80);
  pros :: delay(600);
  loader.set(true);
  intake.move(127);
  stick.move(-80);
  chassis.pid_drive_set(20, 80);
  pros :: delay (600);
  chassis.pid_drive_set(13, 50);
  pros :: delay (900);
  hood.set(false);
  chassis.pid_drive_set(-8, 90);
  pros :: delay (300);
  loader.set(false);
  chassis.pid_turn_set(223, 90);
  lift.set(false);
  pros :: delay (300);
  chassis.pid_drive_set(-50, 90);
  pros :: delay (900);
  chassis.pid_drive_set(-17, 50);
  pros :: delay (400);
  hood.set(true);
  stick.move(55);
  pros :: delay (1000);
  stick.move_absolute(-10, 200);
  pros :: delay (200);
  stick.move(55);
}

void r7ball(){
  hood.set(false);
  chassis.pid_turn_set(20, 90);
  pros :: delay (100);
  intake.move(127);
  chassis.pid_drive_set(33, 90);
  pros :: delay (500);
  loader.set(true);
  chassis.pid_turn_set(130, 90);  
  pros :: delay (600);
  chassis.pid_drive_set(40, 70);
  pros :: delay (1800);
  chassis.pid_turn_set(180, 90);  
  pros :: delay (500);
  loader.set(true);
  chassis.pid_drive_set(15, 90);
  pros :: delay (600);
  chassis.pid_drive_set(8, 45);
  pros :: delay (400);
  chassis.pid_drive_set(-30, 90);
  pros :: delay(700);
  hood.set(true);
  stick.move(80);
  pros :: delay(700);
  stick.move(-80);
  chassis.pid_drive_set(-100, 60);
  hood.set(false);
  lift.set(false);

}

void skills() {
  intake.move(127);
} */