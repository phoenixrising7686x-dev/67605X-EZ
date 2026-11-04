#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup lift({1, -10});
inline pros::Optical optical(12);
inline pros::MotorGroup roller({5, -6});
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::adi::DigitalIn clawSwitch('H');
inline ez::Piston clawPiston('G');

// Color selection
enum SortColor { SORT_BLUE, SORT_RED };
inline SortColor target = SORT_BLUE;          // default if nothing sets it

// Function declarations (code lives in subsystems.cpp)
bool sees_color(SortColor c);
void sensor(SortColor c, int timeout_ms = 3000);

                  
