#include "vex.h"
#include "sylib/addrled.hpp"
using namespace vex;
competition Competition;

vex::brain       Brain;

controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT4, ratio6_1, true);
motor FrontRight = motor(PORT3, ratio6_1, false);
motor BackLeft = motor(PORT2, ratio6_1, true);
motor BackRight = motor(PORT6, ratio6_1, false);
motor TopLeft = motor(PORT12, ratio6_1, false);
motor TopRight = motor(PORT11, ratio6_1, true);
motor Catapult1 = motor(PORT10, ratio36_1, true);
motor Catapult2 = motor(PORT9, ratio36_1, false);
motor Intake = motor(PORT7, ratio18_1, true);
inertial inertialSensor = inertial(PORT5);
pneumatics Wings = pneumatics(Brain.ThreeWirePort.A);
pneumatics Blocker = pneumatics(Brain.ThreeWirePort.B);
rotation rotationSensor = rotation(PORT1);
pot potentiometer = pot(Brain.ThreeWirePort.C);

motor_group leftGroup = motor_group(FrontLeft, BackLeft, TopLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, TopRight);
motor_group Catapult = motor_group(Catapult1, Catapult2);

sylib::Addrled* BlockerLEDS;
sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Top;
float gearRatio = 36.0/84.0;
float wheelDiameter = 4.125;
float wheelRadius = wheelDiameter/2;
float robotRadius = 6.25;
float drivetrainWidth = 12.5;
void toggleWings() {
  Wings.set(!Wings.value());
}
void toggleBlocker() {
  Blocker.set(!Blocker.value());
  
}