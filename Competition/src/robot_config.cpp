#include "vex.h"
#include "sylib/addrled.hpp"
using namespace vex;

// A global instance of competition
competition Competition;

vex::brain Brain;

controller Controller1 = controller(primary);

motor      FrontLeft =            motor(PORT1, ratio6_1, true);
motor      MidLeft =              motor(PORT2, ratio6_1, true);
motor      BackLeft =             motor(PORT7, ratio6_1, true);
motor      FrontRight =           motor(PORT4, ratio6_1, false);
motor      MidRight =             motor(PORT5, ratio6_1, false);
motor      BackRight =            motor(PORT17, ratio6_1, false);
motor      Catapult1 =            motor(PORT10, ratio18_1, true);
motor      Catapult2 =            motor(PORT9, ratio18_1, true);
motor      Intake =               motor(PORT8, ratio18_1, true);

rotation   sideTracking =      rotation(PORT19);
rotation   forwardTracking =   rotation(PORT6);
//rotation   hangSensor =        rotation(PORT14);

pneumatics Hang =             pneumatics(Brain.ThreeWirePort.A);
pneumatics Descore =         pneumatics(Brain.ThreeWirePort.B);
pneumatics Wings =           pneumatics(Brain.ThreeWirePort.D);

motor_group Catapult =   motor_group(Catapult1, Catapult2);

sylib::Addrled* DescoreLEDS;
sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Under3;
sylib::Addrled* Under4;
sylib::Addrled* Top;

float robotRadius = 5.75;
float sideWheelDist = -7.75;
float forwardWheelDist = 0.42;
float prevForwardRotation = 0;
float prevSideRotation = 0;
float orientation = 0;
float orientationHeading = fmod(orientation+36000,360);