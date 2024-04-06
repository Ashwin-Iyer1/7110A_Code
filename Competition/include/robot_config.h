#include "vex.h"
#include "sylib/addrled.hpp"

extern vex::competition Competition;

extern vex::brain Brain;
extern vex::controller Controller1;

extern vex::motor FrontLeft;
extern vex::motor MidLeft;
extern vex::motor BackLeft;
extern vex::motor FrontRight;
extern vex::motor MidRight;
extern vex::motor BackRight;
extern vex::motor Catapult1;
extern vex::motor Catapult2;
extern vex::motor Intake;

extern vex::rotation sideTracking;
extern vex::rotation forwardTracking;
//extern vex::rotation hangSensor;

extern vex::pneumatics Hang;
extern vex::pneumatics Descore;
extern vex::pneumatics Wings;

extern vex::motor_group Catapult;

extern sylib::Addrled* DescoreLEDS;
extern sylib::Addrled* Under1;
extern sylib::Addrled* Under2;
extern sylib::Addrled* Under3;
extern sylib::Addrled* Under4;
extern sylib::Addrled* Top;

extern float robotRadius;
extern float sideWheelDist;
extern float forwardWheelDist;
extern float prevForwardRotation;
extern float prevSideRotation;
extern float orientation;
extern float orientationHeading;