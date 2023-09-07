/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       krish                                                     */
/*    Created:      8/31/2023, 10:02:46 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <fstream>
#include "stdarg.h"
#include <cstring>
#include <string.h>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
controller Controller1 = controller(primary);

motor FrontLeft = motor(PORT6, ratio6_1, true);
motor FrontRight = motor(PORT2, ratio6_1, false);
motor BackLeft = motor(PORT3, ratio6_1, true);
motor BackRight = motor(PORT4, ratio6_1, false);
motor Catapult = motor(PORT10, ratio36_1, true);
motor Intake = motor(PORT7, ratio18_1, true);
inertial inertialSensor = inertial(PORT5);  

int main() {
    Intake.setVelocity(100,pct);
    //inertialSensor.calibrate();
    //waitUntil(!inertialSensor.isCalibrating());
    int deadband = 5;
    bool intakeMode = false;
  while (true) {
    //tank drive

    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position()-Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position()+Controller1.Axis1.position());

    /*if(Launcher1.velocity(rpm)!=0) {
      e = Launcher1.velocity(rpm)-350;
      d = e-eRec;
      i+=e;
      eRec=e;
      launcherSpeed = 350 - 150*(ki*i + ke*e + kd*d);
    }*/
    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    
    if (abs(leftMotorSpeed) < deadband) {
      // Set the speed to zero.
      FrontLeft.setVelocity(0, percent);
      BackLeft.setVelocity(0, percent);
    } else {
      // Set the speed to leftMotorSpeed
      FrontLeft.setVelocity(leftMotorSpeed, percent);
      BackLeft.setVelocity(leftMotorSpeed, percent);
    }
    if (Controller1.ButtonUp.pressing()) {
      intakeMode = true;
    } else if (Controller1.ButtonDown.pressing()) {
      intakeMode = false;
    }
    if (Controller1.ButtonR1.pressing()) {
      Catapult.spinFor(directionType::fwd, 150, rotationUnits::deg, 100, velocityUnits::pct, false);
    } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, false);
    }

    if(Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd);
    } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(directionType::rev);
    } else {
        Intake.stop();
    }

    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(rightMotorSpeed) < deadband) {
      // Set the speed to zero
      FrontRight.setVelocity(0, percent);
      BackRight.setVelocity(0, percent);
    } else {
      // Set the speed to rightMotorSpeed
      FrontRight.setVelocity(rightMotorSpeed, percent);
      BackRight.setVelocity(rightMotorSpeed, percent);
    }
    /*if(Controller1.ButtonX.pressing()) {
      //Piston.set(false);
      expansion();
    }*/ 
    // Spin both motors in the forward direction.
    FrontLeft.spin(forward);
    FrontRight.spin(forward);
    BackLeft.spin(forward);
    BackRight.spin(forward);

    wait(25, msec);
  }
}
