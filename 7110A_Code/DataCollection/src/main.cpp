/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       krish                                                     */
/*    Created:      8/21/2023, 10:02:46 AM                                    */
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
inertial inertialSensor = inertial(PORT5);  

int main() {

    //Brain.Screen.printAt( 10, 50, "Hello V5" );
    //std::ofstream MyFile("data.csv");

    // Write to the file
    
    //MyFile << "FrontLeft,FrontRight,BackLeft,BackRight,Inertial\n";
    /*
    MyFile << "1,2,3,4,5\n";
    MyFile << "2,4,6,8,10\n";
    MyFile << "3,6,9,12,15\n";
    MyFile << "4,8,12,16,20\n";
    MyFile << "5,10,15,20,25\n";
    */

    // Close the file
    //MyFile.close();
    /*
    float prevFrontLeft = 0;
    float prevFrontRight = 0;
    float prevBackLeft = 0;
    float prevBackRight = 0;
    float prevInertial = 0;

    for(int i = 0; i<1000; i++) {
        
        MyFile << (FrontLeft.position(deg) - prevFrontLeft) << "," << (FrontRight.position(deg) - prevFrontRight) << "," << (BackLeft.position(deg) - prevBackLeft) << "," << (BackRight.position(deg) - prevBackRight) << "," << (inertialSensor.rotation(deg) - prevInertial) << "\n";
        prevFrontLeft = FrontLeft.position(deg);
        prevFrontRight = FrontRight.position(deg);
        prevBackLeft = BackLeft.position(deg);
        prevBackRight = BackRight.position(deg);
        prevInertial = inertialSensor.rotation(deg);
        wait(100,msec);
    }
    MyFile.close();
    */
    inertialSensor.calibrate();
    waitUntil(!inertialSensor.isCalibrating());
    int deadband = 5;
    bool intakeMode = false;
    std::ofstream MyFile("data.csv");
    MyFile << "FrontLeft,FrontRight,BackLeft,BackRight,Inertial\n";
    float prevFrontLeft = 0;
    float prevFrontRight = 0;
    float prevBackLeft = 0;
    float prevBackRight = 0;
    float prevInertial = 0;
  while (true) {
    // Get the velocity percentage of the left motor. (Axis3)
    int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());
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
    MyFile << (FrontLeft.position(deg) - prevFrontLeft) << "," << (FrontRight.position(deg) - prevFrontRight) << "," << (BackLeft.position(deg) - prevBackLeft) << "," << (BackRight.position(deg) - prevBackRight) << "," << (inertialSensor.rotation(deg) - prevInertial) << "\n";
    prevFrontLeft = FrontLeft.position(deg);
    prevFrontRight = FrontRight.position(deg);
    prevBackLeft = BackLeft.position(deg);
    prevBackRight = BackRight.position(deg);
    prevInertial = inertialSensor.rotation(deg);

    wait(25, msec);
  }
}
