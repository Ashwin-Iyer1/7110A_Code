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

float gearRatio = 36.0/60.0;
float wheelDiameter = 3.25;
bool smartTurn(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.75;
  float kd = 0;
  float ki = 0;
  float dt = 0.05;
  double currAngle = inertialSensor.rotation(deg);
  double wantedAngle = currAngle + rot;
  e = wantedAngle-inertialSensor.rotation(deg);
  while( abs(e) + abs(d) > 2) {
    e = wantedAngle-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    i += e*dt;
    eRec = e;
    float speed = kp*e + kd*d + ki*i;
    FrontLeft.setVelocity(speed,pct);
    BackLeft.setVelocity(speed,pct);
    FrontRight.setVelocity(-speed,pct);
    BackRight.setVelocity(-speed,pct);
    FrontLeft.spin(fwd);
    BackLeft.spin(fwd);
    FrontRight.spin(fwd);
    BackRight.spin(fwd);
    wait(dt,sec);
  }
  FrontLeft.stop();
  BackLeft.stop();
  FrontRight.stop();
  BackRight.stop();
  //setDriveSpeed(maxVelocity);
  return true;
}
void straight(float dist, distanceUnits units) {
  if(units==distanceUnits::mm) {
    dist*=0.0393701;
  }
  if(units==distanceUnits::cm) {
    dist*=0.393701;
  }
  float rotation = (dist/(wheelDiameter * M_PI)*360) / gearRatio;
  FrontLeft.spinFor(rotation, deg, false);
  BackLeft.spinFor(rotation,deg, false);
  FrontRight.spinFor(rotation,deg, false);
  BackRight.spinFor(rotation,deg, true);
}
void straight(float dist, float speed) {
  FrontLeft.setVelocity(speed,pct);
  FrontRight.setVelocity(speed,pct);
  BackLeft.setVelocity(speed,pct);
  BackRight.setVelocity(speed,pct);
  straight(dist,distanceUnits::in);
}
void straight(float dist) {
  straight(dist,50);
}

int main() {
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  Intake.setVelocity(50,pct);
  straight(25);
  Intake.spinFor(1,sec);
  straight(-26.7);
  smartTurn(90);
  straight(48);
  smartTurn(90);
  //straight(-2);
  Intake.setVelocity(-100,pct);
  Intake.spinFor(1,sec);
  straight(-8,100);
  smartTurn(540);
  straight(-18,100);


}
int drive() {
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

    // INTAKE
    if (Controller1.ButtonUp.pressing()) {
      intakeMode = true;
    } else if (Controller1.ButtonDown.pressing()) {
      intakeMode = false;
    }

    // SINGLE CATAPULT CYCLE
    if (Controller1.ButtonR1.pressing()) {
      Catapult.spinFor(directionType::fwd, 150, rotationUnits::deg, 100, velocityUnits::pct, false);
    } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, false);
    }

    // OUTTAKE
    if(Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd);
    } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(directionType::rev);
    } else {
        Intake.stop();
    }

    // CATAPULT HOLD
    if(Controller1.ButtonA.pressing())

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
