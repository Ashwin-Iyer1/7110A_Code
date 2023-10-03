/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       s611863                                                   */
/*    Created:      10/3/2023, 8:15:04 AM                                     */
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

// A global instance of competition
competition Competition;

vex::brain       Brain;

controller Controller1 = controller(primary);

motor FrontLeft = motor(PORT6, ratio6_1, true);
motor FrontRight = motor(PORT2, ratio6_1, false);
motor BackLeft = motor(PORT3, ratio6_1, true);
motor BackRight = motor(PORT4, ratio6_1, false);
motor Catapult = motor(PORT10, ratio36_1, true);
motor Intake = motor(PORT7, ratio18_1, true);
inertial inertialSensor = inertial(PORT5);  
pneumatics Wings = pneumatics(Brain.ThreeWirePort.A); 
rotation rotationSensor = rotation(PORT1);

bool catatoggle = false;
void toggleCata() {
    
    Controller1.Screen.print(Catapult.isSpinning());
    if(abs(Catapult.velocity(rpm) )>0) {
        Catapult.stop();
    } else {
        Catapult.spin(directionType::fwd, 100, velocityUnits::pct);
    }
}
void stopCata() {
    Catapult.stop();
}
void toggleWings() {
  Wings.set(!Wings.value());
}

float gearRatio = 36.0/60.0;
float wheelDiameter = 3.25;
float wheelRadius = wheelDiameter/2;
float robotRadius = 6.25;
double driveRotationConstant = 0.8721445746*1.054572148;
void simpleTurn(float deg) {
  float dist = driveRotationConstant * gearRatio * deg * robotRadius / wheelRadius;
  FrontLeft.spinFor(fwd, dist, vex::deg, false);
  BackLeft.spinFor(fwd, dist, vex::deg, false);
  FrontRight.spinFor(reverse, dist, vex::deg, false);
  BackRight.spinFor(reverse, dist, vex::deg, true);
}
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

void pre_auton(void) {
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  Intake.setVelocity(50,pct);
}

void autonomous(void) {
  Intake.setVelocity(50,pct);
  straight(48);
  smartTurn(90);
  Intake.setVelocity(-75,pct);
  Intake.spinFor(1.5,sec);
  straight(-8,100);
  smartTurn(180);
  straight(-18,100);

}

void usercontrol(void) {
  Intake.setVelocity(100,pct);
  //inertialSensor.calibrate();
  //waitUntil(!inertialSensor.isCalibrating());
  int deadband = 5;
  bool intakeMode = false;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  while (true) {
    //tank drive

    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * Controller1.Axis1.position());

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
      FrontRight.setVelocity(0, percent);
      BackRight.setVelocity(0, percent);
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
      Brain.Screen.print("Down");
    } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, 100, velocityUnits::pct, false);
        Brain.Screen.print("Up");

    } 
    /*
    else if (Controller1.ButtonB.pressing()) {
        if (catatoggle) {
            catatoggle = false;
            Catapult.stop();
            Brain.Screen.print("Catatoggle stop");

        } else {
            catatoggle = true;
            Catapult.spin(directionType::fwd, 100, velocityUnits::pct);
            Brain.Screen.print("Catatoggle");
        }
        
    }*/ else if(Controller1.ButtonA.pressing()) {
        Catapult.spin(directionType::fwd, 50, velocityUnits::pct);
        //Brain.Screen.print("Hold");
        // Brain.Screen.clearScreen();
    }

    // OUTTAKE
    if(Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd);
    } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(directionType::rev);
    } else {
        Intake.stop();
    }

    // // CATAPULT HOLD
    // if(Controller1.ButtonA.pressing()) {
    //     Catapult.spin(directionType::fwd, 100, velocityUnits::pct);
    // } else if (!(Controller1.ButtonR1.pressing()) || catatoggle != true) {
    //     Catapult.stop();
    //     Brain.Screen.print("Um akshully this motor should um stop");
    //     Brain.Screen.newLine();
    // }

    // // CATAPULT TOGGLE
    // if (Controller1.ButtonB.pressing()) {
    //     if (catatoggle) {
    //         catatoggle = false;
    //         Catapult.stop();
    //     } else {
    //         catatoggle = true;
    //         Catapult.spin(directionType::fwd, 100, velocityUnits::pct);
    //         Brain.Screen.clearScreen();
    //     }
    // }

    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero   .
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

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}