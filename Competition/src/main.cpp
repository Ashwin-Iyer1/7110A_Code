/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       s617995                                                   */
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
motor TopLeft = motor(PORT11, ratio6_1, false);
motor TopRight = motor(PORT12, ratio6_1, true);
motor Catapult1 = motor(PORT10, ratio36_1, true);
motor Catapult2 = motor(PORT9, ratio36_1, false);
motor Intake = motor(PORT7, ratio18_1, true);
inertial inertialSensor = inertial(PORT5);
pneumatics Wings = pneumatics(Brain.ThreeWirePort.A);
rotation rotationSensor = rotation(PORT1);

bool catatoggle = false;
void stopCata() {
    Catapult1.stop();
    Catapult2.stop();
}

void bringCataDown(float angle) {
  while (rotationSensor.position(deg) > angle) {
        Catapult1.spin(fwd);
        Catapult2.spin(fwd);
        wait(20, msec);
      }
      stopCata();
}
int bringCataDown() {
  bringCataDown(148);
  return 1;
}
void cataMatchLoad() {
  bringCataDown(160);
}
int fullCataCycle() {
  Catapult1.spinFor(directionType::fwd, 360, rotationUnits::deg, 100, velocityUnits::pct, false);
  Catapult2.spinFor(directionType::fwd, 360, rotationUnits::deg, 100, velocityUnits::pct, true);
  bringCataDown();
  stopCata();
  return 1;
}
void toggleCata() {
    
    Controller1.Screen.print(Catapult1.isSpinning());
    if(abs(Catapult1.velocity(rpm) )>0) {
        bringCataDown();
        Catapult1.stop();
        Catapult2.stop();
    } else {
        Catapult1.spin(directionType::fwd, 50, velocityUnits::pct);
        Catapult2.spin(directionType::fwd, 50, velocityUnits::pct);
    }
}


void toggleWings() {
  Wings.set(!Wings.value());
}

float gearRatio = 36.0/60.0;
float wheelDiameter = 3.25;
float wheelRadius = wheelDiameter/2;
float robotRadius = 6.25;
double driveRotationConstant = 0.8721445746*1.054572148;

//Currently inaccurate, will require tuning of driveRotationConstant
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
    TopLeft.setVelocity(speed, pct);
    FrontRight.setVelocity(-speed,pct);
    BackRight.setVelocity(-speed,pct);
    TopRight.setVelocity(-speed,pct);
    FrontLeft.spin(fwd);
    BackLeft.spin(fwd);
    TopLeft.spin(fwd);
    FrontRight.spin(fwd);
    BackRight.spin(fwd);
    TopRight.spin(fwd);
    wait(dt,sec);
  }
  FrontLeft.stop();
  BackLeft.stop();
  TopLeft.stop();
  FrontRight.stop();
  BackRight.stop();
  TopRight.stop();
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
  TopLeft.spinFor(rotation, deg, false);
  FrontRight.spinFor(rotation,deg, false);
  BackRight.spinFor(rotation,deg, false);
  TopRight.spinFor(rotation, deg, true);
}
void straight(float dist, float speed) {
  FrontLeft.setVelocity(speed,pct);
  FrontRight.setVelocity(speed,pct);
  BackLeft.setVelocity(speed,pct);
  BackRight.setVelocity(speed,pct);
  TopLeft.setVelocity(speed,pct);
  TopRight.setVelocity(speed,pct);
  straight(dist,distanceUnits::in);
}
void straight(float dist) {
  straight(dist,50);
}
//In case intake requires the robot to rock back and forth to outtake
int shake() {
  for(int i=0; i<3; i++) {
    straight(1,100);
    straight(-1,100); 
  }
  return 1;
}

void pre_auton(void) {
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  Catapult1.setStopping(brakeType::coast);
  Catapult2.setStopping(brakeType::coast);
  Intake.setVelocity(50,pct);
  FrontLeft.setVelocity(100,pct);
  BackLeft.setVelocity(100,pct);
  TopLeft.setVelocity(100,pct);
  FrontRight.setVelocity(100,pct);
  BackRight.setVelocity(100,pct);
  TopRight.setVelocity(100,pct);
}
void brakeAll() {
  FrontLeft.setStopping(brakeType::brake);
  FrontRight.setStopping(brakeType::brake);
  BackLeft.setStopping(brakeType::brake);
  BackRight.setStopping(brakeType::brake);
  TopLeft.setStopping(brakeType::brake);
  TopRight.setStopping(brakeType::brake);
}
void oppositeSide(void) {
  vex::task run(bringCataDown);
  Intake.setVelocity(50,pct);
  straight(48);
  smartTurn(90);
  Intake.setVelocity(-75,pct);
  vex::task async(shake);
  Intake.spinFor(4,sec);
  straight(-8,100);
  smartTurn(180);
  straight(-12,100);
  
  //code for touching elevation
  straight(18);
  smartTurn(90);
  straight(-24);
  smartTurn(90);
  straight(-20);
  toggleWings();
  smartTurn(360);
  straight(-10);
}
void sameSide(void) {
  //Takes triball out of corner with wing and launches triball
  brakeAll();
  bringCataDown();
  toggleWings();
  wait(1,sec);
  straight(-4);
  simpleTurn(190);
  toggleWings();
  straight(-20);
  smartTurn(90);
  straight(-30);
  cataMatchLoad();
}
void programmingSkills(void) {
  bringCataDown();
  straight(10);
  toggleCata();
  wait(40,sec);
  stopCata();
  bringCataDown();
  straight(-20);

  smartTurn(-55);
  straight(-108);
  smartTurn(90);
  straight(-84);
  smartTurn(90);
  toggleWings();
  straight(-55);
  smartTurn(-90);
  straight(-18);
}
  
void usercontrol(void) {
  Intake.setVelocity(100,pct);
  int deadband = 5;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonLeft.pressed(cataMatchLoad);
  while (true) {
    //tank drive

    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? 1 : -1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * Controller1.Axis1.position());

    // Set the speed of the left motors. If the value is less than the deadband,
    // set it to zero.
    
    if (abs(leftMotorSpeed) < deadband) {
      FrontLeft.setVelocity(0, percent);
      BackLeft.setVelocity(0, percent);
      TopLeft.setVelocity(0, percent);
      FrontRight.setVelocity(0, percent);
      BackRight.setVelocity(0, percent);
      TopRight.setVelocity(0, percent);
    } else {
      FrontLeft.setVelocity(leftMotorSpeed, percent);
      BackLeft.setVelocity(leftMotorSpeed, percent);
      TopLeft.setVelocity(leftMotorSpeed, percent);
    }

    // Drivetrain inversion
    if (Controller1.ButtonUp.pressing()) {
      intakeMode = true;
    } else if (Controller1.ButtonDown.pressing()) {
      intakeMode = false;
    }

    // Single Catapult Cycle
    if (Controller1.ButtonR1.pressing()) {
      vex::task run(bringCataDown);
    } else if (Controller1.ButtonR2.pressing()) {
        vex::task run(fullCataCycle);

    } else if(Controller1.ButtonA.pressing()) {
        Catapult1.spin(directionType::fwd, 50, velocityUnits::pct);
        Catapult2.spin(directionType::fwd, 50, velocityUnits::pct);
    }

    // OUTTAKE
    if(Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd);
    } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(directionType::rev);
    } else {
        Intake.stop();
    }

    // Set the speed of the right motors. If the value is less than the deadband,
    // set it to zero   .
    if (abs(rightMotorSpeed) < deadband) {
      FrontRight.setVelocity(0, percent);
      BackRight.setVelocity(0, percent);
      TopRight.setVelocity(0, percent);
    } else {
      FrontRight.setVelocity(rightMotorSpeed, percent);
      BackRight.setVelocity(rightMotorSpeed, percent);
      TopRight.setVelocity(rightMotorSpeed, percent);
    }
    // Spin all drivetrain motors in the forward direction.
    FrontLeft.spin(forward);
    FrontRight.spin(forward);
    BackLeft.spin(forward);
    BackRight.spin(forward);
    TopLeft.spin(forward);
    TopRight.spin(forward);

    wait(25, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(oppositeSide);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(programmingSkills);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}