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
#include "vex_motor.h"

using namespace vex;

// A global instance of competition
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

motor_group leftGroup = motor_group(FrontLeft, BackLeft, TopLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, TopRight);
motor_group Catapult = motor_group(Catapult1, Catapult2);
vex::task cata;
bool catatoggle = false;
void stopCata() {
    Catapult.stop();
}
void cataSpin() {
  float bandStrength = (rotationSensor.angle(deg)<220 && rotationSensor.angle(deg)>140) ? 220-rotationSensor.angle(deg) : 0;
  float cataVoltage = fmin(10.9, 2 + bandStrength/7);
  Catapult.spin(fwd,cataVoltage, volt);
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
  Catapult1.spinFor(directionType::fwd, 360, rotationUnits::deg, 100, velocityUnits::pct, true);
  bringCataDown();
  stopCata();
  return 1;
}
int toggleCataTask() {
  while(true) {
    cataSpin();
    wait(100, msec);
  }
  return 1;
}
void toggleCata() {
  if(!catatoggle) {
    cata.resume();
    catatoggle = true;
  } else {
    cata.suspend();
    stopCata();
    catatoggle = false;
  }
}
void toggleWings() {
  Wings.set(!Wings.value());
}
void toggleBlocker() {
  Blocker.set(!Blocker.value());
}

float gearRatio = 36.0/84.0;
float wheelDiameter = 4;
float wheelRadius = wheelDiameter/2;
float robotRadius = 6.25;
float drivetrainWidth = 12.5;
float prevLeftRotation = 0;
float prevRightRotation = 0;
float orientation = 0;
double driveRotationConstant = 0.8721445746*1.054572148;
float x = 0;
float y = 0;

float distToRot(float dist) {
  return (dist/(wheelDiameter * M_PI)*360) / gearRatio;
}
//Currently inaccurate, will require tuning of driveRotationConstant
void simpleTurn(float deg) {
  float dist = driveRotationConstant * gearRatio * deg * robotRadius / wheelRadius;
  leftGroup.spinFor(fwd, dist, vex::deg, false);
  rightGroup.spinFor(reverse, dist, vex::deg, false);
}
bool smartTurn(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.75;
  float kd = 0.01;
  float ki = 0;
  float dt = 0.05;
  double currAngle = inertialSensor.rotation(deg);
  double wantedAngle = currAngle + rot;
  e = wantedAngle-inertialSensor.rotation(deg);
  while( fabs(e) + fabs(d) > 2) {
    e = wantedAngle-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    i += e*dt;
    eRec = e;
    float speed = kp*e + kd*d + ki*i;
    leftGroup.setVelocity(speed,pct);
    rightGroup.setVelocity(-speed,pct);
    leftGroup.spin(fwd);
    rightGroup.spin(fwd);
    wait(dt,sec);
  }
  leftGroup.stop();
  rightGroup.stop();
  return true;
}
bool turnToHeading(float heading) {
  float clockwiseRotation = heading-inertialSensor.heading();
  float closestPath = 0;
  if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
    closestPath = clockwiseRotation;
    if(fabs(clockwiseRotation-360) < (closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  smartTurn(closestPath);
  return true;
}
void odomUpdate() {
  float leftPos = (TopLeft.position(deg)-prevLeftRotation)/180*M_PI * wheelRadius;
  float rightPos = (TopRight.position(deg)-prevRightRotation)/180*M_PI * wheelRadius;
  orientation += ((leftPos-rightPos)/drivetrainWidth)/M_PI*180;
  float radOrientation = orientation/180*M_PI;
  if(fabs((leftPos-rightPos)/drivetrainWidth) < 0.001) {
    x += (leftPos+rightPos)/2 * cosf(-radOrientation + M_PI/2);
    y += (leftPos+rightPos)/2 * sinf(-radOrientation + M_PI/2);
  } else {
    float radius = (leftPos/((leftPos-rightPos)/drivetrainWidth)) - drivetrainWidth/2;
    x += -radius * (cosf(radOrientation) - cosf(radOrientation - ((leftPos-rightPos)/drivetrainWidth)));
    y += radius * (sinf(radOrientation) - sinf(radOrientation - ((leftPos-rightPos)/drivetrainWidth)));
  }
}
void straight(float dist, distanceUnits units) {
  if(units==distanceUnits::mm) {
    dist*=0.0393701;
  }
  if(units==distanceUnits::cm) {
    dist*=0.393701;
  }
  float rotation = distToRot(dist);
  leftGroup.spinFor(rotation, deg, false);
  rightGroup.spinFor(rotation,deg, false);
}
void straight(float dist, float speed) {
  leftGroup.setVelocity(speed,pct);
  rightGroup.setVelocity(speed,pct);
  straight(dist,distanceUnits::in);
}
void straight(float dist) {
  straight(dist,50);
}
void backwards(float dist, float speed) {
  straight(-dist, speed);
}
//gives robot brain trauma
void slam(directionType direction) {
  leftGroup.spin(direction, 100, pct);
  rightGroup.spin(direction, 100, pct);
  wait(0.5,sec);
  while (!((fabs(TopLeft.velocity(pct))<10) || (inertialSensor.acceleration(yaxis)))) {
    wait(5, msec);
  }
  leftGroup.stop();
  rightGroup.stop();
}
void arc(float radius, float angle, turnType side) {
  float radAngle = angle/180*M_PI;
  float leftArc;
  float rightArc;
  if(side==right) {
    leftArc = (radius+drivetrainWidth/2)*radAngle;
    rightArc = (radius-drivetrainWidth/2)*radAngle;
  } else {
    leftArc = -1*(radius-drivetrainWidth/2)*radAngle;
    rightArc = -1*(radius+drivetrainWidth/2)*radAngle;
  }
  leftGroup.setVelocity(sqrtf(leftArc/rightArc)*30,pct);
  rightGroup.setVelocity(sqrtf(rightArc/leftArc)*30,pct);
  leftGroup.spinFor(distToRot(leftArc), deg, false);
  rightGroup.spinFor(distToRot(rightArc), deg, true);
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
  cata = vex::task(toggleCataTask);
  cata.suspend();
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  inertialSensor.resetHeading();
  Catapult1.setStopping(brakeType::coast);
  Catapult2.setStopping(brakeType::coast);
  Catapult1.setVelocity(100,pct);
  Intake.setVelocity(100,pct);
  leftGroup.setVelocity(50, percent);
  rightGroup.setVelocity(50, percent);
}
void brakeAll() {
  leftGroup.setStopping(brakeType::brake);
  rightGroup.setStopping(brakeType::brake);
}
void oppositeSide(void) {
  //start close to left of tile touching wall
  vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net         
  straight(5);
  turnToHeading(48);
  Intake.spinFor(reverse, 0.5, sec);
  turnToHeading(270);
  straight(-7); 
  turnToHeading(225);
  toggleWings();
  straight(-12);
  turnToHeading(170);
  toggleWings();
  turnToHeading(225);
  straight(4);
  turnToHeading(180);
  slam(reverse);
  // score triball not touching black bar
  straight(3);
  turnToHeading(270);
  Intake.spin(forward);
  straight(32);
  turnToHeading(0);
  straight(30);
  straight(-6);
  turnToHeading(90);
  Intake.spin(reverse);
  wait(500, msec);
  Intake.stop();
  turnToHeading(270);
  slam(reverse);
  straight(18);
  turnToHeading(235);
  Intake.spin(forward);
  straight(12);
  straight(-8);
  turnToHeading(90);
  Intake.spinFor(reverse, 0.5, sec);
  turnToHeading(270);
  slam(reverse);
  /*
  straight(16);
  turnToHeading(0);
  Intake.spin(forward);
  straight(5);
  turnToHeading(90);
  Intake.stop();
  Intake.spin(reverse);
  slam(fwd);
  Intake.stop();
  */
  //code for touching elevation
  /*
  smartTurn(135);
  straight(36);
  smartTurn(45);
  toggleWings();
  straight(4);
  */
  
}
void sameSide(void) {
  brakeAll();
  vex::task run(bringCataDown);
  straight(5);
  smartTurn(-45);
  straight(30);
  turnToHeading(0);
  Intake.spin(reverse);
  wait(0.5,sec);
  Intake.stop();
  straight(-7.2);
  turnToHeading(180);
  straight(-12.5);
  straight(12.5);
  turnToHeading(250);
  straight(2);
  // straight(7.2);
  // turnToHeading(90);
  // straight(32);
  // turnToHeading(0);
  // Intake.spin(fwd);
  // straight(24);
  // wait(1,sec);
  // turnToHeading(180);
  // Intake.stop();
  // straight(20);
  // turnToHeading(225);
  // straight(24);
  // turnToHeading(135);
  // straight(24);
  // turnToHeading(90);
  // Intake.spin(reverse);
  // wait(1,sec);
  // Intake.stop();
}
void AWPSameSide(void) {
  brakeAll();
  vex::task run(bringCataDown);
  straight(2);
  smartTurn(-45);
  straight(24);
  turnToHeading(340);
  Intake.spin(reverse);
  wait(1,sec);
  Intake.stop();
  straight(-2);
  turnToHeading(180);
  slam(reverse);
  turnToHeading(180);
  straight(14);
  turnToHeading(150);
  straight(12);
  turnToHeading(120);
  toggleWings();
  straight(-10);
  turnToHeading(240);
  straight(-21);
  turnToHeading(270);
  slam(reverse);
  smartTurn(-10);
  slam(reverse);
  smartTurn(-20);
  brakeAll();
}
void programmingSkills(void) {
  bringCataDown();
  straight(2.4);
  /*
  turnToHeading(315);
  Intake.spin(reverse);
  straight(5);
  Intake.stop();
  straight(-5);
  turnToHeading(135);
  //-10 during prog skills run
  straight(-15);
  turnToHeading(180);
  //slam was straight(8) during prog skills
  slam(reverse);
  //8 during prog skills
  straight(11);
  turnToHeading(250);
  straight(3);
  */
  toggleCata();
  wait(45,sec);
  toggleCata();
  bringCataDown();
  turnToHeading(10);
  straight(60);
  turnToHeading(90);
  straight(36);
  turnToHeading(270);
  toggleWings();
  slam(reverse);

  /*
  turnToHeading(270);
  straight(4);
  turnToHeading(225);
  straight(4);
  toggleCata();
  wait(40,sec);
  stopCata();
  bringCataDown();
  straight(-4);
  turnToHeading(300);
  straight();
  */
}
void testing(void) {
  straight(-8.5);
}
void usercontrol(void) {
  Intake.setVelocity(100,pct);
  int deadband = 5;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonX.pressed(toggleBlocker);
  Controller1.ButtonLeft.pressed(cataMatchLoad);
  while (true) {
    //tank drive

    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * Controller1.Axis1.position());

    // Set the speed of the left motors. If the value is less than the deadband,
    // set it to zero.
    if (fabs(leftMotorSpeed) < deadband) {
      leftGroup.setVelocity(0, percent);
    } else {
      leftGroup.setVelocity(leftMotorSpeed, percent);
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
        Catapult1.spin(directionType::fwd);
        Catapult2.spin(directionType::fwd);
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
    if (fabs(rightMotorSpeed) < deadband) {
      rightGroup.setVelocity(0, percent);
    } else {
      rightGroup.setVelocity(rightMotorSpeed, percent);
    }
    // Spin all drivetrain motors in the forward direction.
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    //test odom
    //odomUpdate();
    //Controller1.Screen.clearLine();
    //Controller1.Screen.print("odom: %.2f inertial %.2f", orientation, inertialSensor.rotation());
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  // Competition.autonomous(oppositeSide);
  Competition.autonomous(sameSide);
  // Competition.autonomous(programmingSkills);
  //Competition.autonomous(AWPSameSide);
  //Competition.autonomous(testing);
  Competition.drivercontrol(usercontrol);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    /*
    if(Controller1.ButtonLeft.pressing()) {
      Competition.autonomous(oppositeSide);
      Controller1.Screen.newLine();
      Controller1.Screen.print("opposite side");
    }
    if(Controller1.ButtonRight.pressing()) {
      Competition.autonomous(sameSide);
      Controller1.Screen.newLine();
      Controller1.Screen.print("same side");
    }*/
    wait(100, msec);
  }
}