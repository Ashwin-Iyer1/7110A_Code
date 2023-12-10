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
bool catatoggle = false;
void stopCata() {
    Catapult.stop();
}
void bringCataDown(float angle) {
  while (rotationSensor.position(deg) < angle) {
        Catapult.spin(fwd);
        wait(20, msec);
      }
      stopCata();
}
int bringCataDown() {
  bringCataDown(270);
  return 1;
}
void cataMatchLoad() {
  bringCataDown(160);
}
int fullCataCycle() {
  Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, 100, velocityUnits::pct, true);
  bringCataDown();
  stopCata();
  return 1;
}
void toggleCata() {
  if(!catatoggle) {
    Catapult.spin(fwd,100,pct);
    catatoggle = true;
  } else {
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
float x = 0;
float y = 0;
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
  float kd = 0.00825;
  float ki = 0;
  float dt = 0.05;
  float t=0;
  double currAngle = inertialSensor.rotation(deg);
  double wantedAngle = currAngle + rot;
  e = wantedAngle-inertialSensor.rotation(deg);
  while ((fabs(e) + fabs(d) > 2) && t<4) {
    e = wantedAngle-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    i += e*dt;
    t+=dt;
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
bool smartTurnOdom(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.75;
  float kd = 0.00825;
  float ki = 0;
  float dt = 0.05;
  float t=0;
  double currAngle = orientation;
  double wantedAngle = currAngle + rot;
  e = wantedAngle-orientation;
  while ((fabs(e) + fabs(d) > 2) && t<4) {
    e = wantedAngle-orientation;
    d = (e-eRec)/dt;
    i += e*dt;
    t+=dt;
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
  float leftPos = (leftGroup.position(deg)-prevLeftRotation)/180*M_PI * wheelRadius;
  float rightPos = (rightGroup.position(deg)-prevRightRotation)/180*M_PI * wheelRadius;
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
  prevLeftRotation = leftGroup.position(deg);
  prevRightRotation = rightGroup.position(deg);
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
  for(double t=0; t<fabs(dist/20); t+=0.025) {
    if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  leftGroup.stop();
  rightGroup.stop();
}
void straight(float dist, float speed) {
  leftGroup.setVelocity(speed,pct);
  rightGroup.setVelocity(speed,pct);
  straight(dist,distanceUnits::in);
}
void straight(float dist) {
  straight(dist,100);
}
void backwards(float dist, float speed) {
  straight(-dist, speed);
}
//gives robot brain trauma
void slam(directionType direction) {
  leftGroup.spin(direction, 100, pct);
  rightGroup.spin(direction, 100, pct);
  wait(0.5,sec);
  while (!((fabs(leftGroup.velocity(pct))<10 || fabs(rightGroup.velocity(pct))<10))) {
    wait(5, msec);
  }
  leftGroup.stop();
  rightGroup.stop();
}
void arc(float radius, float angle, turnType side) {
  float radAngle = angle/180*M_PI;
  float leftArc;
  float rightArc;
  float leftspeed;
  float rightspeed;
  if(side==right) {
    leftArc = (radius+drivetrainWidth/2)*radAngle;
    rightArc = (radius-drivetrainWidth/2)*radAngle;
  } else {
    leftArc = -1*(radius-drivetrainWidth/2)*radAngle;
    rightArc = -1*(radius+drivetrainWidth/2)*radAngle;
  }
  leftspeed = sqrtf(fabs(leftArc/rightArc));
  rightspeed = sqrtf(fabs(rightArc/leftArc));
  if(leftspeed >rightspeed)
    {
      rightspeed = rightspeed/leftspeed;
      leftspeed=1;
  }else{
    leftspeed = leftspeed/rightspeed;
    rightspeed=1;
  }
  
  leftGroup.setVelocity(leftspeed * 75,pct);
  rightGroup.setVelocity(rightspeed * 75,pct);
  leftGroup.spinFor(distToRot(leftArc), deg, false);
  rightGroup.spinFor(distToRot(rightArc), deg, false);
  for(double t=0; t<fabs(fmax(leftArc,rightArc))/18; t+=0.025) {
    if(!leftGroup.isSpinning() && !rightGroup.isSpinning()) break;
    wait(0.025,sec);
  }
  leftGroup.stop();
  rightGroup.stop();
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
  inertialSensor.resetHeading();
  Catapult.setStopping(brakeType::coast);
  Catapult.setVelocity(100,pct);
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
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(2, 15);
  wait(1,sec);
  straight(-24,75);
  Intake.stop();
  toggleWings();
  arc(16.5,-90,right);
  toggleWings();
  turnToHeading(205);
  slam(reverse);
  turnToHeading(180);
  arc(12,180,right);
  straight(30);
  turnToHeading(80);
  Intake.spin(reverse);
  wait(.2, sec);
  straight(13);
  wait(0.3, sec);
  straight(-13);
  turnToHeading(30);
  Intake.spin(fwd);
  toggleBlocker();
  straight(-30);
  smartTurn(25);
  
  // smartTurn(-90);
  /* GET THE FOURTH TRIBALL
  turnToHeading(275);
  Intake.spin(fwd);
  straight(13);
  turnToHeading(70);
  straight(8);
  Intake.stop();
  Intake.spin(reverse);
  wait(0.5, sec);
  Intake.stop();
  turnToHeading(260);
  slam(reverse);
  */
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
void oppositeSideElim(void) {
    //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(2, 50);
  wait(1,sec);
  straight(-24,75);
  Intake.stop();
  toggleWings();
  arc(16.5,-90,right);
  toggleWings();
  turnToHeading(205);
  slam(reverse);
  turnToHeading(180);
  arc(12,180,right);
  straight(30);
  turnToHeading(80);
  Intake.spin(reverse);
  wait(.2, sec);
  straight(-8);
  arc(0,180,right);
  Intake.spin(fwd);
  straight(10);
  wait(0.5,sec);
  straight(-8);
  arc(0,180,right);
  Intake.spin(reverse);
  slam(fwd);
  turnToHeading(30);
  toggleBlocker();
  straight(-30);
}
void sameSide(void) {
  //vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(180,deg); 
  arc(110, 24, left);
  toggleWings();
  Intake.spin(reverse);
  turnToHeading(270);
  straight(-16);
  Intake.stop();
  arc(0,180,right);
  turnToHeading(90);
  slam(reverse);
  toggleWings();
  arc(15, 180, right);
  straight(24);
  turnToHeading(0);
  toggleWings();
  straight(2);
  arc(16.5, -90, right);
  toggleWings();
  turnToHeading(315);
  straight(-8);
  turnToHeading(270);
  Blocker.set(true);
  straight(-30);
  straight(-4,35);
  smartTurn(20);
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
  inertialSensor.setHeading(90,deg);
  arc(16.5,90,left);
  slam(reverse);
  straight(4);
  turnToHeading(0);
  straight(-2);
  toggleWings();
  arc(16.5,-90,right);
  straight(2);
  toggleWings();
  straight(4);
  turnToHeading(315);
  straight(-16);
  turnToHeading(270);
  toggleBlocker();
  straight(-20,100);
  straight(-8,35);
}
void programmingSkills(void) {
  inertialSensor.setHeading(90,deg);
  arc(16.5,90,left);
  slam(reverse);
  straight(9);
  turnToHeading(73.5);
  straight(-4);
  float realOrientation = inertialSensor.heading(deg);
  toggleWings();
  toggleCata();
  wait(33,sec);
  toggleCata();
  toggleWings();
  //bringCataDown(250);
  inertialSensor.setHeading(realOrientation,deg);
  turnToHeading(315);
  arc(120,-17,right);
  turnToHeading(270);
  straight(-54);
  //go to other side
  //toggleWings();
  arc(16.5,-90,right);
  turnToHeading(180);
  slam(reverse);
  //push side triballs in
  //toggleWings();
  arc(10,160,right);
  turnToHeading(160);
  //backup
  toggleWings();
  arc(32,210,left);
  straight(8);
  toggleWings();
  arc(60,-20,right);
  arc(0,-40,right);
  toggleWings();
  arc(60,20,left);
  slam(reverse);
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
  arc(0,90,right);
}
std::string progAlignment[3][3] = {
  {"Safe Opposite Side", "Cool Opposite Side", ""},
  {"Safe Same Side", "Cool Same Side", ""},
  {"Programming Skills", "", ""}
};
typedef void (*callback)();
callback progs[3][3] = {
  {oppositeSide, oppositeSideElim, testing},
  {AWPSameSide, sameSide, testing},
  {programmingSkills, testing, testing}
};
void autonSelection(void) { 
  Brain.Screen.setPenColor(color::white);
  
  for (int i = 1; i <= 2; i ++) {
      //this draws vertical lines. The X stays the same but the Y changes:
      Brain.Screen.drawLine(160*(i),0,160*(i),240);
      //this draws horizontal lines. The X changes but the Y stays the same:
      Brain.Screen.drawLine(0,80*(i),480,80*(i));
  }
  
  for(int x=0; x<3; x++) {
    for(int y=0; y<3; y++) {
      Brain.Screen.printAt(160*x + 80, 80*y + 40, progAlignment[y][x].c_str());
    }
  }
}
void draw_touch() {
    Brain.Screen.setPenColor(color::red);
    //this draws a circle around the place the user is touching or last touched the LCD
    Brain.Screen.drawCircle(Brain.Screen.xPosition(),Brain.Screen.yPosition(),30);
}
void draw_button(int x, int y, int w, int h, color color, char *text) {
  Brain.Screen.setPenColor(color);
  Brain.Screen.drawRectangle(x, y, w, h);
  Brain.Screen.printAt(x+(w/2), y+(h/2), text);
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
    //Controller1.Screen.newLine();
    //Controller1.Screen.print("odom: %.2f inertial %.2f", orientation, inertialSensor.rotation());
    wait(0.025,sec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Brain.Screen.render(true,false); //set VSync (vertical sync) on, automatic refresh to off
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(oppositeSide);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(programmingSkills);
  //Competition.autonomous(AWPSameSide);
  //Competition.autonomous(testing);
  //if(Competition.isEnabled()) selectAuton();
  Competition.drivercontrol(usercontrol);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    Brain.Screen.clearScreen(); //clears the back buffer for drawing, default clear color is black
        autonSelection(); //draws our grid to the back buffer
        Brain.Screen.render(); //flips the back buffer to the screen all at once, preventing flickering
        if (Brain.Screen.pressing()) { //if screen is touched...
            while (Brain.Screen.pressing()) { //wait until the user stops touching the screen
                Brain.Screen.clearScreen(); //while waiting, maintain the grid and draw
                autonSelection();                //a touch indicator around the user's finger
                Brain.Screen.render();
            }
            wait(1, sec); //wait a second for their hand to get a little further away
            Competition.autonomous(progs[Brain.Screen.yPosition()/80][Brain.Screen.xPosition()/160]);
        } else {
            wait(.1, sec);
        }
    wait(100, msec);
  }
}