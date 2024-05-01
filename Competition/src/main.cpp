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
#include <iterator>
#include <vector>
#include "stdarg.h"
#include <cstring>
#include <string.h>
#include "sylib/sylib.hpp"
#include "sylib/addrled.hpp"
#include "vex_motor.h"
#include <cmath>
#include <map>
#include <set>
#include <bits/stdc++.h>
#include <functional>

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
motor      Catapult2 =            motor(PORT9, ratio18_1, false);
motor      Intake =               motor(PORT8, ratio18_1, true);

inertial   inertialSensor =    inertial(PORT20);
rotation   sideTracking =      rotation(PORT19);
rotation   forwardTracking =   rotation(PORT6);
//rotation   hangSensor =        rotation(PORT14);

pneumatics Hang =             pneumatics(Brain.ThreeWirePort.A);
pneumatics Descore =         pneumatics(Brain.ThreeWirePort.B);
pneumatics Wings =           pneumatics(Brain.ThreeWirePort.C);

motor_group leftGroup =  motor_group(FrontLeft, BackLeft, MidLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, MidRight);
motor_group Catapult =   motor_group(Catapult1, Catapult2);

sylib::Addrled* DescoreLEDS;
sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Under3;
sylib::Addrled* Under4;
sylib::Addrled* Top;

float gearRatio = 36.0/48.0;
float wheelDiameter = 3.25;
float wheelRadius = wheelDiameter/2;
float robotRadius = 5.75;
float drivetrainWidth = 11.5;
float sideWheelDist = -7.75;
float forwardWheelDist = 0.42;
float prevForwardRotation = 0;
float prevSideRotation = 0;
float orientation = 0;
float orientationHeading = fmod(orientation+36000,360);

typedef struct _delayedFunction {
    std::function<void()> function;
    float waitTime;
} delayedFunction;

delayedFunction runFunction1;
delayedFunction runFunction2;
delayedFunction runFunction3;

double signum(double num) {
  if(num<0) return -1;
  else if(num>0) return 1;
  else return 0;
}
int delayExecution(void *arg) {
  delayedFunction *func = (delayedFunction *)arg;
  this_thread::sleep_for(func->waitTime);
  func->function();
  return 1;
}
struct Vector2d {
  double x, y;
  Vector2d() {
    x=0;
    y=0;
  }
  Vector2d(double xLocal, double yLocal, double angle=0) {
    x = xLocal*cos(angle/180*M_PI) - yLocal*sin(angle/180*M_PI);
    y = xLocal*sin(angle/180*M_PI) + yLocal*cos(angle/180*M_PI);
  }
  double distance(Vector2d other) {
    return sqrt(pow(x-other.x,2) + pow(y-other.y,2));
  }
  Vector2d operator+(const Vector2d& other) {
    return {this->x+other.x, this->y+other.y};
  }
  Vector2d operator-(const Vector2d& other) {
    return {this->x-other.x, this->y-other.y};
  }
  double operator*(const Vector2d& other) {
    return this->x * other.x + this->y * other.y;
  }
  bool operator==(const Vector2d& other) {
    return (this->x==other.x && this->y==other.y);
  }
};
Vector2d currentPosition;

bool catatoggle = false;
bool hang = true;


void toggleWings() {
  Wings.set(!Wings.value());
}

void toggleDescore() {
  Descore.set(!Descore.value());
}
// int movePTOGear() {
//   if(!hang) {
//     waitUntil(hangSensor.velocity(dps)<0);
//   } else {
//     waitUntil(hangSensor.velocity(dps)>0);
//   }
//   Catapult.stop();
//   return 1;
// }
void toggleHang() {
  Hang.set(!Hang.value());
}

int releaseIntake() {
  Intake.spinFor(-1,rev,false);
  return 1;
}

// int hangSetup() {
//   Catapult.spin(fwd,100,pct);
//   waitUntil(hangSensor.position(deg)>786);
//   Catapult.stop();
//   return 1;
// }
// int sideHang() {
//   Catapult.spin(reverse,100,pct);
//   waitUntil(hangSensor.position(deg)<210);
//   Catapult.stop();
//   return 1;
// }
/*
int handleLEDs() {
      Top -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Top -> cycle(**Top, 10);
      Under1 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under1 -> cycle(**Under1, 10);
      Under2 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under2 -> cycle(**Under2, 10);
      Under3 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under3 -> cycle(**Under3, 10);
      Under4 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under4 -> cycle(**Under4, 10);
  while(false) {
      //DescoreLEDS->set_all(0x000000);
      // Under3->set_all(0xFF00FF);
      // Under1->set_all(0xFF00FF);
      // Under2->set_all(0xFF00FF);
      // Under4->set_all(0xFF00FF);
      // DescoreLEDS -> gradient(0x600000, 0x600002, 0, 0, false, true);
      // DescoreLEDS -> cycle(**DescoreLEDS, 10);

      // Under3 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      // Under3 -> cycle(**Under3, 10);
      
    wait(50,msec);
  }
  return 1;
}*/
int endgameLEDHandler() {
  Top->gradient(0xFF0000,0x0000FF);
  Under1->gradient(0xFF0000,0x0000FF);
  Under2->gradient(0xFF0000,0x0000FF);
  Under3->gradient(0xFF0000,0x0000FF);
  Under4->gradient(0xFF0000,0x0000FF);
  Top->cycle(**Top,10);
  Under1->cycle(**Under1,10);
  Under2->cycle(**Under2,10);
  Under3->cycle(**Under3,10);
  Under4->cycle(**Under4,10);
  //waitUntil(hangSensor.position(deg)>650);
  /*Top->set_all(0xFF00FF);
  Under1->set_all(0xFF00FF);
  Under2->set_all(0xFF00FF);
  Under3->set_all(0xFF00FF);
  Under4->set_all(0xFF00FF);*/
  //waitUntil(hangSensor.position(deg)<300);
  /*Top->set_all(0x00FF00);
  Under1->set_all(0x00FF00);
  Under2->set_all(0x00FF00);
  Under3->set_all(0x00FF00);
  Under4->set_all(0x00FF00);*/
  wait(28,sec);
  Top->gradient(0xFF0000,0xFF0005);
  Under1->gradient(0xFF0000,0xFF0005);
  Under2->gradient(0xFF0000,0xFF0005);
  Under3->gradient(0xFF0000,0xFF0005);
  Under4->gradient(0xFF0000,0xFF0005);
  Top->cycle(**Top,10);
  Under1->cycle(**Under1,10);
  Under2->cycle(**Under2,10);
  Under3->cycle(**Under3,10);
  Under4->cycle(**Under4,10);
  return 1;
}
void endgameWarning() {
  task warn(endgameLEDHandler);
}
vex::task odom;

float distToRot(float dist) {
  return (dist/(wheelDiameter * M_PI)*360) / gearRatio;
}
float rotToDist(float rot) {
  return (rot/360*(wheelDiameter * M_PI)) * gearRatio;
}

bool smartTurn(float rot, float timeout=4) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.285;
  float kd = 0.0225;
  float ki = 0.10;
  float dt = 0.02;
  float t=0;
  double currAngle = inertialSensor.rotation(deg);
  double wantedAngle = currAngle + rot;
  e = wantedAngle-inertialSensor.rotation(deg);
  while (((fabs(e) > 2) || fabs(d) > 20) && t < timeout) {
    e = wantedAngle-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    if(e*eRec < 0) i=0;
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

bool smartTurnOdom(float rot, float timeout=4) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.375;
  float kd = 0.025;
  float ki = 0;
  float dt = 0.02;
  float t=0;
  double currAngle = orientation;
  double wantedAngle = currAngle + rot;
  e = wantedAngle-orientation;
  while ((fabs(e) > 2) && t<timeout) {
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
void setInertial(float d) {
  inertialSensor.setHeading(d, deg);
  inertialSensor.setRotation(d, deg);
}
bool turnToHeading(float heading, float timeout=4) {
  float clockwiseRotation = heading-inertialSensor.heading();
  float closestPath = 0;
  if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
    closestPath = clockwiseRotation;
    if(fabs(clockwiseRotation-360) < fabs(closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  smartTurn(closestPath, timeout);
  return true;
}

void straight(float dist, distanceUnits units, float speed) {
  if(units==distanceUnits::mm) {
    dist*=0.0393701;
  }
  if(units==distanceUnits::cm) {
    dist*=0.393701;
  }
  float rotation = distToRot(dist);
  leftGroup.setVelocity(speed,pct);
  rightGroup.setVelocity(speed,pct);
  leftGroup.spinFor(fwd,rotation, deg, false);
  rightGroup.spinFor(fwd, rotation,deg, false);
  float kp = 0.8;
  float kd = 0;
  float ki = 0;
  float e = 0;
  float eRec = 0;
  float d = 0;
  float i = 0;
  float dt = 0.025;
  float currentRotation = inertialSensor.rotation();
  for(double t=0; t<fabs(dist/20); t+=0.025) {
    e = inertialSensor.rotation() - currentRotation;
    i += e*dt;
    d = (e-eRec)/dt;
    eRec = e;
    float speedDiff = kp*e + kd*d + ki*i;
    leftGroup.setVelocity(speed + speedDiff, pct);
    rightGroup.setVelocity(speed - speedDiff, pct);
    if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  leftGroup.stop(coast);
  rightGroup.stop(coast);
}

void straight(float dist, float speed=80) {
  straight(dist,distanceUnits::in, speed);
}

void backwards(float dist, float speed=100) {
  straight(-dist, speed);
}
void smartStraight(float dist) {
  float rotation = distToRot(dist);
  leftGroup.setVelocity(80,pct);
  rightGroup.setVelocity(80,pct);
  leftGroup.spin(fwd);
  rightGroup.spin(fwd);
  float turnkp = 0.8;
  float turnkd = 0;
  float turnki = 0;
  float straightkp = 0.5;
  float turne = 0;
  float turneRec = 0;
  float turnd = 0;
  float turni = 0;
  float straighte = 0;
  float straighteRec = 0;
  float straightd = 0;
  float straighti = 0;
  float dt = 0.025;
  float currentRotation = inertialSensor.rotation();
  float wantedMotorPosition = leftGroup.position(deg)+rotation;
  for(double t=0; t<fabs(dist/20) && leftGroup.position(deg)<wantedMotorPosition; t+=0.025) {
    turne = inertialSensor.rotation() - currentRotation;
    turni += turne*dt;
    turnd = (turne-turneRec)/dt;
    turneRec = turne;
    straighte = wantedMotorPosition-leftGroup.position(deg);
    straighti += straighte*dt;
    straightd = (straighte-straighteRec)/dt;
    straighteRec = straighte;
    float speedDiff = turnkp*turne + turnkd*turnd + turnki*turni;
    float speed = fmin(80,straightkp*straighte);
    leftGroup.setVelocity(speed + speedDiff, pct);
    rightGroup.setVelocity(speed - speedDiff, pct);
    //if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  leftGroup.stop(coast);
  rightGroup.stop(coast);
}
//gives robot brain trauma
void slam(directionType direction) {
  leftGroup.spin(direction, 80, pct);
  rightGroup.spin(direction, 80, pct);
  wait(0.5,sec);
  while (!((fabs(leftGroup.velocity(pct))<10 || fabs(rightGroup.velocity(pct))<10) || fabs(inertialSensor.acceleration(yaxis))>1)) {
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
  
  leftGroup.setVelocity(leftspeed * 55,pct);
  rightGroup.setVelocity(rightspeed * 55,pct);
  leftGroup.spinFor(distToRot(leftArc), deg, false);
  rightGroup.spinFor(distToRot(rightArc), deg, false);
  wait(0.5,sec);
  for(double t=0; t<fabs(fmax(fabs(leftArc),fabs(rightArc)))/20; t+=0.025) {
    if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  //wait(1,sec);
  leftGroup.stop(coast);
  rightGroup.stop(coast);
}
/*
void moveToPoint(Vector2d point) {
  turnToHeading(atan2())
  smartStraight()
}*/
//Currently inaccurate, will require tuning of driveRotationConstant
void simpleTurn(float deg) {
  /*float dist = driveRotationConstant * gearRatio * deg * robotRadius / wheelRadius;
  leftGroup.spinFor(fwd, dist, vex::deg, false);
  rightGroup.spinFor(reverse, dist, vex::deg, false);*/
  arc(0,deg,right);
}
bool simpleTurnToHeading(float heading) {
  float clockwiseRotation = heading-inertialSensor.heading();
  float closestPath = 0;
  if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
    closestPath = clockwiseRotation;
    if(fabs(clockwiseRotation-360) < fabs(closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  simpleTurn(closestPath);
  return true;
}/*
bool moveToPoint(float xPos, float yPos) {
  turnToHeading(-atanf((xPos-x)/(yPos-y))/M_PI*180);
  straight(sqrtf(pow(xPos-x,2) + pow(yPos-y,2)));
  return true;
}
*/


//In case intake requires the robot to rock back and forth to outtake
int shake() {
  for(int i=0; i<3; i++) {
    straight(1,100);
    straight(-1,100); 
  }
  return 1;
}
//chaining controller
namespace MotionController {
  //moves robot straight for distance in inches
  std::function<void()> straight(float distance, float speed = 80, float heading = inertialSensor.heading()) {
    return [=] (){
      float rotation = distToRot(distance);
      //leftGroup.spin(distance > 0 ? fwd : reverse, speed, pct);
      //rightGroup.spin(distance > 0 ? fwd : reverse, speed, pct);
      float kp = 2;
      float kd = 0;
      float ki = 2;
      float e = 0;
      float eRec = 0;
      float d = 0;
      float i = 0;
      float dt = 0.025;
      float currentRotation = inertialSensor.rotation();
      float wantedMotorPosition = leftGroup.position(deg)+rotation;
      for(double t=0; t<fabs(distance/20) && signum(distance)*(wantedMotorPosition - leftGroup.position(deg))>0; t+=0.025) {
        e = -1 * signum(distance) * (inertialSensor.rotation() - currentRotation);
        i += e*dt;
        d = (e-eRec)/dt;
        eRec = e;
        float speedDiff = kp*e + kd*d + ki*i;
        float leftVoltage = fmin((speed+speedDiff)/8.3, 12);
        float rightVoltage = fmin((speed-speedDiff)/8.3,12);
        leftGroup.spin(distance > 0 ? fwd : reverse, leftVoltage, volt);
        rightGroup.spin(distance > 0 ? fwd : reverse, rightVoltage, volt);
        //leftGroup.setVelocity(speed + speedDiff,pct);
        //rightGroup.setVelocity(speed - speedDiff, pct);
        wait(0.025,sec);
      }
    };
  }
  std::function<void()> straight(directionType direction, float time, timeUnits units=msec, float heading = inertialSensor.heading()) {
    return [=] (){
      //leftGroup.spin(distance > 0 ? fwd : reverse, speed, pct);
      //rightGroup.spin(distance > 0 ? fwd : reverse, speed, pct);
      float kp = 2;
      float kd = 0;
      float ki = 2;
      float e = 0;
      float eRec = 0;
      float d = 0;
      float i = 0;
      float dt = 0.025;
      float currentRotation = inertialSensor.rotation();
      for(double t=0; t<time; t+=0.025) {
        e = (direction==fwd ? -1 : 1) * (inertialSensor.rotation() - currentRotation);
        i += e*dt;
        d = (e-eRec)/dt;
        eRec = e;
        float speedDiff = kp*e + kd*d + ki*i;
        float leftVoltage = fmin(11+(speedDiff)/9, 12);
        float rightVoltage = fmin(11-(speedDiff)/9,12);
        leftGroup.spin(direction, leftVoltage, volt);
        rightGroup.spin(direction, rightVoltage, volt);
        //leftGroup.setVelocity(speed + speedDiff,pct);
        //rightGroup.setVelocity(speed - speedDiff, pct);
        wait(0.025,sec);
      }
    };
  }
  //goes straight in a direction until it collides with something
  std::function<void()> slam(directionType direction, float currentRotation = inertialSensor.rotation()) {
    return [=] (){
      leftGroup.spin(direction, 11, volt);
      rightGroup.spin(direction, 11, volt);
      wait(0.5,sec);
      float kp = 2;
      float kd = 0;
      float ki = 0;
      float e = 0;
      float eRec = 0;
      float d = 0;
      float i = 0;
      float dt = 0.025;
      while (!(fabs(inertialSensor.acceleration(yaxis))>0.5)) {
        e = (direction==fwd ? -1 : 1) * (inertialSensor.rotation() - currentRotation);
        i += e*dt;
        d = (e-eRec)/dt;
        eRec = e;
        float speedDiff = kp*e + kd*d + ki*i;
        float leftVoltage = fmin((80+speedDiff)/9, 11);
        float rightVoltage = fmin((80-speedDiff)/9,11);
        leftGroup.spin(direction, leftVoltage, volt);
        rightGroup.spin(direction, rightVoltage, volt);
        wait(0.025,sec);
      }
    };
  }
  //turns clockwise a given amount
  std::function<void()> turn(float rotation, float timeout = 4, float endError = 2) {
    return [=] (){
      float e = 0;
      float d = 0;
      float i = 0;
      float eRec = 0;
      float dt = 0.02;
      float kp = 1.5;
      float kd = 0.1;
      float ki = 0.0;
      //if(rotation<0) {
      //  float kp = 1.5;
      //  float kd = 0.0;
      //  float ki = 0.0;
      //}
      float t=0;
      double currAngle = inertialSensor.rotation(deg);
      double wantedAngle = currAngle + rotation;
      e = wantedAngle-inertialSensor.rotation(deg);
      while (((fabs(e) > endError) || fabs(d) > 20) && t < timeout) {
        e = wantedAngle-inertialSensor.rotation(deg);
        d = (e-eRec)/dt;
        if(e*eRec < 0) i=0;
        i += e*dt;
        t+=dt;
        eRec = e;
        float speed = kp*e + kd*d + ki*i;
        float voltage = speed/9;
        if(voltage>11) {
          voltage = 11;
        } else if(voltage<-11) {
          voltage = -11;
        } else if(fabs(voltage)<1.5) {
          if(voltage<0) voltage = -1.5;
          else voltage = 1.5;
        }
        leftGroup.spin(fwd, voltage, volt);
        rightGroup.spin(reverse, voltage, volt);
        wait(dt,sec);
      }
    };
  }
  //turns to a specific heading
  std::function<void()> turnToHeading(float heading, float timeout = 4, float endError = 2) {
    return [=]() {
      float clockwiseRotation = heading-inertialSensor.heading();
      float closestPath = 0;
      if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
        closestPath = clockwiseRotation;
        if(fabs(clockwiseRotation-360) < fabs(closestPath)) closestPath-=360;
      } else {
        closestPath = clockwiseRotation+360;
      }
      turn(closestPath, timeout, endError)();
    };
  }
  //moves one side of the drivetrain until the robot has turned a specific amount
  std::function<void()> swing(turnType side, float rotation, float timeout = 4, float endError = 2) {
    return [=]() {
      float e = 0;
      float d = 0;
      float i = 0;
      float eRec = 0;
      float kp = 2.6;
      float kd = 0.2;
      float ki = 0.20;
      float dt = 0.02;
      float t=0;
      double currAngle = inertialSensor.rotation(deg);
      double wantedAngle = currAngle + rotation;
      e = wantedAngle-inertialSensor.rotation(deg);
      if(side==left) {
        leftGroup.stop(brake);
        while (((fabs(e) > endError) || fabs(d) > 20) && t < timeout) {
          e = wantedAngle-inertialSensor.rotation(deg);
          d = (e-eRec)/dt;
          if(e*eRec < 0) i=0;
          i += e*dt;
          t+=dt;
          eRec = e;
          float speed = kp*e + kd*d + ki*i;
          float voltage = speed/9;
          if(voltage>11) {
            voltage = 11;
          } else if(voltage<-11) {
            voltage = -11;
          } else if(fabs(voltage)<2) {
            if(voltage<0) voltage = -2;
            else voltage = 2;
          }
          rightGroup.spin(fwd,-voltage,volt);
          vex::wait(dt,sec);
        }
      } else {
      rightGroup.stop(brake);
      while (((fabs(e) > endError) || fabs(d) > 20) && t < timeout) {
        e = wantedAngle-inertialSensor.rotation(deg);
        d = (e-eRec)/dt;
        if(e*eRec < 0) i=0;
        i += e*dt;
        t+=dt;
        eRec = e;
        float speed = kp*e + kd*d + ki*i;
        float voltage = speed/9;
        if(voltage>11) {
          voltage = 11;
        } else if(voltage<-11) {
          voltage = -11;
        } else if(fabs(voltage)<2) {
          if(voltage<0) voltage = -2;
          else voltage = 2;
        }
        leftGroup.spin(fwd,voltage,volt);
        wait(dt,sec);
      }
    }
    };
  }
  //swings to a heading
  std::function<void()> swingToHeading(turnType side, float heading, directionType direction = fwd, float timeout = 4, float endError = 2) {
    return [=]() {
      float clockwiseRotation = heading-inertialSensor.heading();
      float closestPath = clockwiseRotation;
      if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
        if(fabs(clockwiseRotation-360) < fabs(closestPath)) closestPath-=360;
      } else {
        closestPath = clockwiseRotation+360;
      }
      swing(side, closestPath, timeout, endError)();
    };
  }
  //performs an arc of size specified in degrees around a point of a given side of robot
  std::function<void()> arc(float radius, float rotation, turnType side, float speed=50) {
    return[=]() {
      float radAngle = rotation/180*M_PI;
      float leftArc;
      float rightArc;
      float leftspeed;
      float rightspeed;
      float maxSpeed = fmin(speed+radius,100);
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
      
      float wantedLeftPosition = leftGroup.position(deg) + distToRot(leftArc);
      float wantedRightPosition = rightGroup.position(deg) + distToRot(rightArc);
      leftGroup.spin(leftArc > 0 ? fwd : reverse, leftspeed * maxSpeed, pct);
      rightGroup.spin(rightArc > 0 ? fwd : reverse, rightspeed * maxSpeed,pct);
      wait(0.5,sec);
      for(double t=0; t<fabs(fmax(fabs(leftArc),fabs(rightArc)))/20; t+=0.025) {
        if(signum(leftArc)*(wantedLeftPosition - leftGroup.position(deg)) < 0 && signum(rightArc)*(wantedRightPosition - rightGroup.position(deg)) < 0) break;
        wait(0.025,sec);
      }
    };
  }
 
  void run(std::function<void()> func) {
    func();
    leftGroup.stop();
    rightGroup.stop();
  }
  void chain(const std::vector<std::function<void()>> funcs) {
    for(auto func : funcs) {
      func();
    }
    leftGroup.stop();
    rightGroup.stop();
  }
}
void pre_auton(void) {
  sylib::initialize();
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  inertialSensor.resetHeading();
  Catapult.setStopping(brakeType::brake);
  Catapult.setVelocity(100,pct);
  Intake.setVelocity(100,pct);
  leftGroup.setVelocity(50, percent);
  leftGroup.setStopping(coast);
  rightGroup.setStopping(coast);
  rightGroup.setVelocity(50, percent);
  Wings.set(false);
  Descore.set(false);
  //hangSensor.setPosition(hangSensor.angle(),deg);
}
void brakeAll() {
  leftGroup.setStopping(brakeType::brake);
  rightGroup.setStopping(brakeType::brake);
}
/*
vex::task LED;
int LEDRainbow() {
    std::uint32_t clock = sylib::millis();
    DescoreLEDS -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    DescoreLEDS -> cycle(Under3 -> buffer, 10);

    Under3 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under3 -> cycle(Under3 -> buffer, 10);

    Under1 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under1 -> cycle(Under1 -> buffer, 10);

    Under2 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under2 -> cycle(Under2 -> buffer, 10);
  while (true) {
    sylib::delay_until(&clock, 10);
  }
  return 1;
}*/
void oppositeSide(void) {
  setInertial(90);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  straight(5, 35);
  straight(-28, 60);
  Intake.stop();
  //turnToHeading(55);
  toggleDescore();
  arc(13,-90,right);
  toggleDescore();
  //straight(-22);
  turnToHeading(30);
  MotionController::chain({
    MotionController::straight(-5),
    MotionController::swingToHeading(right,0),
    MotionController::straight(reverse,0.75)
  });
  straight(12);
  MotionController::run(MotionController::turnToHeading(180));
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(17);
  straight(-14);
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::turnToHeading(107),
    MotionController::straight(40),
    MotionController::swingToHeading(right,215,fwd,1.5)
  });
  Intake.stop();
  MotionController::chain({
    MotionController::straight(10),
    MotionController::turnToHeading(270),
    [=]() {
      Intake.spin(reverse);
    },
    toggleWings,
    MotionController::straight(fwd,1),
    toggleWings,
    MotionController::arc(21,-90,right),
    MotionController::straight(-34),
    toggleDescore,
    MotionController::turn(-30)
  });
  //turnToHeading(110);
  //Intake.spin(fwd);
  //straight(44);
}
void oppositeSideUnsafe(void) {
  setInertial(90);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  MotionController::run(MotionController::straight(5, 35));
  wait(0.25,sec);
  straight(-29, 60);
  Intake.stop();
  //turnToHeading(55);
  toggleDescore();
  arc(13,-90,right);
  toggleDescore();
  //straight(-22);
  turnToHeading(30);
  MotionController::chain({
    MotionController::arc(24,-30,right),
    MotionController::straight(reverse,0.5)
  });
  straight(12);
  MotionController::run(MotionController::turn(180));
  Intake.spin(reverse);
  straight(17);
  straight(-14);
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::turnToHeading(113),
    MotionController::straight(44),
    MotionController::turnToHeading(225)
  });
  Intake.spinFor(reverse,0.5,sec);
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::turnToHeading(157),
    MotionController::straight(14),
    MotionController::turnToHeading(270)
  });
  Intake.spin(reverse);
  toggleWings();
  MotionController::run(MotionController::straight(fwd,1)); 
  toggleWings();
  MotionController::chain({
    MotionController::arc(21,-90,right),
    MotionController::straight(-40),
    toggleDescore,
    MotionController::turn(-30)
  });
  //turnToHeading(110);
  //Intake.spin(fwd);
  //straight(44);
}
void oppositeSideElim(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  setInertial(270);
  Intake.spin(fwd);   
  straight(3, 35);
  wait(0.5,sec);
  straight(-26,75);
  toggleWings();
  arc(17.25,-85,right);
  toggleWings();
  straight(-10);
  turnToHeading(180);
  wait(0.3,sec);
  straight(-10);
  // slam(reverse);
  straight(1.5);
  arc(6,180,right);
  straight(30);
  turnToHeading(80);
  Intake.stop();
  Intake.spin(reverse);
  straight(-2.5);
  straight(16.5);
  straight(-17);
  simpleTurn(170);
  Intake.stop();
  Intake.spin(fwd);
  straight(18);
  turnToHeading(60);
  straight(10);
  Intake.stop();
  Intake.spin(reverse);
  straight(-2);
  slam(fwd);
  straight(-10);

  /*5th ball
  turnToHeading(285);
  Intake.spin(fwd);
  straight(30.75);
  toggleWings();
  turnToHeading(270);
  slam(reverse);
  */
}
void sameSide(void) {
 //odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(180);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  MotionController::chain({
    MotionController::straight(27,100),
    [=]() {
      Intake.spin(fwd);
    },
    MotionController::arc(18,15,right,100),
    toggleWings,
    MotionController::turnToHeading(270),
    [=]() {
      Intake.spin(reverse);
    },
    MotionController::straight(fwd,1.25),
    toggleWings,
    MotionController::straight(-8)
  });
  MotionController::chain({
    MotionController::turnToHeading(214.5),
    MotionController::straight((inertialSensor.heading(deg)<265)? -45 : -47),
    toggleDescore,
    MotionController::turnToHeading(90),
    toggleDescore
  });
  MotionController::chain({
    MotionController::turnToHeading(295),
    MotionController::straight(13.5),
    MotionController::swingToHeading(left,270),
    MotionController::straight(17),
    MotionController::straight(2,35)
  });
  brakeAll();

}
void AWPSameSide(void) {
//odom = vex::task(runOdom);
  //orientation = -90;
  vex::task release(releaseIntake);
  setInertial(135);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  straight(8);
  brakeAll();
  toggleDescore();
  wait(0.5,sec);
  MotionController::run(MotionController::swingToHeading(right,90));
  toggleDescore();
  straight(6);
  turnToHeading(300);
  Intake.spin(reverse);
  straight(21);
  turnToHeading(270);
  straight(26);
  straight(2.5,35);
}
void testing(void) {

}
void testPID(void) {
  for(int i=6; i>0; i--) {
    for(int j=0; j<i; j++) {
      MotionController::run(MotionController::turn(360/i));
      //straight(12);
    }
  }
  //straight(70);
}
void testHang(void) {
  releaseIntake();
  //hangSetup();
  straight(24);
  Catapult.spinFor(reverse, 1500, deg);
}
void usercontrol(void) {
  hang = true;
  auto top = sylib::Addrled(22,6,15);
  Top = &top;
  auto under1 = sylib::Addrled(22,7,20);
  Under1 = &under1;
  auto under2 = sylib::Addrled(22,8,20);
  Under2 = &under2;
  
  //pre_auton();
  //currentPosition = {36,12};
  //inertialSensor.setRotation(90,deg);
  //inertialSensor.setHeading(90,deg);
  //orientation = -90;
  //odom = vex::task(runOdom);
  //odom.stop();
  Intake.setVelocity(100,pct);
  int deadband = 1;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleHang);
  Controller1.ButtonR1.pressed(toggleWings);
  Controller1.ButtonR2.pressed(toggleDescore);
  // Controller1.ButtonLeft.pressed(cataMatchLoad);
  //vex::task printCoords(printOdom);
  //vex::task leds(handleLEDs);
  Top -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Top -> cycle(**Top, 10);
  Under2 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Under2 -> cycle(**Under2, 10);
  Under1 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Under1 -> cycle(**Under1, 10);
  // Under1 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  // Under1 -> cycle(**Under1, 10);
  // Under2 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  // Under2 -> cycle(**Under2, 10);
  // Under3 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  // Under3 -> cycle(**Under3, 10);
  // Under4 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  // Under4 -> cycle(**Under4, 10);
  //currentPosition = {0,0};
  
  while (true) {
    // Top -> cycle(**Top, 10);
    // Under1 -> cycle(**Under1, 10);
    // Under2 -> cycle(**Under2, 10);
    // Under3 -> cycle(**Under3, 10);
    // Under4 -> cycle(**Under4, 10);
    //tank drive
    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? -1 : 1) * ((Controller1.Axis3.position()) + (intakeMode ? -1 : 1) * 0.65 * (Controller1.Axis1.position()));
    int rightMotorSpeed = (intakeMode ? -1 : 1) * ((Controller1.Axis3.position()) + (intakeMode ? 1 : -1) * 0.65 * (Controller1.Axis1.position()));

    //cycle based on robot speed
    //addrled.cycle(*addrled, ((leftMotorSpeed + rightMotorSpeed)/10));

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
    //if(hang) {
      // OUTTAKE
      if(Controller1.ButtonR1.pressing()) {
          Catapult.spin(fwd);
      } else if (Controller1.ButtonR2.pressing()) {
          Catapult.spin(directionType::rev);
      } else if(!catatoggle){
          Catapult.stop();
      }
    //} else {
      // Single Catapult Cycle
    //  if(Controller1.ButtonA.pressing()) {
    //    Catapult.spin(fwd);
    //  }
    //}
    if (Controller1.ButtonL1.pressing()) {
        Intake.spin(fwd, 100, pct);
      } else if (Controller1.ButtonL2.pressing()) {
          Intake.spin(reverse, 100, pct);
      }
      else {
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
    //Controller1.Screen.setCursor(0,0);
    //Controller1.Screen.clearLine();
    //Controller1.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, -inertialSensor.rotation(), forwardTracking.position(deg), sideTracking.position(deg));
    wait(0.025,sec);
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  typedef void (*callback)();
  callback progs[5] = {oppositeSide, oppositeSideElim, AWPSameSide, sameSide, testing};
  std::string progNames[5] = {"Safe Opposite", "Cool Opposite", "Safe Same", "Cool Same", "test"};
  //Brain.Screen.render(true,false); //set VSync (vertical sync) on, automatic refresh to off
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  //Competition.autonomous(programmingSkills);
  //Competition.autonomous(oppositeSide);
  //Competition.autonomous(oppositeSideUnsafe);
  //Competition.autonomous(AWPSameSide);
  Competition.autonomous(sameSide);
  //Competition.autonomous(testing);
  //Competition.autonomous(testPID);
  //if(Competition.isEnabled()) selectAuton();
  //Competition.drivercontrol(driverSkills);
  Competition.drivercontrol(usercontrol);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}