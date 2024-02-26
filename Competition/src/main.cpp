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

pneumatics pto =             pneumatics(Brain.ThreeWirePort.A);
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

void stopCata() {
    Catapult.stop();
}

void cataMatchLoad() {
}

int fullCataCycle(int cataSpeed=50) {
  if(!hang){
    Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, cataSpeed, velocityUnits::pct, true);
    stopCata();
  }
  return 1;
}

void toggleCata(int cataSpeed) {
  if(!hang) {
    if(!catatoggle) {
      Catapult.spin(fwd, cataSpeed, pct);
      catatoggle = true;
    } else {
      stopCata();
      catatoggle = false;
    }
  }
}
void toggleCata() {
  toggleCata(60);
}

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
void togglePTO() {
  pto.set(!pto.value());
  if(!hang) {
    //Catapult.spin(fwd);
  }
  hang = !hang;
  if(hang) {
    Catapult.setStopping(brake);
  } else {
    Catapult.setStopping(coast);
  }
  //task move(movePTOGear);
}

int releaseIntake() {
  //Intake.spinFor(-1,rev,false);
  Catapult.spinFor(fwd, 300, deg);
  Catapult.spinFor(reverse, 650, deg);
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

bool turnToHeadingOdom(float heading, float timeout=4) {
  float clockwiseRotation = heading-inertialSensor.heading();
  float closestPath = 0;
  if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
    closestPath = clockwiseRotation;
    if(fabs(clockwiseRotation-360) < (closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  smartTurnOdom(closestPath, timeout);
  return true;
}

void odomUpdate() {
  float orientationDiff = -inertialSensor.rotation()-orientation;
  float radOrientationDiff = orientationDiff*M_PI/180;
  float radOrientation = orientation/180*M_PI;
  orientation = -inertialSensor.rotation();
  orientationHeading = fmod(orientation+36000,360);
  float forwardDiff = ((forwardTracking.position(deg)-prevForwardRotation)/360*(wheelDiameter * M_PI));
  float sideDiff = -((sideTracking.position(deg)-prevSideRotation)/360*(wheelDiameter * M_PI));
  prevForwardRotation = forwardTracking.position(deg);
  prevSideRotation = sideTracking.position(deg);
  //Brain.Screen.newLine();
  //Brain.Screen.print(forwardDiff/radOrientationDiff + forwardWheelDist);
  if(fabs(orientationDiff) < 0.001) {
    Vector2d positionChange(sideDiff,forwardDiff,orientation);
    currentPosition.x += positionChange.x;
    currentPosition.y += positionChange.y;
  } else {
    Vector2d positionChange(2*sin(radOrientationDiff/2)*(sideDiff/radOrientationDiff + sideWheelDist),2*sin(radOrientationDiff/2)*(forwardDiff/radOrientationDiff + forwardWheelDist),orientation + orientationDiff/2);
    currentPosition.x += positionChange.x;
    currentPosition.y += positionChange.y;
  }
}

int printOdom() {
  while(true);
  Controller1.Screen.clearLine();
  Controller1.Screen.print("%.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, -inertialSensor.rotation());
    wait(100,msec);
}

int runOdom() {
  while(true) {
    odomUpdate();
    wait(25,msec);
  }
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

int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

struct Robot {
  double left_motor_speed;
  double right_motor_speed;
  double calcLinearSpeed(float pct) {
    return 55;
  }
};
namespace PascalTriangle {
  std::vector<std::vector<int>> coefficients;
  static void generateCoefficients(int degree) {
    coefficients.clear();
    for (int i = 1; i <= degree; i++) {
      std::vector<int> row;
      for (int j = 0; j <= i; j++) {
        row.push_back(factorial(i)/(factorial(j)*factorial(i-j)));
      }
      coefficients.push_back(row);
    }
  }
}
class BezierCurve {
  private:
    std::vector<Vector2d> points;
     std::map<double,Vector2d> lut;

  public:
    BezierCurve(std::initializer_list<Vector2d> args) {
      for(Vector2d vector : args) {
        points.push_back(vector);
      }
      if(points.size() > PascalTriangle::coefficients.size()+1) {
        PascalTriangle::generateCoefficients(points.size());
      }
    }
    BezierCurve(std::vector<Vector2d> args) {
      for(Vector2d vector : args) {
        points.push_back(vector);
      }
      if(points.size() > PascalTriangle::coefficients.size()+1) {
        PascalTriangle::generateCoefficients(points.size());
      }
    }
    Vector2d calculatePoint(double t) {
      Vector2d result;
        if(t<0) {
          result.x = points.at(0).x;
          result.y = points.at(0).y;
          return result;
        }
        if(t>1) {
          result.x = points.back().x;
          result.y = points.back().y;
          return result;
        }
        std::vector<int> bezierCoefficients = PascalTriangle::coefficients.at(points.size()-2);
        for(int i=0; i<points.size(); i++) {
          result.x += bezierCoefficients.at(i)*pow(1-t,points.size()-1-i)*pow(t,i)*points.at(i).x;
          result.y += bezierCoefficients.at(i)*pow(1-t,points.size()-1-i)*pow(t,i)*points.at(i).y;
        }
        return result; 
    }
    Vector2d calculateTangent(double t) {
      Vector2d result;
        if(t<0) {
          return calculateTangent(0);
        }
        if(t>1) {
          return calculateTangent(1);
        }
        std::vector<int> bezierCoefficients = PascalTriangle::coefficients.at(points.size()-2);
        for(int i=0; i<points.size(); i++) {
          result.x += ((i==points.size()-1 ? 0 :(int(-1*(points.size()-1-i)*bezierCoefficients.at(i))*pow(1-t,int(points.size()-2-i))*pow(t,i)*points.at(i).x)) + (i==0 ? 0 :(bezierCoefficients.at(i)*pow(1-t,int(points.size()-1-i))*i*pow(t,i-1)*points.at(i).x)));
          result.y += ((i==points.size()-1 ? 0 :(int(-1*(points.size()-1-i)*bezierCoefficients.at(i))*pow(1-t,int(points.size()-2-i))*pow(t,i)*points.at(i).y)) + (i==0 ? 0 :(bezierCoefficients.at(i)*pow(1-t,int(points.size()-1-i))*i*pow(t,i-1)*points.at(i).y)));
          Controller1.Screen.print("%.1f, %.1f", result.x, result.y);
        }
        return result; 
    }
      void bake(int intervals) {
    for(int t=0; t<intervals; t++) {
      lut[1.0/intervals*t] = calculatePoint(1.0/intervals*t);
    }
  }
  double intersectCircle(double r, double x, double y) {
    int intervals = lut.size();
    Vector2d circleCenter(x,y);
    std::set<double> intersections;
    for(int i=1; i<lut.size()-1; i++) {
      double beginning = 1.0/intervals*(i-1);
      double midpoint = 1.0/intervals*i;
      double end = 1.0/intervals*(i+1);
      double prevDist = fabs(circleCenter.distance(lut.at(beginning))-r);
      double currDist = fabs(circleCenter.distance(lut.at(midpoint))-r);
      double nextDist = fabs(circleCenter.distance(lut.at(end))-r);
      double closest;
      double closestPoint;
      for(int it=0; it<10; it++) {
        closest = fmin(prevDist, fmin(currDist,nextDist));
        if(prevDist==closest) {
          closestPoint = beginning;
          end = midpoint;
          midpoint = (beginning+end)/2;
          nextDist = currDist;
          currDist = fabs(circleCenter.distance(calculatePoint(midpoint))-r);
        } else if(currDist==closest) {
          closestPoint = midpoint;
          beginning = (beginning+midpoint)/2;
          end = (midpoint+end)/2;
          prevDist = fabs(circleCenter.distance(calculatePoint(beginning))-r);
          nextDist = fabs(circleCenter.distance(calculatePoint(end))-r);
        } else {
          closestPoint = end;
          beginning = midpoint;
          midpoint = (beginning+end)/2;
          prevDist = currDist;
          currDist = fabs(circleCenter.distance(calculatePoint(midpoint))-r);
        }
      }
      if(closest<0.05) intersections.insert(closestPoint);
    }
    return (intersections.size()>0) ? *intersections.rbegin() : -1;
  }
};
class ApproxBezierCurve {
  public:
    std::vector<Vector2d> curvePoints;
    ApproxBezierCurve(std::vector<Vector2d> points) {
      double approxLength = (*points.begin()).distance(*points.rbegin());
      BezierCurve bc(points);
      for(double t=0; t<1; t += 3/approxLength) {
        curvePoints.push_back(bc.calculatePoint(t));
      }
      curvePoints.push_back(bc.calculatePoint(1));
    }
    int closestPointIndex(Vector2d point) {
      double closestDist = 10000000;
      int closestIndex = 0;
      for(int i=0; i<curvePoints.size(); i++) {
        double dist = point.distance(curvePoints.at(i));
        if(dist<closestDist) {
          closestDist = dist;
          closestIndex = i;
        }
      }
      return closestIndex;
    }
    double intersectCircle(Vector2d start, Vector2d end, Vector2d center, double radius) {
      Vector2d difference = {end.x-start.x, end.y-start.y};
      Vector2d normalizedStart = {start.x-center.x, start.y-center.y};
      double a = difference*difference;
      double b = 2 * (normalizedStart*difference);
      double c = (normalizedStart*normalizedStart) - (radius*radius);
      double discriminant = b*b - 4*a*c;

      if(discriminant >=0) {
        discriminant = sqrt(discriminant);
        double t1 = (-b-discriminant) / (2*a);
        double t2 = (-b+discriminant) / (2*a);

        if(t2>=0 && t2<=1) {
          return t2;
        } else if(t1>=0 && t1<=1) {
          return t1;
        }

      }
      return -1;
    }
    Vector2d lookAheadPoint(Vector2d center, double lookAhead) {
      for(int i = closestPointIndex(center); i<curvePoints.size()-1; i++) {
        Vector2d currentPoint = curvePoints.at(i);
        Vector2d nextPoint = curvePoints.at(i+1);
        double t = intersectCircle(currentPoint, nextPoint, center, lookAhead);
        if(t!=-1) {
          return {currentPoint.x + (nextPoint.x - currentPoint.x) * t, currentPoint.y + (nextPoint.y - currentPoint.y) * t};
        }
      }
      return *curvePoints.rbegin();
    }
};
class BoomerangBezier : public ApproxBezierCurve {
  public:
    BoomerangBezier(std::vector<Vector2d> points, double heading) : ApproxBezierCurve(points) {
      Vector2d lastPoint = *points.rbegin();
      Vector2d followUp = {lastPoint.x+12*cos(heading),lastPoint.y+12*sin(heading)};
      curvePoints.push_back(followUp);
      Brain.Screen.print("%.2f, %.2f", followUp.x, followUp.y);
    }
    Vector2d lookAheadPoint(Vector2d center, double lookAhead) {
      for(int i = closestPointIndex(center); i<curvePoints.size()-1; i++) {
        Vector2d currentPoint = curvePoints.at(i);
        Vector2d nextPoint = curvePoints.at(i+1);
        double t = intersectCircle(currentPoint, nextPoint, center, lookAhead);
        if(t!=-1) {
          return {currentPoint.x + (nextPoint.x - currentPoint.x) * t, currentPoint.y + (nextPoint.y - currentPoint.y) * t};
        }
      }
      return curvePoints.at(curvePoints.size()-2);
    }
};
void followApproxBezier(std::vector<Vector2d> points, vex::directionType direction = fwd, double lookAhead = 18) {
  points.insert(points.begin(),currentPosition);
  ApproxBezierCurve spline(points);
  Vector2d ogtan = spline.curvePoints.at(1)-spline.curvePoints.at(0);
  turnToHeading(fmod(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + (direction==fwd ? 90 : 270),360),0.5);
  double fullDistance = currentPosition.distance(*points.rbegin());
  double timeStep=0.05;
  double t=0;
  for (double distance=currentPosition.distance(*points.rbegin()); distance>=4 && t<fullDistance ; distance=currentPosition.distance(*points.rbegin())) {
      Vector2d desiredPoint;
      if(distance>lookAhead)
      {
        desiredPoint = spline.lookAheadPoint(currentPosition, lookAhead);
      } else {
        desiredPoint = *points.rbegin();
      }
      Vector2d tangent = {desiredPoint.x-currentPosition.x, desiredPoint.y-currentPosition.y};
      double angle = std::atan2(tangent.y, tangent.x); //angle of tangent vector in radians
      // Brain.Screen.print(angle);
      // Brain.Screen.newLine
      // Brain.Screen.print(orientationHeading);
      //double theta = orientationHeading/180*M_PI - (direction==fwd? 0 : 180);
      //double x = (desiredPoint.x/tan(theta) + currentPosition.x*tan(theta) + desiredPoint.x - currentPosition.x)/(tan(theta) + 1/tan(theta));
      //double y = tan(theta)*(x-currentPosition.x) + currentPosition.y;
      //double curvature = 2*desiredPoint.distance({x,y})/(lookAhead*lookAhead);
      double theta = orientationHeading -  (direction==fwd ? M_PI/2 : 3*M_PI/2);
      double side = (sin(theta) * (desiredPoint.x - currentPosition.x) - cos(theta) * (desiredPoint.y - currentPosition.y)) < 0 ? 1 : -1;
      // calculate center point and radius
      double a = -tan(theta);
      double c = tan(theta) * currentPosition.x - currentPosition.y;
      double x = fabs(a * desiredPoint.x + desiredPoint.y + c) / sqrt((a * a) + 1);
      double d = desiredPoint.distance(currentPosition);
      double curvature = side * ((2 * x) / (d * d));

    /*
      angle = angle / M_PI * 180 - (direction==fwd ? 90 : 270); //convert to degrees
      double angle_difference = fmod(angle - orientationHeading, 360);
      if(fabs(angle_difference) > 180) angle_difference = (360-angle_difference*((angle_difference>0)? 1 : -1));
      e = angle_difference;
      d = (e-eRec)/timeStep;
      i += (e*timeStep);
      eRec = e;*/
      //distance = sqrt((desired_position.x-x)*(desired_position.x-x) + (desired_position.y-y)*(desired_position.y-y));
      //float speed = 2 * sinf((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth/2) * (rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60/((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth) + drivetrainWidth/2);
      Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      Brain.Screen.print("%.1f, %.1f", currentPosition.x, currentPosition.y);
      //1 pct speed diff in 0.1 seconds = 0.3 degrees change
      //double speed_difference = ki*i + kd*d + kp*e;

      double avg_speed = fabs(50-15*sqrt(fabs(curvature)));
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(drivetrainWidth))/2 : avg_speed*(2-curvature*(drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(drivetrainWidth))/2 : avg_speed*(2+curvature*(drivetrainWidth))/2;
      //max_speed = fabs(80-4*fabs(speed_difference));
      //double left_speed = (direction == fwd) ? max_speed-speed_difference : max_speed+speed_difference;
      //double right_speed = (direction == fwd) ? max_speed+speed_difference : max_speed-speed_difference;

      // Send motor commands to the robot
      // Brain.Screen.print("left: ");
      // Brain.Screen.print(left_speed);
      // Brain.Screen.print("right: ");
      // Brain.Screen.print(right_speed);
      // Brain.Screen.newLine();

      //Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      //Brain.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f", desiredPoint.x, desiredPoint.y, x, side, theta);
      // Move forward in time
      
      leftGroup.spin(direction,left_speed,pct);
      rightGroup.spin(direction,right_speed,pct);
      t+=timeStep;
      wait(timeStep, sec);
    }
}
void followApproxBezier(std::vector<Vector2d> points, double heading, vex::directionType direction = fwd, double lookAhead = 18) {
  points.insert(points.begin(),currentPosition);
  heading = heading*M_PI/180;
  BoomerangBezier spline(points, (direction==fwd ? heading+M_PI/2 : heading-M_PI/2));
  Vector2d ogtan = spline.curvePoints.at(1)-spline.curvePoints.at(0);
  turnToHeading(fmod(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + (direction==fwd ? 90 : 270),360),0.5);
  double fullDistance = currentPosition.distance(*points.rbegin());
  double timeStep=0.05;
  double t=0;
  for (double distance=currentPosition.distance(*points.rbegin()); distance>=4 && t<fullDistance ; distance=currentPosition.distance(*points.rbegin())) {
      Vector2d desiredPoint;
      if(distance>lookAhead)
      {
        Brain.Screen.newLine();
        //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
        
        desiredPoint = spline.lookAheadPoint(currentPosition, lookAhead);
        Brain.Screen.print("%.1f, %.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, desiredPoint.x, desiredPoint.y);
      } else {
        desiredPoint = points.at(points.size()-1);
      }
      Vector2d tangent = {desiredPoint.x-currentPosition.x, desiredPoint.y-currentPosition.y};
      double angle = std::atan2(tangent.y, tangent.x); //angle of tangent vector in radians
      // Brain.Screen.print(angle);
      // Brain.Screen.newLine
      // Brain.Screen.print(orientationHeading);
      //double theta = orientationHeading/180*M_PI - (direction==fwd? 0 : 180);
      //double x = (desiredPoint.x/tan(theta) + currentPosition.x*tan(theta) + desiredPoint.x - currentPosition.x)/(tan(theta) + 1/tan(theta));
      //double y = tan(theta)*(x-currentPosition.x) + currentPosition.y;
      //double curvature = 2*desiredPoint.distance({x,y})/(lookAhead*lookAhead);
      double theta = orientationHeading -  (direction==fwd ? M_PI/2 : 3*M_PI/2);
      double side = (sin(theta) * (desiredPoint.x - currentPosition.x) - cos(theta) * (desiredPoint.y - currentPosition.y)) < 0 ? 1 : -1;
      // calculate center point and radius
      double a = -tan(theta);
      double c = tan(theta) * currentPosition.x - currentPosition.y;
      double x = fabs(a * desiredPoint.x + desiredPoint.y + c) / sqrt((a * a) + 1);
      double d = desiredPoint.distance(currentPosition);
      double curvature = side * ((2 * x) / (d * d));

    /*
      angle = angle / M_PI * 180 - (direction==fwd ? 90 : 270); //convert to degrees
      double angle_difference = fmod(angle - orientationHeading, 360);
      if(fabs(angle_difference) > 180) angle_difference = (360-angle_difference*((angle_difference>0)? 1 : -1));
      e = angle_difference;
      d = (e-eRec)/timeStep;
      i += (e*timeStep);
      eRec = e;*/
      //distance = sqrt((desired_position.x-x)*(desired_position.x-x) + (desired_position.y-y)*(desired_position.y-y));
      //float speed = 2 * sinf((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth/2) * (rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60/((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth) + drivetrainWidth/2);
      
      //1 pct speed diff in 0.1 seconds = 0.3 degrees change
      //double speed_difference = ki*i + kd*d + kp*e;

      double avg_speed = fabs(60-35*sqrt(fabs(curvature)));
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(drivetrainWidth))/2 : avg_speed*(2-curvature*(drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(drivetrainWidth))/2 : avg_speed*(2+curvature*(drivetrainWidth))/2;
      //max_speed = fabs(80-4*fabs(speed_difference));
      //double left_speed = (direction == fwd) ? max_speed-speed_difference : max_speed+speed_difference;
      //double right_speed = (direction == fwd) ? max_speed+speed_difference : max_speed-speed_difference;

      // Send motor commands to the robot
      // Brain.Screen.print("left: ");
      // Brain.Screen.print(left_speed);
      // Brain.Screen.print("right: ");
      // Brain.Screen.print(right_speed);
      // Brain.Screen.newLine();

      //Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      //Brain.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f", desiredPoint.x, desiredPoint.y, x, side, theta);
      // Move forward in time
      
      leftGroup.spin(direction,left_speed,pct);
      rightGroup.spin(direction,right_speed,pct);
      t+=timeStep;
      wait(timeStep, sec);
    }
}
void followBezier(std::vector<Vector2d> points, vex::directionType direction = fwd, double lookAhead=18) {
  points.insert(points.begin(),currentPosition);
  BezierCurve spline(points);
  double fullDistance = currentPosition.distance(*points.rbegin());
  spline.bake(int(fullDistance/3));
  Vector2d ogtan = spline.calculateTangent(0);
  //turnToHeading(fmod(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + (direction==fwd ? 90 : 270),360));
  //Controller1.Screen.print(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + 90);
  //leftGroup.spin(fwd);
  // leftGroup.setVelocity(20, pct);
  //rightGroup.spin(fwd);
  // rightGroup.setVelocity(20, pct);
  float kp = 0.4;
  float ki = 0.0;
  float kd = 0.0;
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float t = 0;
  // Controller1.Screen.print("DOES THIS WORK");
  // Controller1.Screen.print("STILL WORKING");
  //wait(0.5,sec);
  //wait(5,sec);
  //wait(5,sec);
  double timeStep = 0.05;
    for (double distance=currentPosition.distance(*points.rbegin()); distance>=4 && t<fullDistance ; distance=currentPosition.distance(*points.rbegin())) {
      Vector2d desiredPoint;
      if(distance>lookAhead)
      {
        desiredPoint = spline.calculatePoint(spline.intersectCircle(lookAhead,currentPosition.x,currentPosition.y));
      } else {
        desiredPoint = *points.rbegin();
      }
      Vector2d tangent = {desiredPoint.x-currentPosition.x, desiredPoint.y-currentPosition.y};
      double angle = std::atan2(tangent.y, tangent.x); //angle of tangent vector in radians
      // Brain.Screen.print(angle);
      // Brain.Screen.newLine
      // Brain.Screen.print(orientationHeading);
      //double theta = orientationHeading/180*M_PI - (direction==fwd? 0 : 180);
      //double x = (desiredPoint.x/tan(theta) + currentPosition.x*tan(theta) + desiredPoint.x - currentPosition.x)/(tan(theta) + 1/tan(theta));
      //double y = tan(theta)*(x-currentPosition.x) + currentPosition.y;
      //double curvature = 2*desiredPoint.distance({x,y})/(lookAhead*lookAhead);
      double theta = orientationHeading -  (direction==fwd ? M_PI/2 : 3*M_PI/2);
      double side = (sin(theta) * (desiredPoint.x - currentPosition.x) - cos(theta) * (desiredPoint.y - currentPosition.y)) < 0 ? 1 : -1;
      // calculate center point and radius
      double a = -tan(theta);
      double c = tan(theta) * currentPosition.x - currentPosition.y;
      double x = fabs(a * desiredPoint.x + desiredPoint.y + c) / sqrt((a * a) + 1);
      double d = desiredPoint.distance(currentPosition);
      double curvature = side * ((2 * x) / (d * d));

    /*
      angle = angle / M_PI * 180 - (direction==fwd ? 90 : 270); //convert to degrees
      double angle_difference = fmod(angle - orientationHeading, 360);
      if(fabs(angle_difference) > 180) angle_difference = (360-angle_difference*((angle_difference>0)? 1 : -1));
      e = angle_difference;
      d = (e-eRec)/timeStep;
      i += (e*timeStep);
      eRec = e;*/
      //distance = sqrt((desired_position.x-x)*(desired_position.x-x) + (desired_position.y-y)*(desired_position.y-y));
      //float speed = 2 * sinf((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth/2) * (rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60/((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth) + drivetrainWidth/2);
      Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      Brain.Screen.print("%.1f, %.1f", currentPosition.x, currentPosition.y);
      //1 pct speed diff in 0.1 seconds = 0.3 degrees change
      //double speed_difference = ki*i + kd*d + kp*e;

      double avg_speed = fabs(50-25*sqrt(fabs(curvature)));
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(drivetrainWidth))/2 : avg_speed*(2-curvature*(drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(drivetrainWidth))/2 : avg_speed*(2+curvature*(drivetrainWidth))/2;
      //max_speed = fabs(80-4*fabs(speed_difference));
      //double left_speed = (direction == fwd) ? max_speed-speed_difference : max_speed+speed_difference;
      //double right_speed = (direction == fwd) ? max_speed+speed_difference : max_speed-speed_difference;

      // Send motor commands to the robot
      // Brain.Screen.print("left: ");
      // Brain.Screen.print(left_speed);
      // Brain.Screen.print("right: ");
      // Brain.Screen.print(right_speed);
      // Brain.Screen.newLine();

      //Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      //Brain.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f", desiredPoint.x, desiredPoint.y, x, side, theta);
      // Move forward in time
      
      leftGroup.spin(direction,left_speed,pct);
      rightGroup.spin(direction,right_speed,pct);
      t+=timeStep;
      wait(timeStep, sec);
    }
    //straight(lookAhead);

    // Stop the robot when the path is complete
    Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      Brain.Screen.print("%.1f, %.1f", currentPosition.x, currentPosition.y);
      
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
      float kd = 0.11;
      float ki = 0.20;
      if(rotation<0) {
        float kp = 1.5;
        float kd = 0.11;
        float ki = 0.20;
      }
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
        } else if(fabs(voltage)<2) {
          if(voltage<0) voltage = -2;
          else voltage = 2;
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
  std::function<void()> arc(float radius, float rotation, turnType side) {
    return[=]() {
      float radAngle = rotation/180*M_PI;
      float leftArc;
      float rightArc;
      float leftspeed;
      float rightspeed;
      float maxSpeed = fmin(50+radius,100);
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
  std::function<void()> arc(float radius, float rotation, turnType side, float timeout) {
    return[=]() {
      float radAngle = rotation/180*M_PI;
      float leftArc;
      float rightArc;
      float leftspeed;
      float rightspeed;
      float maxSpeed = fmin(50+radius,100);
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
      for(double t=0; t<timeout; t+=0.025) {
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
  sideTracking.resetPosition();
  forwardTracking.resetPosition();
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
//odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(90);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  straight(3, 35);
  //pp stuff
  wait(0.25,sec);
  turnToHeading(88.5);
  wait(0.5,sec);
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
  //smartTurn(-25);
  //turnToHeading(0);
  smartTurn(180);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(17);
  straight(-14);
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::turnToHeading(113),
    MotionController::straight(38),
    MotionController::swingToHeading(right,270,fwd,1.5)
  });
  Intake.spin(reverse);
  MotionController::chain({
    MotionController::straight(fwd,1),
    MotionController::straight(-10),
    MotionController::turnToHeading(31),
    MotionController::straight(38)
  });
  //turnToHeading(110);
  //Intake.spin(fwd);
  //straight(44);
}
void oppositeSideUnsafe(void) {
  auto top = sylib::Addrled(22,8,22);
  Top = &top;
  auto block = sylib::Addrled(22,3,40);
  DescoreLEDS = &block;
  auto und1 = sylib::Addrled(22,7,9);
  Under1 = &und1;
  auto und2 = sylib::Addrled(22,6,11);
  Under2 = &und2;
  auto und3 = sylib::Addrled(22,5,19);
  Under3 = &und3;
  auto und4 = sylib::Addrled(22,4,19);
  Under4 = &und4;
  Top->gradient(0xC05DBF,0xFF6AAB);
  Under1->gradient(0xC05DBF,0xFF6AAB);
  Under2->gradient(0xC05DBF,0xFF6AAB);
  Under3->gradient(0xC05DBF,0xFF6AAB);
  Under4->gradient(0xC05DBF,0xFF6AAB);
  Top->cycle(**Top,10);
  Under1->cycle(**Under1,10);
  Under2->cycle(**Under2,10);
  Under3->cycle(**Under3,10);
  Under4->cycle(**Under4,10);
//orientation = -90;
  setInertial(90);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  straight(3, 35);
  //pp stuff
  wait(0.25,sec);
  turnToHeading(88.5);
  wait(0.5,sec);
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
  //smartTurn(-25);
  //turnToHeading(0);
  smartTurn(180);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(17);
  straight(-14);
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::turnToHeading(113),
    MotionController::straight(38),
    MotionController::swingToHeading(right,270,fwd,1.5)
  });
  Intake.spin(reverse);
  MotionController::chain({
    MotionController::straight(fwd,1.25),
    MotionController::straight(6)
  });
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
  Intake.spin(fwd);
  MotionController::chain({
    MotionController::straight(42,100),
    MotionController::arc(12,20,right),
    MotionController::straight(-38),
    MotionController::turnToHeading(295)
  });
  Intake.spin(reverse);
  wait(1,sec);
  Intake.stop();
  MotionController::chain({
    MotionController::turnToHeading(115),
    MotionController::straight(18.5),
    MotionController::swingToHeading(right,135),
    toggleDescore,
    MotionController::straight(-15),
    MotionController::turnToHeading(90,1),
    toggleDescore,
    MotionController::swingToHeading(left,120,fwd,1),
    MotionController::straight(-16),
    MotionController::turnToHeading(90,1),
    MotionController::straight(-8)
  });
  brakeAll();

}
void AWPSameSide(void) {
//odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(135);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  straight(13);
  toggleDescore();
  MotionController::run(MotionController::swingToHeading(right,90));
  toggleDescore();
  turnToHeading(120);
  Catapult.spinFor(1700,deg,false);
  straight(-18);
  turnToHeading(90);
  straight(-28);
  straight(-6,20);
}
void oldProg(void) {
   inertialSensor.setHeading(90,deg);
  
  arc(18,90,left);
  slam(reverse);
  straight(10.5);
  turnToHeading(73.5);
  straight(-3);
  turnToHeading(69);
  toggleDescore();
  toggleCata();
  float realOrientation = inertialSensor.heading(deg);
  wait(5,sec);
  toggleCata();
  toggleDescore();
  //bringCataDown(250);
  //go to other side
  inertialSensor.setHeading(realOrientation,deg);
  straight(4);
  turnToHeading(315);
  arc(120,-17,right);
  turnToHeading(270);
  
  straight(-54);
  //go to other side
  //toggleWings();
  arc(22.5,-90,right);
  turnToHeading(180);
  slam(reverse);
  straight(4);
  slam(reverse);
  //push side triballs in
  // arc(21,-90,right);
  // turnToHeading(180);
  // slam(reverse);
  //toggleWings();
  //backup
  arc(11.5,160,right);
  turnToHeading(160);
  //backup
  straight(-22);
  toggleDescore();
  arc(15,102.5,left);
  slam(reverse);
  straight(6);
  slam(reverse);
  straight(8);
  toggleDescore();
  arc(50,40,right);
  turnToHeading(0);
  straight(40);
  turnToHeading(315);
  toggleDescore();
  arc(70,-45,right);
  straight(10);
  toggleDescore();
  straight(50);
}
void odomSkills(void) {
  inertialSensor.setHeading(90,deg);
  inertialSensor.setRotation(90,deg);
  orientation = -90;
  odom = task(runOdom);
  currentPosition = {36,12};
  wait(0.1,sec);
  followApproxBezier({{18,18}, {12, 38}}, 180, reverse,18);
  slam(reverse);
  followApproxBezier({{24,24}},fwd,8);
  leftGroup.stop();
  rightGroup.stop();
  //togglePTO();
  
  //arc(30,-20,left);
  turnToHeading(70);
  straight(-4);
  //toggleDescore();
  //toggleCata();
  //float currentRotation = inertialSensor.rotation(deg);
  //float currentHeading = inertialSensor.heading(deg);
  //odom.suspend();
  wait(2,sec);
  //toggleCata();
  //toggleDescore();
  //togglePTO();
  //setInertial(currentRotation);
  //odom.resume();
  followBezier({{32.234,7.058},{87.837,12.798},{120.695,13.478},{132.679,29.167},{132.630,40.139}},fwd,12);
  //followBezier({{32,10}});
  /*
  followBezier({{110.742,30.310},{80.786,48.118}},reverse);
  turnToHeading(255);
  toggleWings();
  followBezier({{98.359,59.440},{114.398,59.322}});
  toggleWings();
  followBezier({{90.811,62.978},{79.135,73.120}},reverse);
  turnToHeading(260);
  toggleWings();
  followBezier({{98.123,79.253},{113.219,79.253}});
  toggleWings();
  followBezier({{86.447,82.791},{82.319,108.029}},reverse);
  toggleWings();
  followBezier({{99.892,92.580},{113.455,89.749}});
  toggleWings();
  followBezier({{106.025,94.585},{108.147,130.673}},reverse);
  turnToHeading(90);
  straight(34);
  Controller1.Screen.print("hanging");*/
}
void roomba(void) {
    /*MotionController::chain({
    MotionController::straight(-2),
    MotionController::swingToHeading(right,295),
    MotionController::straight(52),
    MotionController::turnToHeading(260)
  });*/
  
  MotionController::chain({
    MotionController::swingToHeading(left,115),
    MotionController::straight(-30),
    MotionController::swingToHeading(left,270),
    toggleDescore,
    MotionController::straight(-28)
  });
  MotionController::chain({
    MotionController::straight(16),
    toggleDescore,
    MotionController::swing(right,180),
    MotionController::straight(28)
  });
  MotionController::chain({
    MotionController::straight(-16),
    MotionController::swing(left,180),
    toggleDescore,
    MotionController::straight(-28)
  });
  MotionController::chain({
    MotionController::straight(16),
    toggleDescore,
    MotionController::swingToHeading(right,0),
    MotionController::straight(18),
    MotionController::swingToHeading(right,90),
    MotionController::straight(40),
    MotionController::swingToHeading(left,180),
    MotionController::slam(fwd),
    MotionController::arc(30,20,left),
    MotionController::swingToHeading(right,150),
    MotionController::arc(30,30,right),
    MotionController::slam(fwd)
  });
}
void progBeginning() {
    hang = false;
  //vex::task intake(releaseIntake);
  inertialSensor.setHeading(90,deg);
  inertialSensor.setRotation(90,deg);
  orientation = -90;
  //odom = task(runOdom);
  currentPosition = {36,12};
  MotionController::chain({
    MotionController::arc(18,90,left),
    MotionController::straight(reverse,0.5)
  });
  MotionController::chain({
    MotionController::turnToHeading(180,0.25),
    MotionController::straight(10.5,80,180),
    MotionController::turnToHeading(71)
  });
  MotionController::run(MotionController::straight(-2));
  MotionController::run(MotionController::turnToHeading(71));
  toggleDescore();
  toggleCata();
    float currentRotation = inertialSensor.rotation(deg);
  float currentHeading = inertialSensor.heading(deg);
  vex::wait(26,sec);
  toggleCata();
  toggleDescore();
  togglePTO();
  setInertial(currentHeading);
  MotionController::chain({
    MotionController::turnToHeading(305),
    MotionController::straight(-25),
    MotionController::swingToHeading(right,270),
    MotionController::straight(-62),
    /*
    MotionController::swingToHeading(right,225,fwd,0.75),
    MotionController::straight(-16),
    MotionController::swingToHeading(right,180,fwd,0.75),
    MotionController::straight(reverse,0.5),
    MotionController::straight(6,80,180),
    MotionController::turnToHeading(180,0.5),
    MotionController::straight(reverse,0.75,msec,180),
    MotionController::straight(14)
    */
  });
}
void programmingSkills(void) {
  hang = false;
  Catapult.setStopping(coast);
  //vex::task intake(releaseIntake);
  inertialSensor.setHeading(90,deg);
  inertialSensor.setRotation(90,deg);
  orientation = -90;
  //odom = task(runOdom);
  currentPosition = {36,12};
  MotionController::chain({
    MotionController::arc(13,90,left),
    MotionController::straight(reverse,0.5)
  });
  MotionController::chain({
    MotionController::turnToHeading(180,0.25),
    MotionController::straight(10.5,80,180),
    MotionController::turnToHeading(69)
  });
  MotionController::run(MotionController::straight(-3));
  MotionController::run(MotionController::turnToHeading(69,0.5,0.5));
  toggleDescore();
  toggleCata();
  float currentRotation = inertialSensor.rotation(deg);
  float currentHeading = inertialSensor.heading(deg);
  vex::wait(24.5,sec);
  //waitUntil(Controller1.ButtonA.pressing());
  toggleCata();
  toggleDescore();
  //togglePTO();
  setInertial(currentHeading);
  MotionController::chain({
    MotionController::straight(2),
    MotionController::turnToHeading(310),
    MotionController::straight(-23.5),
    MotionController::swingToHeading(right,270),
    MotionController::straight(-60),
    MotionController::arc(18,-90,right),
    MotionController::straight(reverse,1),
    MotionController::straight(12),
    MotionController::straight(reverse,1),
    MotionController::straight(12),
    MotionController::straight(reverse,1),
    // MotionController::swingToHeading(right,225,fwd,0.75),
    // MotionController::straight(-16),
    // MotionController::swingToHeading(right,180,fwd,0.3),
    // MotionController::straight(reverse,0.5),
    // MotionController::straight(8,80,180),
    // MotionController::straight(reverse,1),
    MotionController::straight(14)
  });
  MotionController::chain({ 
    MotionController::swingToHeading(right,105,fwd,1),
    MotionController::straight(-24),
    MotionController::swingToHeading(left,265,fwd,1),
    toggleDescore,
    MotionController::straight(reverse,1),
    MotionController::straight(14),
    MotionController::straight(reverse,1)
  });
  /*
  MotionController::chain({
    MotionController::straight(4),
    toggleDescore,
    MotionController::turnToHeading(90),
    MotionController::straight(-8),
    MotionController::swing(left,60),
    MotionController::straight(-5),
    MotionController::swing(left,120),
    toggleDescore,
    MotionController::straight(reverse,2)
  });
  */
 MotionController::chain({
    MotionController::straight(4),
    toggleDescore,
    MotionController::turnToHeading(90),
    MotionController::straight(-3),
    MotionController::swing(left,180,0.5),
    MotionController::swingToHeading(left,270,fwd,1),
    toggleDescore,
    MotionController::straight(reverse,1),
    MotionController::straight(12),
    MotionController::straight(reverse,1)
  });
  
  //vex::task hang(hangSetup);
  /*MotionController::chain({
    MotionController::arc(35,45,right),
    toggleDescore,  
    MotionController::swingToHeading(right,75),
    MotionController::straight(38),
    MotionController::swingToHeading(right,180),
    MotionController::straight(-4),
    MotionController::straight(fwd,1)
  });*/
  MotionController::chain({
    MotionController::straight(2),
    toggleDescore,
    MotionController::turnToHeading(7),
    MotionController::straight(38),
    MotionController::turnToHeading(90),
    MotionController::arc(16,90,right),
    MotionController::straight(fwd,1),
    MotionController::straight(-8),
    MotionController::straight(fwd,1),
    MotionController::straight(-8),
    MotionController::turnToHeading(0),
    MotionController::straight(reverse,1)
  });

  // MotionController::chain({
  //   MotionController::swingToHeading(right,120),
  //   MotionController::straight(-40),
  //   MotionController::swingToHeading(left,90),
  //   MotionController::straight(24),
  //   MotionController::swingToHeading(right,135),
  //   MotionController::straight(24),
  //   MotionController::swingToHeading(right,180,fwd,0.3),
  //   MotionController::straight(fwd,1)
  // });
  /*if(hangSensor.position(deg)>400) {
    MotionController::chain({
      MotionController::straight(-4),
      MotionController::turnToHeading(315),
      MotionController::straight(24),
      MotionController::swingToHeading(left,270),
      MotionController::straight(40)
    });
    vex::task gaming(sideHang);
  } else {*/
    MotionController::chain({
      MotionController::straight(4),
      MotionController::swingToHeading(right,75),
      MotionController::straight(-40),
      MotionController::swing(right,-165),
      toggleDescore,
      MotionController::straight(reverse,1),
      MotionController::straight(6),
      toggleDescore
    });
  //}
  
}
void testing(void) {
  odom = vex::task(runOdom);
  orientation = -90;
  setInertial(90);
  currentPosition = {12,36};
  
  // followPath({
  //   {48,70},
  //   {36,104},
  //   {40,130},
  //   {108,130}
  // });

    // Move the robot along a Bezier spline to the target position
    followBezier({ {23, -2}, { 121, 8 }, {144,34}});
    //controller.moveRobot({{ 36, 12 }});
    wait(1, sec);
    //left 0, right 10 for 1 second = 21 degrees
  // leftGroup.spin(fwd);
  // rightGroup.spin(fwd);
  // leftGroup.setVelocity(90, pct);
  // rightGroup.setVelocity(100, pct);
  // wait(1, sec);
  // leftGroup.stop();
  // rightGroup.stop();

}
void testPID(void) {
  for(int i=6; i>0; i--) {
    for(int j=0; j<i; j++) {
      MotionController::run(MotionController::swing(right,-360/i));
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
  //pre_auton();
  //currentPosition = {36,12};
  //inertialSensor.setRotation(90,deg);
  //inertialSensor.setHeading(90,deg);
  //orientation = -90;
  //odom = vex::task(runOdom);
  //odom.stop();
  auto top = sylib::Addrled(22,8,22);
  Top = &top;
  auto block = sylib::Addrled(22,3,40);
  DescoreLEDS = &block;
  auto und1 = sylib::Addrled(22,7,9);
  Under1 = &und1;
  auto und2 = sylib::Addrled(22,6,11);
  Under2 = &und2;
  auto und3 = sylib::Addrled(22,5,19);
  Under3 = &und3;
  auto und4 = sylib::Addrled(22,4,19);
  Under4 = &und4;
  Top->gradient(0xC05DBF,0xFF6AAB);
  Under1->gradient(0xC05DBF,0xFF6AAB);
  Under2->gradient(0xC05DBF,0xFF6AAB);
  Under3->gradient(0xC05DBF,0xFF6AAB);
  Under4->gradient(0xC05DBF,0xFF6AAB);
  Top->cycle(**Top,10);
  Under1->cycle(**Under1,10);
  Under2->cycle(**Under2,10);
  Under3->cycle(**Under3,10);
  Under4->cycle(**Under4,10);
  std::uint32_t clock = sylib::millis();
  vex::timer::event(endgameWarning,75000);
  Intake.setVelocity(100,pct);
  int deadband = 1;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonX.pressed(toggleDescore);
  Controller1.ButtonLeft.pressed(togglePTO);
  // Controller1.ButtonLeft.pressed(cataMatchLoad);
  //vex::task printCoords(printOdom);
  //vex::task leds(handleLEDs);
  // Top -> gradient(0x990000, 0x990005, 0, 0, false, true);
  // Top -> cycle(**Top, 10);
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
    sylib::delay_until(&clock, 10);
    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? -1 : 1) * ((Controller1.Axis3.position()) + (intakeMode ? -1 : 1) * 0.35 * (Controller1.Axis1.position()));
    int rightMotorSpeed = (intakeMode ? -1 : 1) * ((Controller1.Axis3.position()) + (intakeMode ? 1 : -1) * 0.35 * (Controller1.Axis1.position()));

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
void driverSkills(void) {
  hang = false;
  //vex::task intake(releaseIntake);
  inertialSensor.setHeading(90,deg);
  inertialSensor.setRotation(90,deg);
  orientation = -90;
  //odom = task(runOdom);
  currentPosition = {36,12};
  MotionController::chain({
    MotionController::arc(13.5,90,left),
    MotionController::straight(reverse,0.5)
  });
  MotionController::chain({
    MotionController::turnToHeading(180,0.25),
    MotionController::straight(10.5,80,180),
    MotionController::turnToHeading(67)
  });
  MotionController::run(MotionController::straight(-2));
  MotionController::run(MotionController::turnToHeading(67));
  toggleDescore();
  toggleCata();
  usercontrol();
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  typedef void (*callback)();
  callback progs[6] = {oppositeSide, oppositeSideElim, AWPSameSide, sameSide, programmingSkills, testing};
  std::string progNames[6] = {"Safe Opposite", "Cool Opposite", "Safe Same", "Cool Same", "Skills", "test"};
  //Brain.Screen.render(true,false); //set VSync (vertical sync) on, automatic refresh to off
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  //Competition.autonomous(programmingSkills);
  //Competition.autonomous(oppositeSide);
  Competition.autonomous(oppositeSideUnsafe);
  //Competition.autonomous(AWPSameSide);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(programmingSkills);
  //Competition.autonomous(odomSkills);
  //Competition.autonomous(oldProg);
  //Competition.autonomous(testing);
  //Competition.autonomous(testPID);
  //if(Competition.isEnabled()) selectAuton();
  //Competition.drivercontrol(driverSkills);
  Competition.drivercontrol(usercontrol);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    /*
    Brain.Screen.clearScreen(); //clears the back buffer for drawing, default clear color is black
        autonSelection(); //draws our grid to the back buffer
        Brain.Screen.render(); //flips the back buffer to the screen all at once, preventing flickering
        if (Brain.Screen.pressing()) { //if screen is touched...
            while (Brain.Screen.pressing()) { //wait until the user stops touching the screen
                Brain.Screen.clearScreen(); //while waiting, maintain the grid and draw
                autonSelection();                //a touch indicator around the user's finger
                draw_touch();
                Brain.Screen.render();
            }
            wait(1, sec); //wait a second for their hand to get a little further away
            Competition.autonomous(progs[Brain.Screen.yPosition()/80][Brain.Screen.xPosition()/160]);
        } else {
            wait(.1, sec);
        }*/
    // int progNumber = (int)(potentiometer.angle(deg)/40);
    // Competition.autonomous(progs[progNumber]);
    // if(progNumber==4) Competition.drivercontrol(driverSkills);
    // Brain.Screen.clearScreen();
    // Brain.Screen.printAt(240,120,progNames[progNumber].c_str());
    wait(100, msec);
  }
}