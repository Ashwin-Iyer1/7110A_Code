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
motor      BackRight =            motor(PORT6, ratio6_1, false);
motor      Catapult1 =            motor(PORT10, ratio18_1, true);
motor      Catapult2 =            motor(PORT9, ratio18_1, false);
motor      Intake =               motor(PORT8, ratio18_1, true);

inertial   inertialSensor =    inertial(PORT20);
rotation   sideTracking =      rotation(PORT17);
rotation   forwardTracking =   rotation(PORT19);
rotation   hangSensor =        rotation(PORT14);

pneumatics pto =             pneumatics(Brain.ThreeWirePort.A);
pneumatics Descore =         pneumatics(Brain.ThreeWirePort.B);
pneumatics Wings =           pneumatics(Brain.ThreeWirePort.C);

motor_group leftGroup =  motor_group(FrontLeft, BackLeft, MidLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, MidRight);
motor_group Catapult =   motor_group(Catapult1, Catapult2);

sylib::Addrled* DescoreLEDS;
sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Top;

float gearRatio = 36.0/48.0;
float wheelDiameter = 3.25;
float wheelRadius = wheelDiameter/2;
float robotRadius = 5.75;
float drivetrainWidth = 11.5;
float sideWheelDist = 7;
float forwardWheelDist = -0.5;
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
};
Vector2d currentPosition = {0,0};
double driveRotationConstant = 0.8721445746*1.054572148;
/*
class OdomGyro: public rotation
{
  private:
    float prevLeft;
    float prevRight;
    float prevInertial;
    rotation leftSide;
    rotation rightSide;
    float orientation;
    float heading;
    int updateRotation() {
      float leftDiff = leftSide.position(deg) - prevLeft;
      float rightDiff = rightSide.position(deg) - prevRight;
      float odomDiff = ((leftDiff-rightDiff)/360*wheelDiameter*M_PI/drivetrainWidth)/M_PI*180;
      float inertialDiff = 
    }
  public:
    OdomGyro(inertial inert, rotation leftSide, rotation rightSide) {

    }
};
*/

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
  toggleCata(55);
}

void toggleWings() {
  Wings.set(!Wings.value());
}

void toggleDescore() {
  Descore.set(!Descore.value());
}
int movePTOGear() {
  if(!hang) {
    waitUntil(hangSensor.velocity(dps)<0);
  } else {
    waitUntil(hangSensor.velocity(dps)>0);
  }
  Catapult.stop();
  return 1;
}
void togglePTO() {
  pto.set(!pto.value());
  Catapult.spin(fwd);
  hang = !hang;
  task move(movePTOGear);
}

int releaseIntake() {
  //Intake.spinFor(-1,rev,false);
  Catapult.spinFor(fwd, 300, deg);
  Catapult.spinFor(reverse, 500, deg);
  return 1;
}

int hangSetup() {
  Catapult.spin(fwd);
  waitUntil(hangSensor.position(deg)>560);
  Catapult.stop();
  return 1;
}
int sideHang() {
  Catapult.spin(reverse);
  waitUntil(hangSensor.position(deg)<0);
  Catapult.stop();
  return 1;
}

int handleLEDs() {
  bool prevDescore = Descore.value();
  bool idle = false;
  while(true) {
    if(Descore.value() && !prevDescore) {
        DescoreLEDS->set_all(0x000000);
        DescoreLEDS -> gradient(0x000000, 0xFF0000, 8, 0, false, false);
        DescoreLEDS -> cycle(**DescoreLEDS, 25);
        Top->set_all(0x000000);
        Top -> gradient(0x000000, 0xFF0000, 8, 0, false, false);
        Top -> cycle(**DescoreLEDS, 25);
        Under1->set_all(0xFF0000);
        Under2->set_all(0xFF0000);
        idle = false;
    } else if(catatoggle){
      /*
      int cataprogress = (int)rotationSensor.position(deg)*2.5;
      DescoreLEDS->set_all(cataprogress*65793);
      Top->set_all(cataprogress*65793);
      Under1->set_all(cataprogress*65793);
      Under2->set_all(cataprogress*65793);
      */
      idle = false;
    } else if(!idle && !Descore.value()){
      DescoreLEDS->set_all(0x000000);
      Top->set_all(0x000000);
      Under1->set_all(0x000000);
      Under2->set_all(0x000000);
      DescoreLEDS -> gradient(0x600000, 0x600002, 0, 0, false, true);
      DescoreLEDS -> cycle(**DescoreLEDS, 10);

      Top -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Top -> cycle(**Top, 10);

      Under1 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under1 -> cycle(**Under1, 10);

      Under2 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under2 -> cycle(**Under2, 10);
      idle = true;
    }
    prevDescore = Descore.value();
    wait(50,msec);
  }
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
  float kp = 0.305;
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
  orientation = -inertialSensor.rotation();
  orientationHeading = fmod(orientation+36000,360);
  float radOrientation = orientation/180*M_PI;
  float forwardDiff = ((forwardTracking.position(deg)-prevForwardRotation)/360*(wheelDiameter * M_PI));
  float sideDiff = -((sideTracking.position(deg)-prevSideRotation)/360*(wheelDiameter * M_PI));
  prevForwardRotation = forwardTracking.position(deg);
  prevSideRotation = sideTracking.position(deg);
  if(orientationDiff < 0.01) {
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
  hangSensor.setPosition(hangSensor.angle(),deg);
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
    DescoreLEDS -> cycle(Top -> buffer, 10);

    Top -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Top -> cycle(Top -> buffer, 10);

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
  arc(18, -30, right);
  slam(reverse);
  straight(12);
  //smartTurn(-25);
  //turnToHeading(0);
  turnToHeading(180);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(17);
  straight(-17);
  turnToHeading(115);
  straight(38);
  turnToHeading(55);
  straight(24,35);
  //turnToHeading(110);
  //Intake.spin(fwd);
  //straight(44);
}
void oppositeSideUnsafe(void) {
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
  turnToHeading(88.5);
  wait(0.25,sec);
  straight(-29, 60);
  Intake.stop();
  //turnToHeading(55);
  toggleDescore();
  arc(13,-90,right);
  toggleDescore();
  //straight(-22);
  turnToHeading(30);
  arc(18, -30, right);
  slam(reverse);
  straight(10);
  //smartTurn(-25);
  //turnToHeading(0);
  turnToHeading(180);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(13);
  straight(-13);
  turnToHeading(110,0.5);
  Intake.spin(fwd);
  smartStraight(44);
  turnToHeading(225);
  Intake.spin(reverse);
  wait(0.5,sec);
  turnToHeading(170);
  Intake.spin(fwd);
  straight(24);
  turnToHeading(90);
  toggleDescore();
  slam(reverse);
  straight(12);
  turnToHeading(270);
  Intake.spin(reverse);
  straight(12);
  /*
  //smartTurn(-25);
  //simpleTurnToHeading(0);
  turnToHeading(180, 1);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(15);
  straight(-17);
  turnToHeading(110, 1);
  Intake.spin(fwd);
  smartStraight(44);
  turnToHeading(180, 1);
  Intake.stop();
  smartStraight(25);
  turnToHeading(270, 1);
  Intake.spin(reverse);
  wait(0.25,sec);
  toggleWings();
  straight(32);
  straight(-19);
  toggleWings();
  turnToHeading(115);
  Intake.spin(fwd);
  straight(8);
  wait(0.5,sec);
  Intake.stop();
  turnToHeading(270);
  Intake.spin(reverse);
  slam(fwd);
  straight(-15);
  */
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
  turnToHeading(193.5);
  Intake.spin(fwd);
  smartStraight(50);
  wait(0.5,sec);
  Intake.stop();
  smartStraight(-50);
  turnToHeading(270);
  Intake.spin(reverse);
  straight(-4);
  straight(28);
  straight(-4);
}
void AWPSameSide(void) {
//odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(90);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task run(releaseIntake);
  //turnToHeading(55);
  arc(11,90,right);
  //straight(-22);
  turnToHeading(180);
  straight(4);
  toggleDescore();
  Intake.spinFor(-3,rev,false);
  //runFunction1.function = toggleDescore;
  //runFunction1.waitTime = 1250;
  //vex::thread delay(delayExecution, &runFunction1);
  arc(11, -60, right);
  smartTurn(-40);
  toggleDescore();
  turnToHeading(135);
  Catapult.spinFor(1800,deg,false); 
  straight(-10);
  turnToHeading(90);
  straight(-28,70);
  toggleDescore();
  straight(-3,20);
  smartTurn(20);
}

void programmingSkills(void) {
  //hang = true;
  //vex::task intake(releaseIntake);
  inertialSensor.setHeading(90,deg);
  inertialSensor.setRotation(90,deg);
  orientation = -90;
  currentPosition = {36,12};
  odom = task(runOdom);
  wait(0.1,sec);
  followApproxBezier({{22,22}, {18, 42}}, 180, reverse,8);
  followApproxBezier({{12,24}},fwd,12);
  //togglePTO();
  
  //arc(30,-20,left);
  turnToHeading(70);
  straight(-1);
  toggleDescore();
  //toggleCata();
  //float currentRotation = inertialSensor.rotation(deg);
  //float currentHeading = inertialSensor.heading(deg);
  //odom.suspend();
  wait(2,sec);
  //toggleCata();
  toggleDescore();
  //togglePTO();
  //setInertial(currentRotation);
  //odom.resume();
  //followBezier({{32.234,10.058},{87.837,12.798},{120.695,13.478},{132.679,29.167},{132.630,40.139}},fwd,6);
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
  //arc(17,90,left);
  /*
  turnToHeading(180);
  slam(reverse);
  togglePTO();
  straight(16);
  turnToHeading(70);
  straight(-1);
  toggleDescore();
  toggleCata();
  float currentRotation = inertialSensor.rotation(deg);
  float currentHeading = inertialSensor.heading(deg);
  //odom.suspend();
  wait(2,sec);
  toggleCata();
  toggleDescore();
  togglePTO();
  setInertial(currentRotation);
  //odom.resume();
  turnToHeading(120);
  smartStraight(24.5);
  turnToHeading(90);
  smartStraight(72);
  turnToHeading(45);
  smartStraight(28);
  turnToHeading(25);
  straight(12);
  straight(-8);
  turnToHeading(0);
  straight(10);
  straight(-12);
  turnToHeading(295);
  smartStraight(52);
  turnToHeading(260);
  toggleDescore();
  arc(150,10,left);
  slam(reverse);
  straight(6);
  toggleDescore();
  arc(35,50,right);
  turnToHeading(270);
  toggleDescore();
  slam(reverse);
  straight(6);
  toggleDescore();
  arc(70,25,right);
  turnToHeading(0);
  smartStraight(25);
  turnToHeading(300);
  toggleDescore();
  arc(50,-30,right);
  slam(reverse);
  straight(4);
  toggleDescore();
  task getUp(hangSetup);
  turnToHeading(0);
  smartStraight(54);
  turnToHeading(270);
  smartStraight(38);
  task down(sideHang);*/
  /*
  followBezier({ {23, -4}, { 128, 17 }, {140,32}});
  turnToHeading(0);
  straight(12);
  straight(-8);
  straight(8);
  straight(-8);
  followBezier({{84,30},{84,60}});
  straight(10);
  turnToHeading(260);
  toggleDescore();
  slam(reverse);
  straight(5);
  toggleDescore();
  followBezier({{90,60},{84,96}});
  straight(10);
  turnToHeading(270);
  toggleDescore();
  slam(reverse);
  straight(5);
  toggleDescore();
  followBezier({{86,78},{84,126}});
  toggleDescore();
  //followBezier({{96,96},{120,84}},reverse);
  turnToHeading(315);
  arc(70,-45,right);
  straight(5);
  toggleDescore();
  // vturnToHeading(0);
  Catapult.spinFor(fwd, 1500, deg, false);
  followBezier({{120,132}});
  turnToHeading(270);
  straight(36);
  Catapult.spinFor(reverse, 1500, deg, false);
  */
  /*
  inertialSensor.setHeading(90,deg);
  
  arc(18,90,left);
  slam(reverse);
  straight(10.5);
  turnToHeading(73.5);
  straight(-3);
  turnToHeading(69);
  float realOrientation = inertialSensor.heading(deg);
  toggleWings();
  toggleCata();
  wait(33,sec);
  toggleCata();
  toggleWings();
  //bringCataDown(250);
  //go to other side
  inertialSensor.setHeading(realOrientation,deg);
  straight(4);
  turnToHeading(317);
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
  arc(9.5,160,right);
  turnToHeading(130);
  //backup
  straight(-30);
  turnToHeading(230);
  toggleWings();
  arc(40,60,left);
  slam(reverse);
  straight(4);
  toggleWings();
  arc(70,20,right);
  smartTurn(-40);
  toggleWings();
  arc(70,20,left);
  slam(reverse);
  straight(8);
  toggleWings();
  arc(50,40,right);
  turnToHeading(0);
  straight(36);
  turnToHeading(315);
  toggleWings();
  arc(70,-45,right);
  straight(10);
  toggleWings();
  straight(50);
  */
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
 /*
 inertialSensor.setHeading(270,deg);
  hang = false;
  arc(18,-90,right);
  slam(reverse);
  straight(10.5);
  turnToHeading(286.5);
  straight(-3);
  turnToHeading(291);
  toggleWings();
  toggleCata();
  float realOrientation = inertialSensor.heading(deg);
  wait(2,sec);
  toggleCata();
  toggleWings();
  //bringCataDown(250);
  //go to other side
  inertialSensor.setHeading(realOrientation,deg);
  straight(4);
  turnToHeading(45);
  arc(120,17,left);
  turnToHeading(90);
  
  straight(-54);
  //go to other side
  //toggleWings();
  arc(22.5,90,left);
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
  arc(11.5,-160,left);
  turnToHeading(20);
  //backup
  straight(22);
  toggleWings();
  arc(10,-102.5,left);
  slam(fwd);
  straight(-14);
  slam(fwd);
  straight(-22);
  toggleWings();
  arc(50,-40,right);
  turnToHeading(180);
  straight(-40);
  turnToHeading(225);
  toggleWings();
  arc(70,45,right);
  straight(-10);
  toggleWings();
  straight(-50);
  */
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
      smartTurn(360/i);
      //straight(12);
    }
  }
  //straight(70);
}
void testHang(void) {
  releaseIntake();
  hangSetup();
  straight(24);
  Catapult.spinFor(reverse, 1500, deg);
}
void usercontrol(void) {
  //pre_auton();
  //currentPosition = {36,12};
  //inertialSensor.setRotation(90,deg);
  //inertialSensor.setHeading(90,deg);
  //orientation = -90;
  //odom = task(runOdom);
  //odom.stop();
  std::uint32_t clock = sylib::millis();
  Intake.setVelocity(100,pct);
  int deadband = 5;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonX.pressed(toggleDescore);
  Controller1.ButtonLeft.pressed(togglePTO);
  // Controller1.ButtonLeft.pressed(cataMatchLoad);
  //vex::task printCoords(printOdom);
  /*
  auto block = sylib::Addrled(22,8,40);
  DescoreLEDS = &block;
  auto und1 = sylib::Addrled(22,7,14);
  Under1 = &und1;
  auto und2 = sylib::Addrled(22,6,13);
  Under2 = &und2;
  auto top = sylib::Addrled(22,5,23);
  Top = &top;
  vex::task leds(handleLEDs);*/
  //currentPosition = {0,0};
  while (true) {
    //tank drive
    sylib::delay_until(&clock, 10);
    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * 0.35 * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * 0.35 * Controller1.Axis1.position());

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
    if(hang) {
      // OUTTAKE
      if(Controller1.ButtonR1.pressing()) {
          Catapult.spin(fwd);
      } else if (Controller1.ButtonR2.pressing()) {
          Catapult.spin(directionType::rev);
      } else if(!catatoggle){
          Catapult.stop();
      }
    } else {
      // Single Catapult Cycle
      if(Controller1.ButtonA.pressing()) {
        Catapult.spin(fwd);
      }
    }
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
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.clearLine();
    Controller1.Screen.print("%.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, -inertialSensor.rotation());
    wait(0.025,sec);
  }
}
void driverSkills(void) {
  inertialSensor.setHeading(90,deg);
  arc(18,90,left);
  slam(reverse);
  straight(11);
  turnToHeading(73.5);
  straight(-3);
  turnToHeading(69);
  float realOrientation = inertialSensor.heading(deg);
  toggleWings();
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
  Competition.autonomous(programmingSkills);
  //Competition.autonomous(oppositeSide);
  //Competition.autonomous(oppositeSideUnsafe);
  //Competition.autonomous(AWPSameSide);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(programmingSkills);
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