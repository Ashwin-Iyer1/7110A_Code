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

motor FrontLeft =             motor(PORT1, ratio6_1, true);
motor MidLeft =               motor(PORT2, ratio6_1, true);
motor BackLeft =              motor(PORT3, ratio6_1, true);
motor FrontRight =            motor(PORT4, ratio6_1, false);
motor MidRight =              motor(PORT5, ratio6_1, false);
motor BackRight =             motor(PORT6, ratio6_1, false);
motor Catapult1 =             motor(PORT10, ratio18_1, true);
motor Catapult2 =             motor(PORT9, ratio18_1, false);
motor Intake =                motor(PORT8, ratio18_1, true);
inertial inertialSensor =  inertial(PORT20);
rotation sideTracking =    rotation(PORT18);
rotation forwardTracking = rotation(PORT19);
pneumatics pto = pneumatics(Brain.ThreeWirePort.A);
pneumatics Descore = pneumatics(Brain.ThreeWirePort.B);
pneumatics Wings = pneumatics(Brain.ThreeWirePort.C);

motor_group leftGroup = motor_group(FrontLeft, BackLeft, MidLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, MidRight);
motor_group Catapult = motor_group(Catapult1, Catapult2);

sylib::Addrled* DescoreLEDS;
sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Top;
float gearRatio = 36.0/48.0;
float wheelDiameter = 3.25;
float wheelRadius = wheelDiameter/2;
float robotRadius = 5.75;
float drivetrainWidth = 11.5;
float sideWheelDist = 6;
float forwardWheelDist = -0.5;
float prevForwardRotation = 0;
float prevSideRotation = 0;
float orientation = 0;
float orientationHeading = fmod(orientation+36000,360);
void delayExecution(std::function<void()> function, float waitTime, vex::timeUnits timeUnit=msec) {
  this_thread::sleep_for((timeUnit==msec) ? waitTime : waitTime/1000);
  function();
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
bool hang = !pto.value();
void stopCata() {
    Catapult.stop();
}

void cataMatchLoad() {
}
int fullCataCycle() {
  if(!hang){
    Catapult.spinFor(directionType::fwd, 360, rotationUnits::deg, 50, velocityUnits::pct, true);
    stopCata();
  }
  return 1;
}
void toggleCata() {
  if(!hang) {
    if(!catatoggle) {
      Catapult.spin(fwd,65,pct);
      catatoggle = true;
    } else {
      stopCata();
      catatoggle = false;
    }
  }
}
void toggleWings() {
  Wings.set(!Wings.value());
}
void toggleDescore() {
  Descore.set(!Descore.value());
  
}
void togglePTO() {
  pto.set(!pto.value());
  hang = !hang;
}
int releaseIntake() {
  Catapult.spinFor(fwd,160, deg);
  wait(0.2,sec);
  Catapult.spinFor(reverse, 860, deg);
  return Catapult.position(deg);
}
int getHangReady() {
  Catapult.spinFor(fwd, 1500, deg);
  return Catapult.position(deg);
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

bool smartTurn(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.4;
  float kd = 0.03;
  float ki = 0.03;
  float dt = 0.02;
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
  float kp = 0.375;
  float kd = 0.025;
  float ki = 0;
  float dt = 0.02;
  float t=0;
  double currAngle = orientation;
  double wantedAngle = currAngle + rot;
  e = wantedAngle-orientation;
  while ((fabs(e) > 2) && t<4) {
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
    if(fabs(clockwiseRotation-360) < fabs(closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  smartTurn(closestPath);
  return true;
}
bool turnToHeadingOdom(float heading) {
  float clockwiseRotation = heading-inertialSensor.heading();
  float closestPath = 0;
  if(fabs(clockwiseRotation) < fabs(clockwiseRotation+360)) {
    closestPath = clockwiseRotation;
    if(fabs(clockwiseRotation-360) < (closestPath)) closestPath-=360;
  } else {
    closestPath = clockwiseRotation+360;
  }
  smartTurnOdom(closestPath);
  return true;
}
void odomUpdate() {
  float orientationDiff = -inertialSensor.rotation()-orientation;
  float radOrientationDiff = orientationDiff*M_PI/180;
  orientation = -inertialSensor.rotation();
  orientationHeading = fmod(orientation+36000,360);
  float radOrientation = orientation/180*M_PI;
  float forwardDiff = ((forwardTracking.position(deg)-prevForwardRotation)/360*(wheelDiameter * M_PI));
  float sideDiff = ((sideTracking.position(deg)-prevSideRotation)/360*(wheelDiameter * M_PI));
  prevForwardRotation = forwardTracking.position(deg);
  prevSideRotation = sideTracking.position(deg);
  if(orientationDiff < 0.1) {
    Vector2d positionChange(sideDiff,forwardDiff,orientation);
    currentPosition.x += positionChange.x;
    currentPosition.y += positionChange.y;
  } else {
    Vector2d positionChange(2*sin(radOrientationDiff)*(sideDiff/radOrientationDiff + sideWheelDist),2*sin(radOrientationDiff)*(forwardDiff/radOrientationDiff + forwardWheelDist),orientation + orientationDiff/2);
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
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("o: %.1f x: %.1f y: %.1f", orientationHeading, x, y);
    wait(15,msec);
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
  for(double t=0; t<fabs(dist/20); t+=0.025) {
    if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  leftGroup.stop(brake);
  rightGroup.stop(brake);
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
  
  leftGroup.setVelocity(leftspeed * 50,pct);
  rightGroup.setVelocity(rightspeed * 50,pct);
  leftGroup.spinFor(distToRot(leftArc), deg, false);
  rightGroup.spinFor(distToRot(rightArc), deg, false);
  wait(0.5,sec);
  for(double t=0; t<fabs(fmax(fabs(leftArc),fabs(rightArc)))/20; t+=0.025) {
    if(leftGroup.isDone() && rightGroup.isDone()) break;
    wait(0.025,sec);
  }
  //wait(1,sec);
  leftGroup.stop();
  rightGroup.stop();
}
//Currently inaccurate, will require tuning of driveRotationConstant
void simpleTurn(float deg) {
  /*float dist = driveRotationConstant * gearRatio * deg * robotRadius / wheelRadius;
  leftGroup.spinFor(fwd, dist, vex::deg, false);
  rightGroup.spinFor(reverse, dist, vex::deg, false);*/
  arc(0,deg,right);
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

  public:
    BezierCurve(std::initializer_list<Vector2d> args) {
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
          result.x += int32_t(-1*(points.size()-1-i)*bezierCoefficients.at(i)*pow(1-t,points.size()-2-i)*pow(t,i)*points.at(i).x + bezierCoefficients.at(i)*pow(1-t,points.size()-1-i)*i*pow(t,i-1)*points.at(i).x);
          result.y += int32_t(-1*(points.size()-1-i)*bezierCoefficients.at(i)*pow(1-t,points.size()-2-i)*pow(t,i)*points.at(i).y + bezierCoefficients.at(i)*pow(1-t,points.size()-1-i)*i*pow(t,i-1)*points.at(i).y);
        }
        return result; 
    }
};
class BezierSpline {
public:
  std::map<double,Vector2d> lut;
  BezierSpline(const Vector2d& p0, const Vector2d& p1, const Vector2d& p2)
    : P0(p0), P1(p1), P2(p2) {}

  Vector2d calculatePoint(double t) const {
    Vector2d result;
    if(t<0) {
      result.x = P0.x;
      result.y = P0.y;
      return result;
    }
    if(t>1) {
      result.x = P2.x;
      result.y = P2.y;
      return result;
    }
    result.x = (1 - t) * (1 - t) * P0.x + 2 * (1 - t) * t * P1.x + t * t * P2.x;
    result.y = (1 - t) * (1 - t) * P0.y + 2 * (1 - t) * t * P1.y + t * t * P2.y;
    return result; 
  }

  Vector2d calculateTangent(double t) const {
    Vector2d tangent;
    // tangent.x = 2 * (1 - t) * (P1.x - P0.x) + 2 * t * (P2.x - P1.x);
    // tangent.y = 2 * (1 - t) * (P1.y - P0.y) + 2 * t * (P2.y - P1.y);
    tangent.x = -2 * (1 - t) * P0.x + 2 * (1 - 2 * t) * P1.x + 2 * t * P2.x;
    tangent.y = -2 * (1 - t) * P0.y + 2 * (1 - 2 * t) * P1.y + 2 * t * P2.y;
    return tangent;
  }
  double approxLength(int intervals) {
    double length = 0;
    for(float t=0; t<intervals; t++) {
      Vector2d currentPos = calculatePoint(1.0/intervals*t);
      Vector2d nextPos = calculatePoint((1.0/intervals)*(t+1));
      length += sqrt(pow(nextPos.x - currentPos.x,2) + pow(nextPos.y - currentPos.y,2));
    }
    return length;
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

private:
    Vector2d P0, P1, P2;
};

class RobotController {
  public:
    RobotController(Robot& robot) : robot(robot) {}

    void moveRobot(const Vector2d& middle_position, const Vector2d& target_position, vex::directionType direction = fwd) {
      double max_speed = 50.0;
      double lookAhead = 24.0;

      Vector2d current_position = getCurrentPosition();
      BezierSpline spline(current_position, middle_position, target_position);
      spline.bake(10);
      Vector2d ogtan = spline.calculateTangent(0);
      turnToHeading(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + 270);
      Controller1.Screen.print(-atan2(ogtan.y, ogtan.x) / M_PI * 180 + 90);
      //leftGroup.spin(fwd);
      // leftGroup.setVelocity(20, pct);
      //rightGroup.spin(fwd);
      // rightGroup.setVelocity(20, pct);
      //splinePos is the percent of the track the robot has completed
      float kp = 0.6;
      float ki = 0.000;
      float kd = 0.01;
      float e = 0;
      float d = 0;
      float i = 0;
      float eRec = 0;
      // Controller1.Screen.print("DOES THIS WORK");
      // Controller1.Screen.print("STILL WORKING");
      //wait(0.5,sec);
      //wait(5,sec);
      //wait(5,sec);
      double timeStep = 0.1;
        for (double distance=currentPosition.distance(target_position); distance>=lookAhead; distance=currentPosition.distance(target_position)) {
          double desiredPoint = spline.intersectCircle(lookAhead,currentPosition.x,currentPosition.y);
          Vector2d tangent = spline.calculateTangent(desiredPoint);
          // Simple control logic: adjust left and right motor speeds based on tangent
          double angle = std::atan2(tangent.y, tangent.x); //angle of tangent vector in radians
          // Brain.Screen.print(angle);
          // Brain.Screen.newLine();
          // Brain.Screen.print(orientationHeading);
          angle = angle / M_PI * 180 + 90; //convert to degrees
          double angle_difference = -fmod(angle - orientationHeading, 360);
          max_speed = fabs(50.0 - fabs(angle_difference) / 5);
          if(fabs(angle_difference) > 180) angle_difference = (360-angle_difference*((angle_difference>0)? 1 : -1));
          e = angle_difference;
          d = (e-eRec)/timeStep;
          i += (e*timeStep);
          eRec = e;
          //distance = sqrt((desired_position.x-x)*(desired_position.x-x) + (desired_position.y-y)*(desired_position.y-y));
          //float speed = 2 * sinf((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth/2) * (rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60/((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth) + drivetrainWidth/2);
          
          //1 pct speed diff in 0.1 seconds = 0.3 degrees change
          double speed_difference = ki*i + kd*d + kp*e;

          double left_speed = (direction == fwd) ? max_speed - speed_difference : max_speed+speed_difference;
          double right_speed = (direction == fwd) ? max_speed + speed_difference : max_speed-speed_difference;

          // Send motor commands to the robot
          robot.left_motor_speed = left_speed;
          robot.right_motor_speed = right_speed;
          // Brain.Screen.print("left: ");
          // Brain.Screen.print(left_speed);
          // Brain.Screen.print("right: ");
          // Brain.Screen.print(right_speed);
          // Brain.Screen.newLine();

          Brain.Screen.newLine();
          //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
          Brain.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f, %.1f", angle_difference, angle, orientation, inertialSensor.heading(), currentPosition.x, currentPosition.y);
          // Move forward in time

          // Simulate robot movement (you may replace this with your actual motion control logic)
          leftGroup.setVelocity(robot.left_motor_speed, pct);
          leftGroup.spin(direction);
          rightGroup.setVelocity(robot.right_motor_speed, pct);
          rightGroup.spin(direction);
          wait(timeStep, sec);
        }
        //straight(lookAhead);

        // Stop the robot when the path is complete
        robot.left_motor_speed = 0.0;
        robot.right_motor_speed = 0.0;
        leftGroup.stop();
        rightGroup.stop();
    }

  private: 
    Robot& robot;

    Vector2d getCurrentPosition() const {
        // Replace this with your actual logic to get the current robot position
        // For simplicity, assume the robot starts at the origin
        return currentPosition;
    }

};

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
  rightGroup.setVelocity(50, percent);
  sideTracking.resetPosition();
  forwardTracking.resetPosition();
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
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  odom = vex::task(runOdom);
  orientation = -45;
  inertialSensor.setHeading(45,deg); 
  inertialSensor.setRotation(45, deg);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  releaseIntake();
  toggleDescore();
  //straight(-5);
  arc(18, -45, right);
  toggleDescore();
  slam(reverse);
  straight(5);
  turnToHeading(90);
  straight(40);
  turnToHeading(40);
  straight(20);
  /*
  Intake.spin(fwd);   
  straight(3, 35);
  wait(0.5,sec);
  //pp stuff
  straight(-23, 75);
  Intake.stop();
  toggleDescore();
  arc(16,-75,right);
  toggleDescore();
  simpleTurn(15);
  slam(reverse);
  arc(20,150,right);
  straight(18);
  turnToHeading(270);
  Intake.spin(reverse);
  straight(12);*/
  /*
  Robot myRobot{ 0.0, 0.0 };
  RobotController controller(myRobot);
  toggleDescore();
  controller.moveRobot({ 12.5, 120 }, { 9.5, 118 }, reverse);
  toggleDescore();
  slam(reverse);*/
  /*
  straight(-26,75);
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
  wait(0.3, sec);
  slam(fwd);
  turnToHeading(35);
  toggleDescore();
  straight(-38);
  smartTurn(-20);
  */
}
void oppositeSideUnsafe(void) {
  //odom = vex::task(runOdom);
  //orientation = -90;
  inertialSensor.setHeading(90,deg); 
  inertialSensor.setRotation(90, deg);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  vex::task run(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);   
  straight(19, 35);
  wait(1,sec);
  //pp stuff
  straight(-40, 75);
  Intake.stop();
  toggleDescore();
  arc(15,-90,right);
  toggleDescore();
  simpleTurn(15);
  slam(reverse);
  arc(15,150,right);
  straight(32);
  turnToHeading(270);
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(22);
  Intake.stop();
  straight(-15);
  turnToHeading(70);
  Intake.spin(fwd);
  straight(24);
  Intake.stop();
  simpleTurn(70);
  straight(24);
  turnToHeading(270);
  toggleWings();
  slam(fwd);
  straight(-15);
  toggleWings();
}
void oppositeSideElim(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
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
  straight(-18);
  releaseIntake();
  //Intake.spin(fwd);
  //vex::task run(LEDRainbow);
  //vex::task run(bringCataDown);
  //Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  /*
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
  arc(13, 180, right);
  straight(27);
  turnToHeading(0);
  toggleWings();
  arc(16.5, -90, right);
  toggleWings();
  turnToHeading(315);
  straight(-8);
  turnToHeading(270);
  Descore.set(true);
  straight(-22);
  straight(-12,50);
  smartTurn(20);*/
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
  inertialSensor.setHeading(120,deg);
  /*
  arc(17,90,right);
  toggleDescore();
  Catapult.spinFor(fwd, 900, deg, false);
  arc(17,-90,left); 
  turnToHeading(90);
  straight(-32);
  */

  
  straight(15);
  smartTurn(15);
  toggleDescore();
  wait(0.5, sec);
  straight(-8);
  smartTurn(-20);
  Catapult.spinFor(fwd, 1100, deg, false);
  wait(0.5,sec);
  toggleDescore();
  straight(-10);
  wait(1, sec);
  straight(2);
  turnToHeading(90);
  straight(-18);
  straight(-10,25);
  
}
void programmingSkills(void) {
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
}
void testing(void) {
  odom = vex::task(runOdom);
  orientation = -270;
  inertialSensor.setHeading(270, deg);
  inertialSensor.setRotation(270, deg);
  currentPosition = {0,0};
  
  // followPath({
  //   {48,70},
  //   {36,104},
  //   {40,130},
  //   {108,130}
  // });
  Robot myRobot{ 0.0, 0.0 };
    RobotController controller(myRobot);

    // Move the robot along a Bezier spline to the target position
    controller.moveRobot({ 12, -12 }, { 36, 12 }, reverse);
    wait(1, sec);
    controller.moveRobot({-24, 48}, {0, 48  });
    //left 0, right 10 for 1 second = 21 degrees
  // leftGroup.spin(fwd);
  // rightGroup.spin(fwd);
  // leftGroup.setVelocity(90, pct);
  // rightGroup.setVelocity(100, pct);
  // wait(1, sec);
  // leftGroup.stop();
  // rightGroup.stop();

}
void testHang(void) {
  releaseIntake();
  getHangReady();
  straight(24);
  Catapult.spinFor(reverse, 1500, deg);
}
void usercontrol(void) {
  std::uint32_t clock = sylib::millis();
  pre_auton();
  Intake.setVelocity(100,pct);
  int deadband = 5;
  bool intakeMode = true;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonX.pressed(toggleDescore);
  Controller1.ButtonLeft.pressed(togglePTO);
  // Controller1.ButtonLeft.pressed(cataMatchLoad);
  odom = vex::task(runOdom);
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
    int leftMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * 0.5 * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * 0.5 * Controller1.Axis1.position());

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
      } else {
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
  // Competition.autonomous(programmingSkills);
  // Competition.autonomous(oppositeSide);
  Competition.autonomous(oppositeSideUnsafe);
  //Competition.autonomous(AWPSameSide);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(programmingSkills);
  // Competition.autonomous(testing);
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