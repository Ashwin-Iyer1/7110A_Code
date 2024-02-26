/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       s617995                                                   */
/*    Created:      10/3/2023, 8:15:04 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot_config.h"
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
#include "7110A-Template/chassis.h"

using namespace vex;
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


void odomUpdate() {
  float orientationDiff = -Chassis::inertialSensor.rotation()-orientation;
  float radOrientationDiff = orientationDiff*M_PI/180;
  float radOrientation = orientation/180*M_PI;
  orientation = -Chassis::inertialSensor.rotation();
  orientationHeading = fmod(orientation+36000,360);
  float forwardDiff = ((forwardTracking.position(deg)-prevForwardRotation)/360*(Chassis::wheelDiameter * M_PI));
  float sideDiff = -((sideTracking.position(deg)-prevSideRotation)/360*(Chassis::wheelDiameter * M_PI));
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
  Controller1.Screen.print("%.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, -Chassis::inertialSensor.rotation());
    wait(100,msec);
}

int runOdom() {
  while(true) {
    odomUpdate();
    wait(25,msec);
  }
}


int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}
using namespace Chassis;
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
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2;
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
      
      Chassis::leftGroup.spin(direction,left_speed,pct);
      Chassis::rightGroup.spin(direction,right_speed,pct);
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
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2;
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
      
      Chassis::leftGroup.spin(direction,left_speed,pct);
      Chassis::rightGroup.spin(direction,right_speed,pct);
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
      double left_speed = (direction == fwd) ? avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2;
      double right_speed = (direction == fwd) ? avg_speed*(2-curvature*(Chassis::drivetrainWidth))/2 : avg_speed*(2+curvature*(Chassis::drivetrainWidth))/2;
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
      
      Chassis::leftGroup.spin(direction,left_speed,pct);
      Chassis::rightGroup.spin(direction,right_speed,pct);
      t+=timeStep;
      wait(timeStep, sec);
    }
    //straight(lookAhead);

    // Stop the robot when the path is complete
    Brain.Screen.newLine();
      //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
      Brain.Screen.print("%.1f, %.1f", currentPosition.x, currentPosition.y);
      
    Chassis::leftGroup.stop();
    Chassis::rightGroup.stop();
}

void pre_auton(void) {
  Chassis::init(vex::motor_group(FrontLeft,MidLeft,BackLeft),vex::motor_group(FrontRight,MidRight,BackRight),PORT20,3.25,36/48,11.5);
  sylib::initialize();
  Catapult.setStopping(brakeType::brake);
  Catapult.setVelocity(100,pct); 
  Intake.setVelocity(100,pct);
  Chassis::leftGroup.setVelocity(50, percent);
  Chassis::leftGroup.setStopping(coast);
  Chassis::rightGroup.setStopping(coast);
  Chassis::rightGroup.setVelocity(50, percent);
  sideTracking.resetPosition();
  forwardTracking.resetPosition();
  //hangSensor.setPosition(hangSensor.angle(),deg);
}

void oppositeSide(void) {
  //odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(90);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task intake(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  run(straight(3, 35));
  //pp stuff
  wait(0.25,sec);
  run(turnToHeading(88.5));
  wait(0.5,sec);
  run(straight(-29, 60));
  Intake.stop();
  //turnToHeading(55);
  toggleDescore();
  run(arc(13,-90,right));
  toggleDescore();
  //straight(-22);
  run(turnToHeading(30));
  chain({
    arc(24,-30,right),
    straight(reverse,0.5)
  });
  straight(12);
  //smartTurn(-25);
  //turnToHeading(0);
  run(turn(180));
  Intake.spin(reverse);
  wait(0.5,sec);
  straight(17);
  straight(-14);
  Intake.spin(fwd);
  chain({
    turnToHeading(113),
    straight(38),
    swingToHeading(right,270,fwd,1.5)
  });
  Intake.spin(reverse);
  chain({
    straight(fwd,1),
    straight(-10),
    turnToHeading(31),
    straight(38)
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
  vex::task intake(releaseIntake);
  //straight(-5);
  wait(0.5, sec);
  Intake.spin(fwd);  
  run(straight(3, 35));
  //pp stuff
  wait(0.25,sec);
  run(turnToHeading(88.5));
  wait(0.5,sec);
  run(straight(-29, 60));
  Intake.stop();
  //turnToHeading(55);
  toggleDescore();
  run(arc(13,-90,right));
  toggleDescore();
  //straight(-22);
  run(turnToHeading(30));
  chain({
    arc(24,-30,right),
    straight(reverse,0.5)
  });
  run(straight(12));
  //smartTurn(-25);
  //turnToHeading(0);
  run(turn(180));
  Intake.spin(reverse);
  wait(0.5,sec);
  run(straight(17));
  run(straight(-14));
  Intake.spin(fwd);
  chain({
    turnToHeading(113),
    straight(38),
    swingToHeading(right,270,fwd,1.5)
  });
  Intake.spin(reverse);
  chain({
    straight(fwd,1.25),
    straight(6)
  });
}
void sameSide(void) {
 //odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(180);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  // score alliance triball to near net     
  vex::task intake(releaseIntake);
  Intake.spin(fwd);
  chain({
    straight(42,100),
    arc(12,20,right),
    straight(-38),
    turnToHeading(295)
  });
  Intake.spin(reverse);
  wait(1,sec);
  Intake.stop();
  chain({
    turnToHeading(115),
    straight(18.5),
    swingToHeading(right,135),
    toggleDescore,
    straight(-15),
    turnToHeading(90,1),
    toggleDescore,
    swingToHeading(left,120,fwd,1),
    straight(-16),
    turnToHeading(90,1),
    straight(-8)
  });
  brakeAll();

}
void AWPSameSide(void) {
//odom = vex::task(runOdom);
  //orientation = -90;
  setInertial(135);
  //currentPosition = {58,132};
  Intake.setVelocity(100,pct);
  run(straight(13));
  toggleDescore();
  run(swingToHeading(right,90));
  toggleDescore();
  run(turnToHeading(120));
  Catapult.spinFor(1700,deg,false);
  run(straight(-18));
  run(turnToHeading(90));
  run(straight(-28));
  run(straight(-6,20));
}

void odomSkills(void) {
  Chassis::inertialSensor.setHeading(90,deg);
  Chassis::inertialSensor.setRotation(90,deg);
  orientation = -90;
  odom = task(runOdom);
  currentPosition = {36,12};
  wait(0.1,sec);
  followApproxBezier({{18,18}, {12, 38}}, 180, reverse,18);
  slam(reverse);
  followApproxBezier({{24,24}},fwd,8);
  Chassis::leftGroup.stop();
  Chassis::rightGroup.stop();
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

void programmingSkills(void) {
  hang = false;
  Catapult.setStopping(coast);
  setInertial(90);
  orientation = -90;
  currentPosition = {36,12};
  chain({
    arc(13,90,left),
    straight(reverse,0.5)
  });
  chain({
    turnToHeading(180,0.25),
    straight(10.5,80,180),
    turnToHeading(69)
  });
  run(straight(-3));
  run(turnToHeading(69,0.5,0.5));
  toggleDescore();
  toggleCata();
  float currentRotation = Chassis::inertialSensor.rotation(deg);
  float currentHeading = Chassis::inertialSensor.heading(deg);
  vex::wait(24.5,sec);
  toggleCata();
  toggleDescore();
  setInertial(currentHeading);
  chain({
    straight(2),
    turnToHeading(310),
    straight(-23.5),
    swingToHeading(right,270),
    straight(-60),
    arc(18,-90,right),
    straight(reverse,1),
    straight(12),
    straight(reverse,1),
    straight(12),
    straight(reverse,1),
    straight(14)
  });
  chain({ 
    swingToHeading(right,105,fwd,1),
    straight(-24),
    swingToHeading(left,265,fwd,1),
    toggleDescore,
    straight(reverse,1),
    straight(14),
    straight(reverse,1)
  });
 chain({
    straight(4),
    toggleDescore,
    turnToHeading(90),
    straight(-3),
    swing(left,180,0.5),
    swingToHeading(left,270,fwd,1),
    toggleDescore,
    straight(reverse,1),
    straight(12),
    straight(reverse,1)
  });
  chain({
    straight(2),
    toggleDescore,
    turnToHeading(7),
    straight(38),
    turnToHeading(90),
    arc(16,90,right),
    straight(fwd,1),
    straight(-8),
    straight(fwd,1),
    straight(-8),
    turnToHeading(0),
    straight(reverse,1)
  });
  chain({
    straight(4),
    swingToHeading(right,75),
    straight(-40),
    swing(right,-165),
    toggleDescore,
    straight(reverse,1),
    straight(6),
    toggleDescore
  });

}

void testPID(void) {
  for(int i=6; i>0; i--) {
    for(int j=0; j<i; j++) {
      run(swing(right,-360/i));
      //straight(12);
    }
  }
  //straight(70);
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
      Chassis::leftGroup.setVelocity(0, percent);
    } else {
      Chassis::leftGroup.setVelocity(leftMotorSpeed, percent);
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
      Chassis::rightGroup.setVelocity(0, percent);
    } else {
      Chassis::rightGroup.setVelocity(rightMotorSpeed, percent);
    }
    // Spin all drivetrain motors in the forward direction.
    Chassis::leftGroup.spin(forward);
    Chassis::rightGroup.spin(forward);
    //Controller1.Screen.setCursor(0,0);
    //Controller1.Screen.clearLine();
    //Controller1.Screen.print("%.1f, %.1f, %.1f, %.1f, %.1f", currentPosition.x, currentPosition.y, -inertialSensor.rotation(), forwardTracking.position(deg), sideTracking.position(deg));
    wait(0.025,sec);
  }
}
void driverSkills(void) {
  hang = false;
  //vex::task intake(releaseIntake);
  setInertial(90);
  orientation = -90;
  //odom = task(runOdom);
  currentPosition = {36,12};
  chain({
    arc(13.5,90,left),
    straight(reverse,0.5)
  });
  chain({
    turnToHeading(180,0.25),
    straight(10.5,80,180),
    turnToHeading(67)
  });
  run(straight(-2));
  run(turnToHeading(67));
  toggleDescore();
  toggleCata();
  usercontrol();
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  using namespace vex;
  typedef void (*callback)();
  callback progs[6] = {oppositeSide, oppositeSideUnsafe, AWPSameSide, sameSide, programmingSkills};
  std::string progNames[6] = {"Safe Opposite", "Cool Opposite", "Safe Same", "Cool Same", "Skills"};
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