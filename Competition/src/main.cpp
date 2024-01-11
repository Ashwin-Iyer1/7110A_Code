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
#include "vex_motor.h"
#include <cmath>

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
pot potentiometer = pot(Brain.ThreeWirePort.C);

motor_group leftGroup = motor_group(FrontLeft, BackLeft, TopLeft);
motor_group rightGroup = motor_group(FrontRight, BackRight, TopRight);
motor_group Catapult = motor_group(Catapult1, Catapult2);

// sylib::Addrled BlockerLEDS;
// sylib::Addrled Under1;
// sylib::Addrled Under2;
// sylib::Addrled Top;


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
  if(Blocker.value()) {
    // BlockerLEDS.gradient(0xFF0000, 0xFFFFFF, 0, 0, false, true);
    // BlockerLEDS.cycle(*BlockerLEDS, 10);
  } else {
    // BlockerLEDS.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    // BlockerLEDS.cycle(*BlockerLEDS, 10);
  }
}

float gearRatio = 36.0/84.0;
float wheelDiameter = 4.125;
float wheelRadius = wheelDiameter/2;
float robotRadius = 6.25;
float drivetrainWidth = 12.5;
float prevLeftRotation = 0;
float prevRightRotation = 0;
float orientation = 0;
float orientationHeading = fmod(orientation+36000,360);
float x = 0;
float y = 0;
double driveRotationConstant = 0.8721445746*1.054572148;
vex::task odom;

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
  float leftPos = (leftGroup.position(deg)-prevLeftRotation)/180*M_PI * wheelRadius * gearRatio;
  float rightPos = (rightGroup.position(deg)-prevRightRotation)/180*M_PI * wheelRadius * gearRatio;
  orientation += ((leftPos-rightPos)/drivetrainWidth)/M_PI*180;
  orientationHeading = fmod(orientation+36000,360);
  float radOrientation = orientation/180*M_PI;
  if(fabs((leftPos-rightPos)/drivetrainWidth) < 0.005) {
    x += (leftPos+rightPos)/2 * cosf(-radOrientation + M_PI/2);
    y += (leftPos+rightPos)/2 * sinf(-radOrientation + M_PI/2);
  } else {
    float radius = (leftPos/((leftPos-rightPos)/drivetrainWidth)) - drivetrainWidth/2;
    //x += -radius * (sinf(-(radOrientation-M_PI/2)) - sinf(-((radOrientation - ((leftPos-rightPos)/drivetrainWidth))-M_PI/2)));
    x += sinf(radOrientation) * 2 * sinf((leftPos-rightPos)/drivetrainWidth/2) * (rightPos/((leftPos-rightPos)/drivetrainWidth) + drivetrainWidth/2);
    //y += radius * (cosf(-(radOrientation-M_PI/2)) - cosf(-((radOrientation - ((leftPos-rightPos)/drivetrainWidth))-M_PI/2)));
    x += cosf(radOrientation) * 2 * sinf((leftPos-rightPos)/drivetrainWidth/2) * (rightPos/((leftPos-rightPos)/drivetrainWidth) + drivetrainWidth/2);
  }
  prevLeftRotation = leftGroup.position(deg);
  prevRightRotation = rightGroup.position(deg);
}
int runOdom() {
  while(true) {
    odomUpdate();
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("o: %.1f x: %.1f y: %.1f", orientationHeading, x, y);
    wait(50,msec);
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
  for(double t=0; t<fabs(fmax(leftArc,rightArc))/20; t+=0.025) {
    if(!leftGroup.isSpinning() && !rightGroup.isSpinning()) break;
    wait(0.025,sec);
  }
  leftGroup.stop();
  rightGroup.stop();
}
bool moveToPoint(float xPos, float yPos) {
  turnToHeadingOdom(-atanf((xPos-x)/(yPos-y))/M_PI*180);
  straight(sqrtf(pow(xPos-x,2) + pow(yPos-y,2)));
  return true;
}
bool followPath(const std::vector<std::vector<int>>& points) {
  for(const auto& coordinates: points) {
    float xTarget = coordinates[0];
    float yTarget = coordinates[1];
    float e = 0;
    float d = 0;
    float i = 0;
    float eRec = 0;
    float kp = 0.03;
    float kd = 0.0;
    float ki = 0.015;
    float dt = 0.05;
    float angle = atan((xTarget-x)/(yTarget-y))*180/M_PI;
    float setpoint = angle + ((yTarget-y) > 0 ? 0 : 180);
    while(pow(xTarget-x,2) + pow(yTarget-y,2) > 25) {
      e = setpoint-fmod(orientation,360);
      d = (e-eRec)/dt;
      i += e*dt;
      eRec = e;
      leftGroup.setVelocity(40 - (e*kp + d*kd + i*ki),pct);
      rightGroup.setVelocity(40 + (e*kp + d*kd + i*ki),pct);
      leftGroup.spin(fwd);
      rightGroup.spin(fwd);
      Controller1.Screen.clearLine();
      Controller1.Screen.print("%.1f, %.1f, %.1f", x, y, pow(xTarget-x,2) + pow(yTarget-y,2));
      wait(dt,sec);
    }
  }
  leftGroup.stop();
  rightGroup.stop();
  return true;
}
struct Vector2d {
    double x, y;
};

struct Robot {
  double left_motor_speed;
  double right_motor_speed;
  double calcLinearSpeed(float pct) {
    return 55;
  }
};

class BezierSpline {
public:
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

private:
    Vector2d P0, P1, P2;
};

class RobotController {
  public:
    RobotController(Robot& robot) : robot(robot) {}

    void moveRobot(const Vector2d& middle_position, const Vector2d& target_position) {
      double max_speed = 50.0;
      double lookAhead = 12.0;

      Vector2d current_position = getCurrentPosition();
      BezierSpline spline(current_position, middle_position, target_position);

      Vector2d ogtan = spline.calculateTangent(0);
      turnToHeading(-atan2(ogtan.y, ogtan.x) / M_PI * 180);
      leftGroup.spin(fwd);
      // leftGroup.setVelocity(20, pct);
      rightGroup.spin(fwd);
      // rightGroup.setVelocity(20, pct);
      //splinePos is the percent of the track the robot has completed
      float kp = 0.05;
      float ki = 0.005;
      float kd = 0.01;
      float e = 0;
      float d = 0;
      float i = 0;
      float eRec = 0;
      Controller1.Screen.print("DOES THIS WORK");
      double distance = sqrt((target_position.x-x)*(target_position.x-x) + (target_position.y-y)*(target_position.y-y));
      Controller1.Screen.print("STILL WORKING");
      wait(0.5,sec);
      double splineLength = spline.approxLength(10);
      Controller1.Screen.clearLine();
      Controller1.Screen.print(splineLength);
      wait(0.5,sec);
      double timeStep = 0.1;
        double splinePos = 0.0;
        while (splinePos < 1-(lookAhead/splineLength)) {
          
          Vector2d desired_position = spline.calculatePoint(splinePos+(lookAhead/splineLength));
          Vector2d tangent = spline.calculateTangent(splinePos+(lookAhead/splineLength));
          
          // Simple control logic: adjust left and right motor speeds based on tangent
          double angle = std::atan2(tangent.y, tangent.x); //angle of tangent vector in radians
          // Brain.Screen.print(angle);
          // Brain.Screen.newLine();
          // Brain.Screen.print(orientationHeading);
          angle = angle / M_PI * 180; //convert to degrees
          double angle_difference = fmod(-orientationHeading - angle, 360);
          if(fabs(angle_difference) > 180) angle_difference = (360-angle_difference*(angle_difference>0)? 1 : -1);
          e = angle_difference;
          d = (e-eRec)/timeStep;
          i += (e*timeStep);
          eRec = e;
          //distance = sqrt((desired_position.x-x)*(desired_position.x-x) + (desired_position.y-y)*(desired_position.y-y));
          float speed = 2 * sinf((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth/2) * (rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60/((leftGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60-rightGroup.velocity(rpm)/gearRatio*wheelDiameter*M_PI/60)/drivetrainWidth) + drivetrainWidth/2);
        
          //1 pct speed diff in 0.1 seconds = 0.3 degrees change
          double speed_difference = ki*i + kd*d + kp*e;

          Controller1.Screen.clearLine();
          //Controller1.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);
          Controller1.Screen.print(angle_difference);
          double left_speed = max_speed - speed_difference;
          double right_speed = max_speed + speed_difference;

          // Send motor commands to the robot
          robot.left_motor_speed = left_speed;
          robot.right_motor_speed = right_speed;
          // Brain.Screen.print("left: ");
          // Brain.Screen.print(left_speed);
          // Brain.Screen.print("right: ");
          // Brain.Screen.print(right_speed);

          // Brain.Screen.newLine();

          //Brain.Screen.newLine();
          //Brain.Screen.print("d: %.1f a: %.1f t: %.1f", distance, angle_difference, time_diff);

          // Move forward in time
          splinePos += (speed*timeStep)/splineLength;

          // Simulate robot movement (you may replace this with your actual motion control logic)
          leftGroup.setVelocity(robot.left_motor_speed, pct);
          leftGroup.spin(fwd);
          rightGroup.setVelocity(robot.right_motor_speed, pct);
          rightGroup.spin(fwd);
          wait(timeStep, sec);
        }
        straight(lookAhead);

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
        return { x, y };
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
vex::task LED;
int LEDRainbow() {
  auto BlockerLEDS = sylib::Addrled(22,8,27);
  auto Under1 = sylib::Addrled(22,7,14);
  auto Under2 = sylib::Addrled(22,6,13);
  auto Top = sylib::Addrled(22,5,23);
  
    std::uint32_t clock = sylib::millis();
  BlockerLEDS.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    BlockerLEDS.cycle(*Top, 10);

    Top.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Top.cycle(*Top, 10);



    Under1.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under1.cycle(*Under1, 10);


    Under2.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under2.cycle(*Under2, 10);
  while (true) {
    sylib::delay_until(&clock, 10);
  }
  return 1;
}
void oppositeSide(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(75,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(3, 35);
  wait(1.5,sec);
  straight(-24,75);
  // Intake.stop();
  toggleWings();
  arc(16.5,-100,right);
  toggleWings();
  turnToHeading(205);
  slam(reverse);
  straight(8);
  slam(reverse);
  turnToHeading(180);
  arc(12,180,right);
  straight(30);
  
  turnToHeading(80);
  Intake.spin(reverse);
  wait(1, sec);
  straight(-5);
  /*
  straight(13);
  wait(0.3, sec);
  straight(-5);
  wait(0.1, sec);
  straight(8);
  wait(0.1, sec);
  Intake.stop();
  straight(-13);
  turnToHeading(35);
  Intake.spin(fwd);
  toggleBlocker();
  straight(-27);
  smartTurn(15);
  */
 turnToHeading(260);
 slam(reverse);
 straight(5);
}
void oppositeSideElim(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(3, 35);
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
  wait(0.3, sec);
  straight(-8);
  turnToHeading(255);
  Intake.spin(fwd);
  straight(18);
  wait(0.1, sec);
  straight(-8);
  turnToHeading(85);
  Intake.spin(reverse);
  straight(-13);
  turnToHeading(30);
  toggleBlocker();
  straight(-30);
}
void sameSide(void) {
  vex::task run(LEDRainbow);
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
  arc(13, 180, right);
  straight(27);
  turnToHeading(0);
  toggleWings();
  arc(16.5, -90, right);
  toggleWings();
  turnToHeading(315);
  straight(-8);
  turnToHeading(270);
  Blocker.set(true);
  straight(-22);
  straight(-12,50);
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
  turnToHeading(180);
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
  toggleWings();
  arc(15,102.5,left);
  slam(reverse);
  straight(6);
  slam(reverse);
  straight(8);
  toggleWings();
  arc(50,40,right);
  turnToHeading(0);
  straight(40);
  turnToHeading(315);
  toggleWings();
  arc(70,-45,right);
  straight(10);
  toggleWings();
  straight(50);
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
  odom = vex::task(runOdom);
  orientation = 0;
  x = 0;
  y = 0;
  
  // followPath({
  //   {48,70},
  //   {36,104},
  //   {40,130},
  //   {108,130}
  // });
  Robot myRobot{ 0.0, 0.0 };
    RobotController controller(myRobot);

    // Move the robot along a Bezier spline to the target position
    controller.moveRobot({ 18, 6 }, { 12, 18 });
    //left 0, right 10 for 1 second = 21 degrees
  // leftGroup.spin(fwd);
  // rightGroup.spin(fwd);
  // leftGroup.setVelocity(90, pct);
  // rightGroup.setVelocity(100, pct);
  // wait(1, sec);
  // leftGroup.stop();
  // rightGroup.stop();

}
/*
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
*/
void usercontrol(void) {


  Intake.setVelocity(100,pct);
  int deadband = 5;
  bool intakeMode = true;
  x = 35;
  y = 7;
  Controller1.ButtonB.pressed(toggleCata);
  Controller1.ButtonA.released(stopCata);
  Controller1.ButtonY.pressed(toggleWings);
  Controller1.ButtonX.pressed(toggleBlocker);
  // Controller1.ButtonLeft.pressed(cataMatchLoad);
  odom = vex::task(runOdom);
  auto BlockerLEDS = sylib::Addrled(22,8,27);
  auto Under1 = sylib::Addrled(22,7,14);
  auto Under2 = sylib::Addrled(22,6,13);
  auto Top = sylib::Addrled(22,5,23);
  
    std::uint32_t clock = sylib::millis();

  while (true) {
    if(Controller1.ButtonRight.pressing()) {
    BlockerLEDS.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    BlockerLEDS.cycle(*Top, 10);

    Top.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Top.cycle(*Top, 10);



    Under1.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under1.cycle(*Under1, 10);


    Under2.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
    Under2.cycle(*Under2, 10);

  }
  else if(Controller1.ButtonLeft.pressing()){
    Top.set_all(0xFF0000);
    Under1.set_all(0xFF0000);
    Under2.set_all(0xFF0000);
    BlockerLEDS.set_all(0xFF0000);
  }
    //tank drive
    sylib::delay_until(&clock, 10);
    // Get the velocity percentage of the left motor. (Axis3)
    //int leftMotorSpeed = intakeMode ? Controller1.Axis3.position() : (-Controller1.Axis2.position());
    // Get the velocity percentage of the right motor. (Axis2)
    //int rightMotorSpeed = intakeMode ? Controller1.Axis2.position() : (-Controller1.Axis3.position());


    //split drive
    int leftMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? -1 : 1) * Controller1.Axis1.position());
    int rightMotorSpeed = (intakeMode ? -1 : 1) * (Controller1.Axis3.position() + (intakeMode ? 1 : -1) * Controller1.Axis1.position());

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
  //Competition.autonomous(oppositeSideElim);
  // Competition.autonomous(sameSide);
  //Competition.autonomous(AWPSameSide);
  // Competition.autonomous(testing);
  //if(Competition.isEnabled()) selectAuton();
  //Competition.drivercontrol(usercontrol);
  //Competition.drivercontrol(driverSkills);
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