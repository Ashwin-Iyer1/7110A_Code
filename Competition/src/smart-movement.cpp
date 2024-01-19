#include "vex.h"
#include "odometry.cpp"
#include "robot-config.cpp"
#include "motors-only-movement.cpp"

using namespace vex;

bool smartTurn(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.7;
  float kd = 0.01;
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
bool moveToPoint(float xPos, float yPos) {
  smartTurn(-atanf((xPos-x)/(yPos-y))/M_PI*180);
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
    return (leftGroup.velocity(rpm)+rightGroup.velocity(rpm))/2*gearRatio/60*wheelDiameter*M_PI;
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