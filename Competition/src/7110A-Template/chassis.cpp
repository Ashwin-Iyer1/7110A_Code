#include "vex.h"
#include <functional>
#include <vector>
using namespace vex;
namespace Chassis {
    vex::motor_group leftGroup;
    vex::motor_group rightGroup;
    vex::inertial inertialSensor = inertial(PORT1);
    float wheelDiameter;
    float gearRatio;
    float drivetrainWidth;
    void init(vex::motor_group leftMotors, vex::motor_group rightMotors) {
        leftGroup = leftMotors;
        rightGroup = rightMotors;
    }
    void init(vex::motor_group leftMotors, vex::motor_group rightMotors, int32_t inertialPort, float wheelDiam, float gr, float driveWidth) {
        leftGroup = leftMotors;
        rightGroup = rightMotors;
        inertialSensor = inertial(inertialPort);
        wheelDiameter = wheelDiam;
        gearRatio = gr;
        drivetrainWidth = driveWidth;
        inertialSensor.calibrate();
        while (inertialSensor.isCalibrating()) {
            wait(100, msec);
        } 
        inertialSensor.resetHeading();
    }
    double signum(double num) {
    if(num<0) return -1;
    else if(num>0) return 1;
    else return 0;
    }
    float distToRot(float dist) {
    return (dist/(wheelDiameter * M_PI)*360) / gearRatio;
    }
    float rotToDist(float rot) {
    return (rot/360*(wheelDiameter * M_PI)) * gearRatio;
    }
    //moves robot straight for distance in inches
    void brakeAll() {
        leftGroup.setStopping(brakeType::brake);
        rightGroup.setStopping(brakeType::brake);
    }
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
    void setInertial(float d) {
        inertialSensor.setHeading(d, deg);
        inertialSensor.setRotation(d, deg);
    }
}