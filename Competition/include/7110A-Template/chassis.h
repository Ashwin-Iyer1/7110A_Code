#include "vex.h"
#include <functional>
#include <vector>

namespace Chassis {
    extern vex::motor_group leftGroup;
    extern vex::motor_group rightGroup;
    extern vex::inertial inertialSensor;
    extern float wheelDiameter;
    extern float gearRatio;
    extern float drivetrainWidth;

    void init(vex::motor_group leftMotors, vex::motor_group rightMotors);
    void init(vex::motor_group leftMotors, vex::motor_group rightMotors, int32_t inertialPort, float wheelDiam, float gr, float driveWidth);
    void setInertial(float d);

    double signum(double num);
    float distToRot(float dist);
    float rotToDist(float rot);
    void brakeAll();

    std::function<void()> straight(float distance, float speed = 80, float heading = inertialSensor.heading());
    std::function<void()> straight(vex::directionType direction, float time, vex::timeUnits units = vex::timeUnits::msec, float heading = inertialSensor.heading());
    std::function<void()> slam(vex::directionType direction, float currentRotation = inertialSensor.rotation());
    std::function<void()> turn(float rotation, float timeout = 4, float endError = 2);
    std::function<void()> turnToHeading(float heading, float timeout = 4, float endError = 2);
    std::function<void()> swing(vex::turnType side, float rotation, float timeout = 4, float endError = 2);
    std::function<void()> swingToHeading(vex::turnType side, float heading, vex::directionType direction = vex::directionType::fwd, float timeout = 4, float endError = 2);
    std::function<void()> arc(float radius, float rotation, vex::turnType side);
    std::function<void()> arc(float radius, float rotation, vex::turnType side, float timeout);
    
    void run(std::function<void()> func);
    void chain(const std::vector<std::function<void()>> funcs);
}