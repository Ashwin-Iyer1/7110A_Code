#include "vex.h"
#include "robot-config.cpp"
#include "motors-only-movement.cpp"
#include "smart-movement.cpp"
#include "sylib/system.hpp"
#include "sylib/sylib.hpp"
#include "catapult.cpp"

using namespace vex;

void pre_auton(void) {
  sylib::initialize();
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  } 
  rotationSensor.resetPosition();
  inertialSensor.resetHeading();
  Catapult.setStopping(brakeType::coast);
  Catapult.setVelocity(100,pct);
  Intake.setVelocity(100,pct);
  leftGroup.setVelocity(50, percent);
  rightGroup.setVelocity(50, percent);
}
void oppositeSide(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(3, 35);
  wait(1,sec);
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
  toggleBlocker();
  straight(-38);
  smartTurn(-20);
}
void oppositeSideUnsafe(void) {
  //start close to left of tile touching wall
  // vex::task run(bringCataDown);
  Intake.setVelocity(100,pct);
  // score alliance triball to near net    
  inertialSensor.setHeading(270,deg); 
  Intake.spin(fwd);   
  straight(3, 35);
  wait(0.5,sec);
  straight(-28,75);
  Intake.stop();
  toggleWings();
  arc(16.5,-90,right);
  toggleWings();
  wait(0.5,sec);
  turnToHeading(205);
  slam(reverse);
  turnToHeading(180);
  arc(12,180,right);
  // elim diff
  straight(30);
  turnToHeading(80);
  Intake.spin(reverse);
  straight(-8);
  turnToHeading(255);
  Intake.spin(fwd);
  straight(18);
  straight(-8);
  turnToHeading(72.5);
  Intake.spin(reverse);
  slam(fwd);
  Intake.stop();
  //outtake both balls
  straight(-4);
  turnToHeading(50);
  toggleBlocker();
  straight(-40);
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
  toggleWings();
  toggleCata();
  float realOrientation = inertialSensor.heading(deg);
  wait(33,sec);
  toggleCata();
  toggleWings();
  inertialSensor.setHeading(realOrientation,deg);
  straight(4);
  turnToHeading(315);
  arc(120,-17,right);
  turnToHeading(270);
  
  straight(-54);
  arc(22.5,-90,right);
  turnToHeading(180);
  slam(reverse);
  straight(4);
  slam(reverse);
  //push side triballs in
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

void usercontrol(void) {
  
    std::uint32_t clock = sylib::millis();

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
  auto block = sylib::Addrled(22,8,40);
  BlockerLEDS = &block;
  auto und1 = sylib::Addrled(22,7,14);
  Under1 = &und1;
  auto und2 = sylib::Addrled(22,6,13);
  Under2 = &und2;
  auto top = sylib::Addrled(22,5,23);
  Top = &top;
  vex::task leds(handleLEDs);
  while (true) {
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