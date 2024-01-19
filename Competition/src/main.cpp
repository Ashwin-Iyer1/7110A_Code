/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       s617995                                                   */
/*    Created:      10/3/2023, 8:15:04 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.cpp"
#include "catapult.cpp"
#include "motors-only-movement.cpp"
#include "smart-movement.cpp"
#include "odometry.cpp"
#include "leds.cpp"
#include "7110A-functions.cpp"
#include "sylib/sylib.hpp"
#include "sylib/addrled.hpp"
#include "vex_motor.h"

using namespace vex;

//
// Main will set up the competition functions and callbacks.
//
int main() {
  typedef void (*callback)();
  callback progs[6] = {oppositeSide, oppositeSideElim, AWPSameSide, sameSide, programmingSkills, testing};
  std::string progNames[6] = {"Safe Opposite", "Cool Opposite", "Safe Same", "Cool Same", "Skills", "test"};
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  //Competition.autonomous(programmingSkills);
  //Competition.autonomous(oppositeSide);
  Competition.autonomous(oppositeSideElim);
  //Competition.autonomous(sameSide);
  //Competition.autonomous(AWPSameSide);
  // Competition.autonomous(testing);
  Competition.drivercontrol(usercontrol);
  //Competition.drivercontrol(driverSkills);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}