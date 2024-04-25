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

competition Competition;
controller Controller1 = controller(primary);
brain Brain;

sylib::Addrled* Under1;
sylib::Addrled* Under2;
sylib::Addrled* Top;

double signum(double num) {
  if(num<0) return -1;
  else if(num>0) return 1;
  else return 0;
}
int currentSpeed = 10;
int pattern = 0;
//Patterns:
//1 - red gradient
//2 - orange gradient
//3 - yellow gradient
//4 - green gradient
//5 - blue gradient
//6 - purple gradient
//7 - pink gradient
//8 - gold gradient
//0 - rainbow
void togglePattern() {
  pattern = (pattern + 1) % 9;
  switch (pattern)
  {
  case 1:
    Top -> gradient(0,0xff0000);
    Under1 -> gradient(0,0xff0000);
    Under2 -> gradient(0,0xff0000);
    break;
  case 2:
    Top -> gradient(0,0xffa5000, 0, false, true);
    Under1 -> gradient(0,0xffa5000, 0, false, true);
    Under2 -> gradient(0,0xffa5000, 0, false, true);
    break;
  case 3:
    Top -> gradient(0,0xffff00, 0, false, true);
    Under1 -> gradient(0,0xffff00, 0, false, true);
    Under2 -> gradient(0,0xffff00, 0, false, true);
    break;
  case 4:
    Top -> gradient(0,0x00ff00, 0, false, true);
    Under1 -> gradient(0,0x00ff00, 0, false, true);
    Under2 -> gradient(0,0x00ff00, 0, false, true);
    break;
  case 5:
    Top -> gradient(0,0x0000ff, 0, false, true);
    Under1 -> gradient(0,0x0000ff, 0, false, true);
    Under2 -> gradient(0,0x0000ff, 0, false, true);
    break;
  case 6:
    Top -> gradient(0,0x800080, 0, false, true);
    Under1 -> gradient(0,0x800080, 0, false, true);
    Under2 -> gradient(0,0x800080, 0, false, true);
    break;
  case 7:
    Top -> gradient(0,0xff69b4, 0, false, true);
    Under1 -> gradient(0,0xff69b4, 0, false, true);
    Under2 -> gradient(0,0xff69b4, 0, false, true);
    break;
  case 8:
    Top -> gradient(0,0xffd700, 0, false, true);
    Under1 -> gradient(0,0xffd700, 0, false, true);
    Under2 -> gradient(0,0xffd700, 0, false, true);
    break;
  case 0:
    Top -> gradient(0x990000, 0x990005, 0, 0, false, true);
    Under2 -> gradient(0x990000, 0x990005, 0, 0, false, true);
    Under1 -> gradient(0x990000, 0x990005, 0, 0, false, true);
    break;
  }
  Top -> cycle(**Top, currentSpeed);
  Under1 -> cycle(**Under1, currentSpeed);
  Under2 -> cycle(**Under2, currentSpeed);
}
void increaseSpeed() {
  currentSpeed+=5;
  Top -> cycle(**Top, currentSpeed);
  Under1 -> cycle(**Under1, currentSpeed);
  Under2 -> cycle(**Under2, currentSpeed);
}
void decreaseSpeed() {
  currentSpeed-=5;
  Top -> cycle(**Top, currentSpeed);
  Under1 -> cycle(**Under1, currentSpeed);
  Under2 -> cycle(**Under2, currentSpeed);
}
// Main will set up the competition functions and callbacks.
//
void usercontrol(void) {
  auto top = sylib::Addrled(22,6,15);
  Top = &top;
  auto under1 = sylib::Addrled(22,7,20);
  Under1 = &under1;
  auto under2 = sylib::Addrled(22,8,20);
  Under2 = &under2;
  Top -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Top -> cycle(**Top, 10);
  Under2 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Under2 -> cycle(**Under2, 10);
  Under1 -> gradient(0x990000, 0x990005, 0, 0, false, true);
  Under1 -> cycle(**Under1, 10);
  Controller1.ButtonRight.pressed(togglePattern);
  Controller1.ButtonUp.pressed(increaseSpeed);
  Controller1.ButtonDown.pressed(decreaseSpeed);
  while (true) {
    wait(25, msec);
  }
}

int main() {
  sylib::initialize();
  Competition.autonomous(usercontrol);
  Competition.drivercontrol(usercontrol);
  while(true) {
    wait(100,msec);
  }
}