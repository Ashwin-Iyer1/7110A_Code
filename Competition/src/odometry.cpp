#include "vex.h"
#include "robot-config.cpp"
#include <cmath>
using namespace vex;
float x = 0;
float y = 0;
vex::task odom;
float prevLeftRotation = 0;
float prevRightRotation = 0;
float orientation = 0;
float orientationHeading = fmod(orientation+36000,360);
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