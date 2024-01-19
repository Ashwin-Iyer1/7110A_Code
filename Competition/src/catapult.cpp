#include "vex.h"
#include "robot-config.cpp"
using namespace vex;
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
  bringCataDown(50);
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