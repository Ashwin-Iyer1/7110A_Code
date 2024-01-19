#include "vex.h"
#include "robot-config.cpp"
#include "catapult.cpp"
int handleLEDs() {
  bool prevBlocker = Blocker.value();
  bool idle = false;
  while(true) {
    if(Blocker.value() && !prevBlocker) {
        BlockerLEDS->set_all(0x000000);
        BlockerLEDS -> gradient(0x000000, 0xFF0000, 8, 0, false, false);
        BlockerLEDS -> cycle(**BlockerLEDS, 25);
        Top->set_all(0x000000);
        Top -> gradient(0x000000, 0xFF0000, 8, 0, false, false);
        Top -> cycle(**BlockerLEDS, 25);
        Under1->set_all(0xFF0000);
        Under2->set_all(0xFF0000);
        idle = false;
    } else if(catatoggle){
      int cataprogress = (int)rotationSensor.position(deg)*2.5;
      BlockerLEDS->set_all(cataprogress*65793);
      Top->set_all(cataprogress*65793);
      Under1->set_all(cataprogress*65793);
      Under2->set_all(cataprogress*65793);
      idle = false;
    } else if(!idle && !Blocker.value()){
      BlockerLEDS->set_all(0x000000);
      Top->set_all(0x000000);
      Under1->set_all(0x000000);
      Under2->set_all(0x000000);
      BlockerLEDS -> gradient(0x600000, 0x600002, 0, 0, false, true);
      BlockerLEDS -> cycle(**BlockerLEDS, 10);

      Top -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Top -> cycle(**Top, 10);



      Under1 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under1 -> cycle(**Under1, 10);


      Under2 -> gradient(0xFF0000, 0xFF0005, 0, 0, false, true);
      Under2 -> cycle(**Under2, 10);
      idle = true;
    }
    prevBlocker = Blocker.value();
    wait(50,msec);
  }
}