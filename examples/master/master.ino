#include <SPI.h>
#include <SpineTrainer.h>

// poll values every 1000 millis
#define DELAY 10000

Master* master;

volatile bool interrupt = false;
struct can_frame frame;

void irqHandler() {
    interrupt = true;
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(2), irqHandler, FALLING);

  master = new Master(10);
  
  Serial.println("------- CAN MASTER ----------");
}

void loop() {
  if (interrupt) {
    Serial.println("------- INTERRUPT ----------");
    master->handleReceive();
    interrupt = false;
    Serial.println("Interrupt handled");
  }
  else {
    for (int vert_id = 0; vert_id < master->getNumConnected(); vert_id++) {
      for (int index = 0; index < 4; index++) {
        if (master->last_timestamps[0][index] + DELAY < millis()) {
          Serial.println("Requesting quats...");
          master->requestQuats(0, index);
          delay(10);
        }
      }
    }
    
    
    //delay(10000);
  }
}
