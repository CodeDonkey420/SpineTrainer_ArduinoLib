#include <SPI.h>
#include <mcp2515.h>
#include <SpineTrainer.h>

Vertebrae* slave;

void setup() {

  slave = new Vertebrae(10);

  // test values
  slave->quats[0] = 420.0;
  slave->quats[1] = 69.0;
  slave->quats[2] = 42.0;
  slave->quats[3] = 7.0;
  
  Serial.println("------- CAN SLAVE ----------");

  slave->sendDiscovery();

  Serial.println("Discovery finished!");
  Serial.print("New ID: ");
  Serial.println(slave->id);
}

void loop() {
  Serial.println("Awaiting request...");
  slave->awaitRequest();
  delay(10);
}
