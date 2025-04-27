#include <SPI.h>
#include <mcp2515.h>
#include <SpineTrainer.h>

struct can_frame canMsg;
vertMessage testMessage = {.type=REGULAR_REPORT, .index=2, .quat=420.69};
MCP2515 mcp2515(10);


void setup() {
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  pack_message(&canMsg, &testMessage, 42);
  mcp2515.sendMessage(&canMsg);

  Serial.println("Messages sent");
  
  delay(1000);
}
