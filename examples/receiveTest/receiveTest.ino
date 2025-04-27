#include <SPI.h>
#include <mcp2515.h>
#include <SpineTrainer.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("ID: ");
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" DLC: ");
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" DATA: ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.print(" => ");

    vertMessage testMessage;
    int id;

    unpack_message(&canMsg, &testMessage, &id);
    Serial.print(" TYPE: ");
    Serial.print(testMessage.type);
    Serial.print(" INDEX: ");
    Serial.print(testMessage.index);
    Serial.print(" QUAT: ");
    Serial.print(testMessage.quat);

    Serial.println();      
  }
}
