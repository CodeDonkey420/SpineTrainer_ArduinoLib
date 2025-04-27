
//=============================================================================================
// SpineTrainer.h
//=============================================================================================
//
// Project library for HS21 MES module
//
// Date			  Author			  Notes
// 12/04/2024	Maxim Walther	initial release
// XX/XX/XXXX 
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Headers

#include "SpineTrainer.h" // SF
#include "Arduino.h"

//-------------------------------------------------------------------------------------------
// Definitions
#define max_vertebrae 12
#define DEBUG true
 
//============================================================================================
// Functions
void pack_message(can_frame* frame, vertMessage* content, int id) {
  frame->can_id = id;
  frame->can_dlc = 8;
  const byte * p = (const byte*) content;
  for (int i = 0; i < 8; i++) {
    frame->data[i] = *p++;
  }
}

void unpack_message(can_frame* frame, vertMessage* content, int* id) {
  byte * p = (byte*) content;
  *id = frame->can_id;
  unsigned int i;
    for (i = 0; i < 8; i++)
      *p++ = frame->data[i];
 }

void print_debug(can_frame* frame) {
  Serial.print("ID: ");
  Serial.print(frame->can_id, HEX); // print ID
  Serial.print(" DLC: ");
  Serial.print(frame->can_dlc, HEX); // print DLC
  Serial.print(" DATA: ");

  for (int i = 0; i<frame->can_dlc; i++)  {  // print the data
    Serial.print(frame->data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void print_debug(can_frame* frame, vertMessage* Msg) {
  Serial.print("ID: ");
  Serial.print(frame->can_id, HEX); // print ID
  Serial.print(" DLC: ");
  Serial.print(frame->can_dlc, HEX); // print DLC
  Serial.print(" DATA: ");

  for (int i = 0; i<frame->can_dlc; i++)  {  // print the data
    Serial.print(frame->data[i],HEX);
    Serial.print(" ");
  }

  Serial.print(" => ");

  vertMessage testMessage = *Msg;
  Serial.print(" TYPE: ");
  Serial.print(testMessage.type);
  Serial.print(" INDEX: ");
  Serial.print(testMessage.index);
  Serial.print(" QUAT: ");
  Serial.print(testMessage.quat);
  Serial.println();
}

Vertebrae::Vertebrae(int cs_pin) : CAN_IO(cs_pin) {
  Serial.begin(115200);
  CAN_IO.reset();
  CAN_IO.setBitrate(CAN_125KBPS, MCP_8MHZ);
  CAN_IO.setNormalMode();
  id = 99;
}

void Vertebrae::sendDiscovery() {
  struct can_frame canMsg;
  // send discovery MSG with initial vals
  for(int i = 0; i < 4; i++) {
    vertMessage quatMessage = {.type=DISCOVERY_MSG, .index=i, .quat=quats[i]};

    pack_message(&canMsg, &quatMessage, id);
    CAN_IO.sendMessage(&canMsg);
    if (DEBUG) {
      print_debug(&canMsg, &quatMessage);
    }

    bool answered = false;
    while(!answered) {
      if (CAN_IO.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        // Check for Discovery Answer
        if (DEBUG) {
          print_debug(&canMsg);
        }

        if (canMsg.can_id == id) {
          if (canMsg.data[7] == 0xFF) {
            pack_message(&canMsg, &quatMessage, id);
            CAN_IO.sendMessage(&canMsg);
            continue;
          }
          answered = true;
        }
      }
      delay(10);
    }
  }
  Serial.println("Discovery sent...");
  // await answer + get id
  bool answered = false;
  while(!answered) {
    if (CAN_IO.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      // Check for Discovery Answer
      if (DEBUG) {
        print_debug(&canMsg);
      }

      if (canMsg.can_id == 0 && canMsg.data[0] == 0xFF) {
        // get new id
        byte * p = (byte*) &id;
        unsigned int i;
        for (i = 4; i < 8; i++)
          *p++ = canMsg.data[i];
        answered = true;
      }
    }
    delay(10);
  }
}

void Vertebrae::sendQuats(int index) {
  struct can_frame canMsg;
  // send individual quads
  vertMessage quatMessage = {.type=REGULAR_REPORT, .index=index, .quat=quats[index]};

  pack_message(&canMsg, &quatMessage, id);
  CAN_IO.sendMessage(&canMsg);
}

void Vertebrae::awaitRequest() {
  struct can_frame canMsg;
  while (true) {
  if (CAN_IO.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (DEBUG) {
      print_debug(&canMsg);
    }
    if (canMsg.can_id == id) {
      int index;
      
      for (int i = 0; i < 4; i++) {
        if (canMsg.data[i] == 0xFF) {
          index = i;
          break;
        }
      }

      if (DEBUG) {
        Serial.println("Answering!");
      }
      sendQuats(index);
    }
  }
  }
}

Master::Master(int cs_pin) : CAN_IO(cs_pin) {
  CAN_IO.reset();
  CAN_IO.setBitrate(CAN_125KBPS, MCP_8MHZ);
  CAN_IO.setNormalMode();

  Serial.begin(115200);

  connected_verts = 0;
}

void Master::requestQuats(int vert_id, int index) {
  struct can_frame canMsg;
  canMsg.can_id = vert_id;
  canMsg.can_dlc = 8;
  for (int i = 0; i < 8; i++) {
    canMsg.data[i] = 0x00;
  }
  canMsg.data[index] = 0xFF;

  CAN_IO.sendMessage(&canMsg);
}

void Master::handleReceive() {
  Serial.println("Message received...");
  struct can_frame frame;
  vertMessage Msg;
  int vert_id;
  uint8_t irq = CAN_IO.getInterrupts();

  if (irq & MCP2515::CANINTF_RX0IF) {
    if (CAN_IO.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
      unpack_message(&frame, &Msg, &vert_id);
    }
  }

  if (irq & MCP2515::CANINTF_RX1IF) {
    if (CAN_IO.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
        // frame contains received from RXB1 message
      unpack_message(&frame, &Msg, &vert_id);
    }
  }

  if (Msg.type == DISCOVERY_MSG) {
    Serial.println("Discovery recognized...");
    float inits[4];
    inits[0] = Msg.quat;
    if (DEBUG) {
      Serial.println("-- DISCOVERY MESSAGE DEBUG --");
      print_debug(&frame, &Msg);
    }
    struct can_frame ackMsg;
    ackMsg.can_id = vert_id;
    ackMsg.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
      ackMsg.data[i] = 0x00;
    }
    // request send again
    //canMsg.data[7] = 0xFF;

    delay(10);
    CAN_IO.sendMessage(&ackMsg);
    delay(10);
    // await all other quats
    for (int i = 1; i < 4; i++) {
      struct can_frame canMsg;
      while (CAN_IO.readMessage(&canMsg) != MCP2515::ERROR_OK || (CAN_IO.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id != vert_id)) {
        if (DEBUG) {
          Serial.print("excluded: ");
          print_debug(&canMsg);
        }
      }

      // save info
      unpack_message(&canMsg, &Msg, &vert_id);
      inits[Msg.index] = Msg.quat;

      if (DEBUG) {
        print_debug(&canMsg, &Msg);
      }

      CAN_IO.sendMessage(&ackMsg);
      delay(10);
    } 
    Serial.print(inits[0]);Serial.print(" ; ");
    Serial.print(inits[1]);Serial.print(" ; ");
    Serial.print(inits[2]);Serial.print(" ; ");
    Serial.print(inits[3]);Serial.println(" ; ");
    // assign id
    answerDiscovery(Msg);
  }

  else {
    // save new quat info
    if (DEBUG) {
      Serial.println("-- MESSAGE DEBUG --");
      print_debug(&frame, &Msg);
    }
    // set timestamp of data get
    last_timestamps[0][Msg.index] = millis();
  }
  CAN_IO.clearInterrupts();
}

void Master::answerDiscovery(vertMessage Msg) {
  struct can_frame canMsg;
  canMsg.can_id = 0;
  canMsg.can_dlc = 8;
  for (int i = 0; i < 8; i++) {
    canMsg.data[i] = 0x00;
  }

  canMsg.data[0] = 0xFF;
  
  int newID = connected_verts++;
  const byte * p = (const byte*) &newID;
  for (int i = 4; i < 8; i++) {
    canMsg.data[i] = *p++;
  }

  CAN_IO.sendMessage(&canMsg);
}

int Master::getNumConnected() { return connected_verts; }

//============================================================================================
// END OF CODE
//============================================================================================
