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

#ifndef SPINETRAINER_h
#define SPINETRAINER_h

#include "Arduino.h"
#include <mcp2515.h>

#define max_vertebrae 12

enum messType {
  REGULAR_REPORT,
  DISCOVERY_MSG,
  PANIC
};

struct vertMessage {
  messType type:4;
  int index:4;
  float quat;
} __attribute__((packed));

void pack_message(can_frame* frame, vertMessage* content, int id);

void unpack_message(can_frame* frame, vertMessage* content, int* id);

class Vertebrae {
  public:
    Vertebrae(int cs_pin);
    void sendDiscovery();
    void sendQuats(int index);
    void awaitRequest();
    int id;
    float quats[4];
  private:
    MCP2515 CAN_IO;
};

class Master {
  public:
    Master(int cs_pin);
    void answerDiscovery(vertMessage Msg);
    void requestQuats(int id, int index);
    void handleReceive();
    float last_timestamps[max_vertebrae][4];
    int getNumConnected();
  private:
    MCP2515 CAN_IO;
    int connected_verts;
};

#endif
