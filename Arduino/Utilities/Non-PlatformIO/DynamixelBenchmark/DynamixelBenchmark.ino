#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <vector>

DynamixelShield dxl;

const int bpsList[] = { 57600, 115200, 1000000 }; // Arduino can't go higher than 2Mbps and even that is unreliable according to the Dynamixel Shield website

const int numBps = 3;
const int reps = 1000;

typedef struct {
  int id;
  int modelNumber;
} Dynamixel;

int bps;
std::vector<Dynamixel> dynamixels;

const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t LEN_PRESENT_POSITION = 4;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t LEN_GOAL_POSITION = 4;
const uint16_t ADDR_GOAL_VELOCITY = 104; 
const uint16_t LEN_GOAL_VELOCITY = 4;

const uint8_t DXL_COUNT = 4;

const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

typedef struct sr_data {
  int32_t present_position;
} __attribute((packed)) sr_data_t;

typedef struct sw_data {
  int32_t goal_position;
} __attribute((packed)) sw_data_t;

sr_data_t sr_data[DXL_COUNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_COUNT];

sw_data_t sw_data[DXL_COUNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_COUNT];

void setupSyncBuffers() {
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = ADDR_PRESENT_POSITION;
  sr_infos.addr_length = LEN_PRESENT_POSITION;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  for(size_t i=0; i<dynamixels.size() && i<4; i++) {
    info_xels_sr[i].id = dynamixels[i].id;
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = ADDR_GOAL_POSITION;
  sw_infos.addr_length = LEN_GOAL_POSITION;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for(size_t i=0; i<dynamixels.size() && i<4; i++) {
    info_xels_sw[i].id = dynamixels[i].id;
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;
}

void runSyncGetSetPosition(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4) {
  uint8_t recv_cnt = dxl.syncRead(&sr_infos);
  if(recv_cnt != 4) Serial.println("Receiving failed!");

  sw_data[0].goal_position = p1;
  sw_data[1].goal_position = p2;
  sw_data[2].goal_position = p3;
  sw_data[3].goal_position = p4;
  sw_infos.is_info_changed = true;
  if(dxl.syncWrite(&sw_infos) == false) Serial.println("Writing failed!");
}

bool detectDynamixels() {
  dxl.setPortProtocolVersion(2.0);

  Serial.print("Detecting Dynamixels at ");
  for(int i=0; i<numBps; i++) {
    Serial.print(String(bpsList[i]) + "bps... ");
    dxl.begin(bpsList[i]);
    if(dxl.scan() == true) {
      Serial.println("success.");
      bps = bpsList[i];

      Serial.print("Enumerating bus... ");
      for(int j=1; j<254; j++) {
        if(dxl.ping(j)) {
          Serial.print(String(j) + "... ");
          dynamixels.push_back({j, dxl.getModelNumber(j)});
        }
      }
      Serial.println();

      return true;
    }
  }

  return false;
}

void runBasicGetSetPosition() {
  for(int i=0; i<dynamixels.size(); i++) {
    float presentPosition = dxl.getPresentPosition(dynamixels[i].id, UNIT_DEGREE);
  }
  for(int i=0; i<dynamixels.size(); i++) {
    dxl.setGoalPosition(dynamixels[i].id, 180.0, UNIT_DEGREE);
  }
}

void setStatusReturnLevel(uint32_t level) {
  for(int i=0; i<dynamixels.size(); i++) {
    dxl.writeControlTableItem(ControlTableItem::STATUS_RETURN_LEVEL, dynamixels[i].id, level);
  }
}

void setReturnDelayTime(uint32_t time) {
  for(int i=0; i<dynamixels.size(); i++) {
    dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, dynamixels[i].id, time);
  }
}

void runSpeedTestsAt(int bpsIndex) {
  int runBps = bpsList[bpsIndex];
  int switched = 0;
  Serial.println(String("Running speed test at ") + runBps + "bps (current: " + bps + ").");
  Serial.println("Type 'yes' to continue.");
  while(true) {
    String str = Serial.readString();
    if(str == "") continue;
    if(str.startsWith("yes")) break;
    else {
      Serial.println("Cowardly refusing to continue. Endless loop!");
      while(true);
    }
  }

  if(bps != runBps) {
    Serial.println(String("Switching all servos to ") + runBps + "bps.");
    for(int i=0; i<dynamixels.size(); i++) {
      dxl.torqueOff(dynamixels[i].id);
    }

    for(int i=0; i<dynamixels.size(); i++) {
      if(dxl.setBaudrate(dynamixels[i].id, runBps) == false) {
        Serial.println(String("Couldn't set bps on ") + dynamixels[i].id + " to " + runBps + " (error code " + dxl.getLastLibErrCode() + ").");
        //while(true);
      } else {
        switched++;
      }
    }

    if(switched != 0) {
      Serial.println(String("Switched ") + switched + " servos.");
      dxl.begin(runBps);
      if(dxl.scan() == false) {
        Serial.println("Couldn't rescan after BPS change.");
        return;
      }
      bps = runBps;
    } else {
      Serial.println("Didnt't switch even one.");
      return;
    }
  }

  int statusReturnLevel, returnDelayTime;
  unsigned long usec;
  
  statusReturnLevel = 2; // default
  returnDelayTime = 250; // default

  Serial.println(String("Status return level: ") + statusReturnLevel + " Return delay time: " + returnDelayTime);
  setStatusReturnLevel(statusReturnLevel); 
  setReturnDelayTime(returnDelayTime);

  usec = micros();
  Serial.print(String("Running sync get/set position (") + reps + "reps)... ");
  for(int i=0; i<reps; i++) {
    runSyncGetSetPosition(2048, 2048, 2048, 2048);
  }
  Serial.println(String(micros()-usec) + " microseconds elapsed.");

  statusReturnLevel = 2;
  returnDelayTime = 10;
  
  Serial.println(String("Status return level: ") + statusReturnLevel + " Return delay time: " + returnDelayTime);
  setStatusReturnLevel(statusReturnLevel); 
  setReturnDelayTime(returnDelayTime);

  usec = micros();
  Serial.print(String("Running sync get/set position (") + reps + "reps)... ");
  for(int i=0; i<reps; i++) {
    runSyncGetSetPosition(2048, 2048, 2048, 2048);
  }
  Serial.println(String(micros()-usec) + " microseconds elapsed.");

  statusReturnLevel = 2;
  returnDelayTime = 250;
  
  Serial.println(String("Status return level: ") + statusReturnLevel + " Return delay time: " + returnDelayTime);
  setStatusReturnLevel(statusReturnLevel); 
  setReturnDelayTime(returnDelayTime);

  usec = micros();
  Serial.print(String("Running basic set/get position (") + reps + "reps)... ");
  for(int i=0; i<reps; i++) {
    runBasicGetSetPosition();
  }
  Serial.println(String(micros()-usec) + " microseconds elapsed.");

  statusReturnLevel = 2;
  returnDelayTime = 10;
  
  Serial.println(String("Status return level: ") + statusReturnLevel + " Return delay time: " + returnDelayTime);
  setStatusReturnLevel(statusReturnLevel); 
  setReturnDelayTime(returnDelayTime);

  usec = micros();
  Serial.print(String("Running basic set/get position (") + reps + "reps)... ");
  for(int i=0; i<reps; i++) {
    runBasicGetSetPosition();
  }
  Serial.println(String(micros()-usec) + " microseconds elapsed.");
}
 

void setup() {
  Serial.begin(2000000);
  while(!Serial);

  if(detectDynamixels() == false) { 
    Serial.println("No Dynamixels found. Halting.");
    while(true);
  }

  Serial.println(String("Found ") + dynamixels.size() + " Dynamixels at " + bps + "bps.");
  for(int i=0; i<dynamixels.size(); i++) {
    dxl.torqueOn(dynamixels[i].id);
    dxl.setGoalPosition(dynamixels[i].id, 180.0, UNIT_DEGREE);
  }
  setupSyncBuffers();
  delay(1000);

  for(int i=0; i<numBps; i++) {
    Serial.println("Set status return level to 2 (default)");
    runSpeedTestsAt(i);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
