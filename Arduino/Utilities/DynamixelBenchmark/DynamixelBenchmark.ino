#include <DynamixelShield.h>
#include <vector>

DynamixelShield dxl;

const int bpsList[] = { 9600, 57600, 115200, 1000000, 2000000 }; // Arduino can't go higher than 2Mbps
const int numBps = 5;
const int reps = 1000;

typedef struct {
  int id;
  int modelNumber;
} Dynamixel;

int bps;
std::vector<Dynamixel> dynamixels;

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

void runSpeedTestsAt(int bpsIndex) {
  int runBps = bpsList[bpsIndex];
  int switched = 0;
  Serial.println(String("Running speed test at ") + runBps + "bps.");
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

    } else {
      Serial.println("Didnt't switch even one.");
      return;
    }
  }

  unsigned long usec = micros();
  Serial.print(String("Running basic set/get position (") + reps + "reps)... ");
  for(int i=0; i<reps; i++) {
    runBasicGetSetPosition();
  }
  Serial.println(String(micros()-usec) + "microseconds elapsed.");
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
  delay(1000);

  for(int i=0; i<numBps; i++) {
    runSpeedTestsAt(i);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
