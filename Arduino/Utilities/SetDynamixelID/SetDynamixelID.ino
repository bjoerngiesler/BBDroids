// USAGE:
// 
// All Dynamixels start with ID 1, 57600bps. This program can be used to change
// IDs interactively. Please make sure that no two servos with the same ID are
// connected at once!
//
// For caution and testing, the define SIMULATION_ONLY disables actual changes
// to be written. Comment it out to go to live mode.

#include <Dynamixel2Arduino.h>

//#define SIMULATION_ONLY
#define DXL_BAUDRATE 57600
#define DXL_PROTOCOL_VERSION 2.0

#define MAXID 253
Dynamixel2Arduino dxl(Serial1, 21);
bool dxlIDsFound[MAXID+1];

void setup() {
  Serial.begin(115200);
  while(!Serial);

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println(); Serial.println();
  Serial.println("Servo ID Changer");
  Serial.println("================");
#if defined(SIMULATION_ONLY)
  Serial.println("SIMULATION ONLY - NO CHANGES WILL BE MADE");
#else
  Serial.println("LIVE MODE - CHANGES WILL BE WRITTEN TO SERVOS. BE CAREFUL!");
#endif

  while(true) {
    int numFound = 0;
    Serial.println("Scanning the bus for Dynamixels...");
    for(int i=0; i<MAXID; i++) {
      dxlIDsFound[i] = dxl.ping(i);
      if(dxlIDsFound[i]) numFound++;
    }

    if(numFound == 0) {
      Serial.println("No Dynamixels found on bus! Please check cables, baud rate and protocol version settings!");
      while(true);
    }
    
    Serial.println("Dynamixel IDs found on bus:");
    Serial.println("ID\tModel");
    for(int i=0; i<MAXID; i++) { 
      if(dxlIDsFound[i]) {
        Serial.print(i); 
        Serial.print("\t");
        Serial.println(dxl.getModelNumber(i));
      }
    }
    
    Serial.println("Which ID would you like to change?");
    while(!Serial.available());
    int fromId = Serial.readString().toInt();
    
    if(fromId<0) {
      Serial.println("ID must be greater than zero.");
      continue;
    } else if(fromId>MAXID) {
      Serial.print("ID must be smaller than "); Serial.println(MAXID);
      continue;
    } else if(!dxlIDsFound[fromId]) {
      Serial.print("ID "); Serial.print(fromId); Serial.println(" not found on the bus.");
      continue;
    }
  
    Serial.print("What ID would you like the servo with current id ");
    Serial.print(fromId);
    Serial.println(" to have?");
    while(!Serial.available());
    int toId = Serial.readString().toInt();

    if(toId<0) {
      Serial.println("ID must be greater than zero.");
      continue;
    } else if(toId>MAXID) {
      Serial.print("ID must be smaller than "); Serial.println(MAXID);
      continue;
    } else if(toId == fromId) {
      Serial.print("Not changing "); Serial.print(fromId); Serial.print(" to "); Serial.print(toId);
      Serial.println(" because they are the same.");
      continue;
    }

    Serial.print("Changing ID "); Serial.print(fromId); Serial.print(" to "); Serial.print(toId);
    Serial.println(". Type \"yes\" if you are sure.");
    while(!Serial.available());
    if(!Serial.readString().startsWith("yes")) {
      Serial.println("Aborting.");
      continue;
    }

#if defined(SIMULATION_ONLY)
    Serial.println("SIMULATION ONLY - NOTHING CHANGED");
#else
    if(dxl.setID(fromId, toId)) {
      Serial.println("Successfully changed.");
      dxl.torqueOn(toId);
      dxl.setGoalPosition(toId, 0.0, UNIT_DEGREE);
      delay(1000);
      dxl.setGoalPosition(toId, 360.0, UNIT_DEGREE);
      delay(1000);
      dxl.setGoalPosition(toId, 180.0, UNIT_DEGREE);
    } else {
      Serial.println("ERROR!");
    }
#endif
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
