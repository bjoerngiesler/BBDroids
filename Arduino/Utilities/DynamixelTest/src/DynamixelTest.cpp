// USAGE:
// 
// All Dynamixels start with ID 1, 57600bps. This program can be used to change
// IDs interactively. Please make sure that no two servos with the same ID are
// connected at once!
//
// For caution and testing, the define SIMULATION_ONLY disables actual changes
// to be written. Comment it out to go to live mode.

#include <Dynamixel2Arduino.h>
#include <vector>

#define DXL_BAUDRATE 57600
#define DXL_PROTOCOL_VERSION 2.0

#define MAXID 253
Dynamixel2Arduino dxl;
bool dxlIDsFound[MAXID+1];

std::vector<String> split(const String& str);

bool print_help();

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println(); Serial.println();
  Serial.println("Dynamixel Test Tool");
  Serial.println("===================");

  print_help();
}

bool setID(uint8_t fromId, uint8_t toId) {
  if(fromId<0) {
    Serial.println("ID must be greater than zero.");
    return false;
  } else if(fromId>MAXID) {
    Serial.print("ID must be smaller than "); Serial.println(MAXID);
    return false;
  } else if(!dxlIDsFound[fromId]) {
    Serial.print("ID "); Serial.print(fromId); Serial.println(" not found on the bus.");
    return false;
  }

  if(toId<0) {
    Serial.println("ID must be greater than zero.");
    return false;
  } else if(toId>MAXID) {
    Serial.print("ID must be smaller than "); Serial.println(MAXID);
    return false;
  } else if(toId == fromId) {
    Serial.print("Not changing "); Serial.print(fromId); Serial.print(" to "); Serial.print(toId);
    Serial.println(" because they are the same.");
    return false;
  }

  Serial.print("Changing ID "); Serial.print(fromId); Serial.print(" to "); Serial.print(toId);

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

bool init(unsigned int baud) {
    int numFound = 0;

  if(baud != 57600 && baud != 115200 && baud != 1000000) {
    Serial.println("Unsupported baud rate!");
    return false;
  }

  Serial.println(String("Setting the bus to ") + baud + ".");
  dxl.begin(baud);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.print("Scanning the bus for Dynamixels... ");
  for(int i=0; i<MAXID; i++) dxlIDsFound[i] = false;

  if(dxl.scan() == false) {
    Serial.println("failure.");
    return false;
  } else Serial.println("success.");
  
  for(int i=0; i<MAXID; i++) {
    dxlIDsFound[i] = dxl.ping(i);
    if(dxlIDsFound[i]) numFound++;
  }

  if(numFound == 0) {
    Serial.println("No Dynamixels found on bus. Please try a different baud rate!");
    return false;
  }
    
  Serial.println("Dynamixel IDs found on bus:");
  Serial.println("ID\tModel\tError status");
  for(int i=0; i<MAXID; i++) { 
    if(dxlIDsFound[i]) {
      Serial.print(i); 
      Serial.print("\t");
      Serial.print(dxl.getModelNumber(i));
      Serial.print("\t0x");
      Serial.println(dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS,i), HEX);
    }
  }

  return true;
}

bool info(uint8_t id) {
  if(id == 0 || id >= MAXID) {
    Serial.println(String("ID ") + id + " out of range!");
    return false;
  }

  if(dxlIDsFound[id] == false) {
    Serial.println(String("ID ") + id + " not found on bus.");
    return false;
  }
  Serial.print(String("ID ") + id + " -- ");
  Serial.print(String("angle: ") + dxl.getPresentPosition(id, UNIT_DEGREE));
  Serial.print(String(", goal: ") + dxl.readControlTableItem(ControlTableItem::GOAL_POSITION, id));
  Serial.print(String(", torque: ") + dxl.getTorqueEnableStat(id));
  Serial.print(String(", error: 0x") + dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id));
  Serial.println();
  return true;
}

bool move(uint8_t id, float angle) {
  if(id == 0 || id >= MAXID) {
    Serial.println(String("ID ") + id + " out of range!");
    return false;
  }
  if(dxlIDsFound[id] == false) {
    Serial.println(String("ID ") + id + " not found on bus.");
    return false;
  }
  dxl.torqueOn(id);
  return dxl.setGoalPosition(id, angle, UNIT_DEGREE);
}

bool set_id(uint8_t oldid, uint8_t newid) {
  if(oldid == 0 || oldid >= MAXID) {
    Serial.println(String("Old ID ") + oldid + " out of range!");
    return false;
  }
  if(newid == 0 || newid >= MAXID) {
    Serial.println(String("NewID ") + newid + " out of range!");
    return false;
  }
  if(dxlIDsFound[oldid] == false) {
    Serial.println(String("ID ") + oldid + " not found on bus.");
    return false;
  }

  dxl.torqueOff(oldid);

  if(dxl.setID(oldid, newid) == false) {
    Serial.println(String("Failed to change ID from ") + oldid + " to " + newid + "!");
    return false;
  }
  dxlIDsFound[oldid] = false;
  dxlIDsFound[newid] = true;
  Serial.println(String("Successfully changed ID from ") + oldid + " to " + newid + "!");
  return true;
}

bool set_baud(uint8_t id, unsigned int baud) {
  if(id == 0 || id >= MAXID) {
    Serial.println(String("ID ") + id + " out of range!");
    return false;
  }

  if(dxlIDsFound[id] == false) {
    Serial.println(String("ID ") + id + " not found on bus.");
    return false;
  }

  typedef enum {
    BAUD_57600   = 1,
    BAUD_115200  = 2,
    BAUD_1000000 = 3
  } DynamixelBaud;

  DynamixelBaud b;
  switch(baud) {
  case 57600:
    b = BAUD_57600;
    break;
  case 115200:
    b = BAUD_115200;
    break;
  case 1000000:
    b = BAUD_1000000;
    break;
  default:
    Serial.println(String("Invalid baud rate ") + baud + "!");
    return false;
  }

  dxl.torqueOff(id);
  if(dxl.writeControlTableItem(ControlTableItem::BAUD_RATE, id, (int32_t)b) == false) {
    Serial.println("Failure.");
    return false;
  } else {
    Serial.println("Success. You want to init with the new baud rate to see the servo.");
  }

  return true;
}

bool print_help() {
  Serial.println("Command options:");
  Serial.println("    help                  Print this help message");
  Serial.println("    init BAUDRATE         Initialize and scan the bus. Supported baud rates: 57600, 115200, 1000000.");
  Serial.println("    move ID ANGLE         Move servo ID to given angle. Switches on torque if necessary. No checks.");
  Serial.println("    set_id ID NEWID       Change servo ID to NEWID.");
  Serial.println("    set_baud ID BAUDRATE  Set ID's baud rate. Supported baud rates: 57600, 115200, 1000000. Do an init afterwards.");
  Serial.print("> ");
}

void loop() {
  String command;
  do {
    command = Serial.readString();
  } while(command == "");

  Serial.print(command);

  std::vector<String> words = split(command);

  if(words.size() == 0) return;

  if(words[0] == "help") {
    if(words.size() != 1) { 
      Serial.println("Wrong number of arguments!");
      return;
    }
    print_help();
    return;
  }

  if(words[0] == "init") {
    if(words.size() != 2) { 
      Serial.println("Wrong number of arguments!");
      return;
    }
    if(init(words[1].toInt()) == false) {
      Serial.println("Failure.");
    } else {
      Serial.println("Success.");
    }
    return;
  }

  if(words[0] == "info") {
    if(words.size() != 2) {
      Serial.println("Wrong number of arguments!");
      return;
    }
    if(info(words[1].toInt()) == false) {
      Serial.println("Failure.");
    } else {
      Serial.println("Success.");
    }
    return;
  }

  if(words[0] == "move") {
    if(words.size() != 3) { 
      Serial.println("Wrong number of arguments!");
      return;
    }
    if(move(words[1].toInt(), words[2].toFloat()) == false) {
      Serial.println("Failure.");
    } else {
      Serial.println("Success.");
    }
    return;
  }

  if(words[0] == "set_id") {
    if(words.size() != 3) { 
      Serial.println("Wrong number of arguments!");
      return;
    }
    if(set_id(words[1].toInt(), words[2].toInt()) == false) {
      Serial.println("Failure.");
    } else {
      Serial.println("Success.");
    }
    return;
  }
  
  if(words[0] == "set_baud") {
    if(words.size() != 3) { 
      Serial.println("Wrong number of arguments!");
      return;
    }
    if(set_baud(words[1].toInt(), words[2].toInt()) == false) {
      Serial.println("Failure.");
    } else {
      Serial.println("Success.");
    }
    return;
  }

  Serial.println("Unknown command.");
}

std::vector<String> split(const String& str) {
  std::vector<String> words;
  unsigned int start = 0, end = 0;
  bool quotes = false;

  while(end < str.length()) {
    if(quotes == true) {
      if(str[end] != '"') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        quotes = false;
        String substr = str.substring(start, end);
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      }
    } else {
      if(str[end] == '"') {
        quotes = true;
        start++;
        end++;
        continue;
      }
  
      if(str[end] != ' ') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        String substr = str.substring(start, end);
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      } 
    }
  }

  return words;
}
