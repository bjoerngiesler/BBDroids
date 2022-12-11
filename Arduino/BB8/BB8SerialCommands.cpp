#include "BB8SerialCommands.h"
#include "BB8WifiServer.h"
#include "BB8Servos.h"
#include "BB8ConfigStorage.h"
#include "BB8Packet.h"
#include "BB8Sound.h"
#include "BB8Controllers.h"
#include "BB8Config.h"
#include "BB8IMU.h"

//#define SERIALCOMMANDS_DEBUG

BB8SerialCommands BB8SerialCommands::sercmd;

bool BB8SerialCommands::begin() {
  return true;
}

bool BB8SerialCommands::handleIfAvailable() {
  if(!Serial.available()) return false;

  String str = Serial.readStringUntil('\n');
  str.trim();
  std::vector<String> words = split(str);

  if(words.size() == 0) return false;

  CommandReturn retval;
  if(words[0] == "help") retval = handleHelpCommand(words);
  else if(words[0] == "wifi") retval = handleWifiCommand(words);
  else if(words[0] == "servos") retval = handleServoCommand(words);
  else if(words[0] == "sound") retval = handleSoundCommand(words);
  else if(words[0] == "set") retval = handleSetVariableCommand(words);
  else if(words[0] == "show") retval = handleShowVariablesCommand(words);
  else if(words[0] == "control") retval = handleControlCommand(words);
  else if(words[0] == "plot") retval = handlePlotCommand(words);
  else if(words[0] == "calib") retval = handleCalibCommand(words);
  else if(words[0] == "store" && words.size() == 1) retval = BB8ConfigStorage::storage.storeFlash() ? PRINT_OK : PRINT_FAILURE;
  else {
    Serial.print("Unknown command '"); Serial.print(words[0]); Serial.println("'");
    return false;
  }

  switch(retval) {
  case OK: return true;
  case FAILURE: return false;
  case PRINT_OK:
    Serial.println("Ok.");
    return true;
  case PRINT_FAILURE:
    Serial.println("Failure.");
    return false;
  }
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleHelpCommand(const std::vector<String>& words) {
  if(words.size() != 1) {
    Serial.println("The 'help' command does not take any arguments.");
    return FAILURE;
  }

  Serial.println("Commands:");
  Serial.println("=========");
  Serial.println("wifi (status|startup|shutdown)        : Handle Wifi.");
  Serial.println("servos (status|startupshutdown|origin): Handle servos, or move all to origin.");
  Serial.println("sound play NUM                        : Play file numbered NUM.");
  Serial.println("sound volume NUM                      : Set volume to NUM.");
  Serial.println("control roll (true|false|FLOAT)       : Activate or deactivate roll controller, or set setpoint to FLOAT.");
  Serial.println("plot (none|roll_ctrl|body_imu)        : Output data for the given subsystem as input for the Arduino signal plotter.");
  Serial.println("calib                                 : Calibrate IMUs. Make sure droid is at COMPLETE REST before you do this!");
  Serial.println("show [setprefix]                      : Show values of all variables. If 'setprefix' is specified, each line is prefixed with 'set'");
  Serial.println("store                                 : Commit variables to flash.");
  Serial.println("set VAR VALUE                         : Set variable VAR to VALUE (use 'store' to commit to flash). Valid variables are:");
  Serial.println("        ap (true|false)     : Run Wifi in access point mode.");
  Serial.println("        ssid STRING         : Use the given SSID (enclose in \"\" if it contains spaces).");
  Serial.println("        key STRING          : Use the given WPA key (enclose in \"\" if it contains spaces).");
  Serial.println("        roll_kp FLOAT       : Kp constant for the roll controller.");
  Serial.println("        roll_ki FLOAT       : Ki constant for the roll controller.");
  Serial.println("        roll_kd FLOAT       : Kd constant for the roll controller.");
  Serial.println("        roll_imax FLOAT     : Max integral part the roll controller will use (for protection; Kp*e + Ki*max(i, imax)) + Kd*d.");
  Serial.println("        roll_iabort FLOAT   : The roll controller stops control if i grows beyond this value (for protection).");
  Serial.println("        roll_deadband FLOAT : If the absolute control input to the roll controller is below this deadband, no control is issued.");

  return OK;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleWifiCommand(const std::vector<String>& words) {
  if(words.size() != 2 || (words[1] != "status" && words[1] != "startup" && words[1] != "shutdown")) {
    Serial.println("Usage: 'wifi status|startup|shutdown'");
    return FAILURE;
  }
  
  if(words[1] == "status") {
    BB8WifiServer::server.printStatus();
    return OK;
  } else if(words[1] == "startup") {
    if(BB8WifiServer::server.isUDPServerStarted()) {
      Serial.println("Wifi and UDP server already running.");
      return FAILURE;
    } else {
      if(BB8WifiServer::server.begin()) return PRINT_OK;
      else return PRINT_FAILURE;
    }
  } else if(words[1] == "shutdown") {
    if(BB8WifiServer::server.isUDPServerStarted() || 
       BB8WifiServer::server.isConnected() || 
       BB8WifiServer::server.isAPStarted()) {
      BB8WifiServer::server.shutdown();
      return PRINT_OK;
    } else {
      Serial.println("UDP server not running and Wifi not connected.");
      return FAILURE;
    }
  }

  Serial.println("Should never get here.");
  return FAILURE;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handlePlotCommand(const std::vector<String>& words) {
  if(words.size() != 2 || (words[1] != "none" && words[1] != "roll_ctrl" && words[1] != "body_imu")) {
    Serial.println("Usage: 'plot (none|roll_ctrl|body_imu)'");
    return FAILURE;
  }
  
  if(words[1] == "none") { 
    plotMode = PLOT_NONE; 
    return PRINT_OK; 
  } else if(words[1] == "roll_ctrl") {
    if(BB8PIDController::rollController.available()) {
      plotMode = PLOT_ROLL_CONTROLLER; 
      return PRINT_OK; 
    } else {
      Serial.println("Roll controller not available.");
      plotMode = PLOT_NONE;
      return FAILURE;
    }
  } else if(words[1] == "body_imu") { 
    if(BB8BodyIMU::imu.available()) {
      plotMode = PLOT_BODY_IMU; 
      return PRINT_OK; 
    } else {
      Serial.println("Body IMU not available.");
      plotMode = PLOT_NONE;
      return FAILURE;
    }
  }

  Serial.println("Should never get here.");
  return FAILURE;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleCalibCommand(const std::vector<String>& words) {
  if(words.size() != 1) {
    Serial.println("Usage: 'calib' (no parameters)");
    return FAILURE;
  }
  if(BB8BodyIMU::imu.available() == false) {
    Serial.println("Body IMU not available.");
    return FAILURE;
  }
  
  if(BB8BodyIMU::imu.calibrateGyro() == true) return PRINT_OK;
  else return PRINT_FAILURE;

  Serial.println("Should never get here.");
  return FAILURE;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleServoCommand(const std::vector<String>& words) {
  if(words.size() != 2 || (words[1] != "status" && words[1] != "startup" && words[1] != "shutdown" && words[1] != "origin")) {
    Serial.println("Usage: 'servos status|shutdown|startup|origin'");
    return FAILURE;
  }

  if(words[1] == "status") {
    BB8Servos::servos.printStatus();
    return OK;
  } else if(words[1] == "startup") {
    if(BB8Servos::servos.begin() == true) {
      state.servosOK = true;
      return PRINT_OK;
    } else {
      state.servosOK = false;
      return PRINT_FAILURE;
    }
  } else if(words[1] == "origin") {
    if(BB8Servos::servos.moveAllServosToOrigin() == true) return PRINT_OK;
    else return PRINT_FAILURE;
  }

  Serial.println("Should never get here.");
  return FAILURE;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleSoundCommand(const std::vector<String>& words) {
  if(words.size() != 3 || (words[1] != "play" && words[1] != "volume")) {
    Serial.println("Usage: 'sound play NUM|volume NUM'");
    return FAILURE;
  }

  if(words[1] == "play") {
    if(BB8Sound::sound.play(atoi(words[2].c_str())) == true) return PRINT_OK;
    else return PRINT_FAILURE;
  } else if(words[1] == "volume") {
    if(BB8Sound::sound.setVolume(atoi(words[2].c_str()))) return PRINT_OK;
    else return PRINT_FAILURE;
  }

  Serial.println("Should never get here.");
  return FAILURE;
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleSetVariableCommand(const std::vector<String>& words) {
  bool ap;
  String ssid, key;
  float roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband;
  
  BB8ConfigStorage::storage.getWifiParams(ap, ssid, key);
  BB8ConfigStorage::storage.getControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);

  if(words.size() != 3) {
    Serial.println("Usage: set ap true|false | set ssid SSIDNAME | set key WIFIKEY");
    return FAILURE;
  }
  if(words[1] == "ap") {
    if(words[2] == "true" || words[2] == "false") {
      BB8ConfigStorage::storage.setWifiParams(words[2] == "true" ? true : false, ssid, key);
      return PRINT_OK;
    } else {
      Serial.println("Syntax error - use 'set ap true' or 'set ap false'.");
      return FAILURE;
    }
  } else if(words[1] == "ssid") {
    BB8ConfigStorage::storage.setWifiParams(ap, words[2], key);
    return PRINT_OK;
  } else if(words[1] == "key") {
    BB8ConfigStorage::storage.setWifiParams(ap, ssid, words[2]);
    return PRINT_OK;
  } else if(words[1] == "roll_kp") {
    if(!safeStringToFloat(words[2], roll_kp, true)) return FAILURE;
    if(roll_kp < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else if(words[1] == "roll_ki") {
    if(!safeStringToFloat(words[2], roll_ki, true)) return FAILURE;
    if(roll_ki < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else if(words[1] == "roll_kd") {
    if(!safeStringToFloat(words[2], roll_kd, true)) return FAILURE;
    if(roll_kd < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else if(words[1] == "roll_imax") {
    if(!safeStringToFloat(words[2], roll_imax, true)) return FAILURE;
    if(roll_imax < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else if(words[1] == "roll_iabort") {
    if(!safeStringToFloat(words[2], roll_iabort, true)) return FAILURE;
    if(roll_iabort < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else if(words[1] == "roll_deadband") {
    if(!safeStringToFloat(words[2], roll_deadband, true)) return FAILURE;
    if(roll_deadband < 0) { Serial.println("Negative values do not make sense."); return FAILURE; }
    BB8ConfigStorage::storage.setControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
    return PRINT_OK;
  } else {
    Serial.print("Unknown variable \""); Serial.print(words[1]); Serial.println("\". Use \"help\" or \"show\" for a list of variables.");
    return FAILURE;
  }
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleShowVariablesCommand(const std::vector<String>& words) {
  bool setprefix = false;

  if(words.size() == 2) {
    if(words[1] == "setprefix") setprefix = true;
    else {
      Serial.println("Usage: show [setprefix]");
      return FAILURE;
    }
  } else if(words.size() > 2) {
    Serial.println("Usage: show [setprefix]");
    return FAILURE;
  }

  bool ap;
  String ssid, key;
  BB8ConfigStorage::storage.getWifiParams(ap, ssid, key);  
  if(ap) Serial.println(String(setprefix?"set ":"") + "ap true");
  else Serial.println(String(setprefix?"set ":"") + "ap false");
  Serial.print(String(setprefix?"set ":"") + "ssid \""); Serial.print(ssid); Serial.println("\"");
  Serial.print(String(setprefix?"set ":"") + "key \""); Serial.print(key); Serial.println("\"");

  float roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband;
  BB8ConfigStorage::storage.getControlParams(BB8ConfigStorage::ROLL_CONTROLLER, roll_kp, roll_ki, roll_kd, roll_imax, roll_iabort, roll_deadband);
  Serial.print(String(setprefix?"set ":"") + "roll_kp "); Serial.println(roll_kp);
  Serial.print(String(setprefix?"set ":"") + "roll_ki "); Serial.println(roll_ki);
  Serial.print(String(setprefix?"set ":"") + "roll_kd "); Serial.println(roll_kd);
  Serial.print(String(setprefix?"set ":"") + "roll_imax "); Serial.println(roll_imax);
  Serial.print(String(setprefix?"set ":"") + "roll_iabort "); Serial.println(roll_iabort);
  Serial.print(String(setprefix?"set ":"") + "roll_deadband "); Serial.println(roll_deadband);
}

BB8SerialCommands::CommandReturn BB8SerialCommands::handleControlCommand(const std::vector<String>& words) {
  float setpoint;
  if(words.size() != 3 || (words[2] != "true" && words[2] != "false" && !safeStringToFloat(words[2], setpoint))) {
    Serial.println("Usage: 'control roll (true|false|FLOAT)'");
    return FAILURE;
  }

  if(words[1] == "roll") {
    if(words[2] == "true") {
      if(BB8PIDController::rollController.enable(true)) return PRINT_OK;
      else return PRINT_FAILURE;
    } else if(words[2] == "false") {
      if(BB8PIDController::rollController.enable(false)) return PRINT_OK;
      else return PRINT_FAILURE;
    } else if(safeStringToFloat(words[2], setpoint)) {
      if(BB8PIDController::rollController.isEnabled()) {
        BB8PIDController::rollController.setSetpoint(setpoint);
      } else {
        Serial.println("Roll controller is not enabled.");
        return FAILURE;
      }
    } else {
      Serial.println("Usage: 'control roll (true|false|FLOAT)'");
      return FAILURE;
    }
  }

  Serial.println("Should never get here.");
  return FAILURE;
}

std::vector<String> BB8SerialCommands::split(const String& str) {
  std::vector<String> words;
  int start = 0, end = 0;
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

#ifdef SERIALCOMMANDS_DEBUG
  Serial.print("Split: ");
  for(int i=0; i<words.size(); i++) {
    Serial.print("'"); Serial.print(words[i]); Serial.print("' ");
  }
  Serial.println();
#endif

  return words;
}

bool BB8SerialCommands::safeStringToFloat(const String& str, float& f, bool printResult) {
  if(str.length() == 0) { 
    if(printResult) Serial.println("Expected float, got string of length 0.");
    return false;
  }

  if(str == "-" || str == "." || str == "-." || str == ".-") {
    if(printResult) {
      Serial.print("Expected float, got \""); Serial.print(str); Serial.println("\".");
    }
    return false;
  }
  
  char c; bool point = false;
  
  c = str.charAt(0);
  if(!isdigit(c) && c != '-' && c != '.') {
    if(printResult) {
      Serial.print("Expected float, got \""); Serial.print(str); Serial.println("\" (starts with invalid character).");
    }
    return false;
  }

  if(c == '.') {
    point = true;
  }

  for(int i=1; i<str.length(); i++) {
    c = str.charAt(i);
    if(isdigit(c)) continue;
    else if(c == '.') {
      if(point == true) {
        if(printResult) {
          Serial.print("Expected float, got \""); Serial.print(str); Serial.println("\" (multiple decimal points).");
        }
        return false; // already have a decimal point
      } else point = true;
    } else {
      if(printResult) {
        Serial.print("Expected float, got \""); Serial.print(str); Serial.println("\" (contains invalid character).");
      }
      return false;
    }
  }

  f = str.toFloat();
  return true;
}