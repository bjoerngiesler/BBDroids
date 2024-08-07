#include "RDSerialInterface.h"
#include "GFX4dIoD9.h"

#include <algorithm>

extern GFX4dIoD9 gfx;

std::map<rd::Command, rd::CmdArgDescription> rd::SerialInterface::cmdArgs = {
  { CMD_LOGO, { 1, "0 => no backlight, 1 => backlight" } },
  { CMD_CLS, { 0, "" } },
  { CMD_POINT, { 3, "x,y,color" } },
  { CMD_LINE, { 5, "x1,y1,x2,y2,color" } },
  { CMD_HLINE, { 4, "x,y,width,color" } },
  { CMD_VLINE, { 4, "x,y,height,color" } },
  { CMD_RECT, { 5, "x1,y1,x2,y2,color" } },
  { CMD_FILLEDRECT, { 5, "x1,y1,x2,y2,color" } },
  { CMD_CIRCLE, { 4, "x,y,radius,color" } },
  { CMD_FILLEDCIRCLE, { 4, "x,y,radius,color" } },
  { CMD_TEXT, { 4, "x,y,color,string (ascii: enclosed in "
                   ", binary: terminated by \0)" } }
};


rd::SerialInterface rd::SerialInterface::serial;

rd::SerialInterface::SerialInterface() {
}

bool rd::SerialInterface::begin() {
  Serial.begin(921600);
  return true;
}

bool rd::SerialInterface::available() {
  return Serial.available();
}

uint16_t rd::SerialInterface::lookupColor(uint8_t color) {
  return gfx.RGBto565((color & 0x30) << 2, (color & 0xC) << 4, (color & 0x03) << 6);
}

rd::SerialInterface::Result rd::SerialInterface::execText(uint8_t x, uint8_t y, uint8_t color, const String& text) {
  gfx.MoveTo(x, y);
  gfx.TextColor(lookupColor(color));
  gfx.print(text);
  return RESULT_OK;
}

rd::SerialInterface::Result rd::SerialInterface::readArgsBinary(uint8_t num, std::vector<uint8_t>& args) {
  args.clear();
  int timeout = 10;
  while (timeout > 0) {
    if (Serial.available()) {
      args.push_back(Serial.read());
      if (args.size() == num) return RESULT_OK;
      timeout = 10;
    } else {
      delay(1);
      timeout--;
    }
  }
  return RESULT_ERROR;
}

rd::SerialInterface::Result rd::SerialInterface::readStringUntil(uint8_t delim, String& str) {
  int timeout = 10;
  while (timeout > 0) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == delim) return RESULT_OK;
      str += c;
      timeout = 10;
    } else {
      delay(1);
      timeout--;
    }
  }
  return RESULT_ERROR;
}

rd::SerialInterface::Result rd::SerialInterface::execInput(Command cmd, std::vector<uint8_t> args) {
  // Double sanity check input; this has been checked before calling, but just to make sure.
  if (cmdArgs.find(cmd) == cmdArgs.end()) return RESULT_ERROR;
  uint8_t numArgs = cmdArgs[cmd].numArgs;
  if (args.size() != numArgs) return RESULT_ERROR;

  switch (cmd) {
    case CMD_LOGO:
      gfx.BacklightOn(args[0]);
      return RESULT_RUN_LOGO_SCREEN;

    case CMD_CLS:
      gfx.Cls();
      return RESULT_OK;

    case CMD_POINT:
      gfx.PutPixel(args[0], args[1], lookupColor(args[2]));
      return RESULT_OK;

    case CMD_HLINE:
      gfx.Hline(args[0], args[1], args[2], lookupColor(args[3]));
      return RESULT_OK;

    case CMD_VLINE:
      gfx.Vline(args[0], args[1], args[2], lookupColor(args[3]));
      return RESULT_OK;

    case CMD_LINE:
      Serial.println(String("Line (")+args[0]+","+args[1]+")->("+args[2]+","+args[3]+"), color " + String(lookupColor(args[4]), HEX));
      gfx.Line(args[0], args[1], args[2], args[3], lookupColor(args[4]));
      return RESULT_OK;

    case CMD_CIRCLE:
      gfx.Circle(args[0], args[1], args[2], lookupColor(args[3]));
      return RESULT_OK;

    case CMD_FILLEDCIRCLE:
      gfx.CircleFilled(args[0], args[1], args[2], lookupColor(args[3]));
      return RESULT_OK;

    case CMD_RECT:
      gfx.Rectangle(args[0], args[1], args[2], args[3], lookupColor(args[4]));
      return RESULT_OK;

    case CMD_FILLEDRECT:
      gfx.RectangleFilled(args[0], args[1], args[2], args[3], lookupColor(args[4]));
      return RESULT_OK;

    default:
      break;
  }

  return RESULT_ERROR;
}

rd::SerialInterface::Result rd::SerialInterface::handleInput() {
  if (!Serial.available()) {
    return RESULT_NOTHING_TO_READ;
  }

  uint8_t cmdByte = (Command)Serial.read();
  Command cmd;
  std::vector<uint8_t> args;
  bool binary = false;

  if (cmdByte & 0x80) {  // binary; strip high bit and continue
    cmdByte &= 0x7f;
    binary = true;
  }

  cmd = (Command)cmdByte;

  // NOPs get a direct "OK" reply
  if (cmd == CMD_NOP) {
    if (binary) Serial.write(CMD_NOP | 0x80);
    else Serial.println("OK.");
    return RESULT_OK;
  }

  // Lookup number of arguments and help message
  if (cmdArgs.find(cmd) == cmdArgs.end()) {
    if (binary) Serial.write(CMD_ERROR | 0x80);
    else Serial.println(String("Unknown command \'") + String(cmdByte) + "\'.");
    return RESULT_ERROR;
  }

  uint8_t numArgs = cmdArgs[cmd].numArgs;
  const char* helpMessage = cmdArgs[cmd].helpMessage;

  if (binary) {
    // Text command handled separately because of the variable argument list
    if (cmd == CMD_TEXT) {
      if (readArgsBinary(3, args) != RESULT_OK) {
        Serial.write(CMD_ERROR | 0x80);
        return RESULT_ERROR;
      }

      String text;
      if (readStringUntil('\0', text) != RESULT_OK) {
        Serial.write(CMD_ERROR | 0x80);
        return RESULT_ERROR;
      }

      if(execText(args[0], args[1], args[2], text) != RESULT_OK) {
        Serial.write(CMD_ERROR | 0x80);
        return RESULT_ERROR;
      }

      Serial.write(CMD_NOP | 0x80);
      return RESULT_OK;
    }

    // Still here? Read arguments and continue
    if(readArgsBinary(numArgs, args) != RESULT_OK) {
      Serial.write(CMD_ERROR | 0x80);
      return RESULT_ERROR;
    }

  } else {  // ASCII
    // Read line and split into words
    String str;
    if (readStringUntil('\n', str) != RESULT_OK) {
      return RESULT_ERROR;
    }
    str.trim();
    std::vector<String> words = split(str, ',');

    // Text again handled separately
    if (cmd == CMD_TEXT) {
      if (words.size() != numArgs) {
        Serial.println(String("Usage: ") + String((char)cmdByte) + helpMessage);
        return RESULT_ERROR;
      }
      if (execText(words[0].toInt(), words[1].toInt(), words[2].toInt(), words[3]) != RESULT_OK) {
        Serial.println("Error handling text!");
        return RESULT_ERROR;
      }
      Serial.println("OK.");
      return RESULT_OK;
    }

    // Still here? Convert arguments and continue
    for (auto w : words) args.push_back(w.toInt());
  }

  // Double check correct number of arguments
  if (args.size() != numArgs) {
    if (binary) Serial.write(CMD_ERROR | 0x80);
    else {
      Serial.println(String("Usage: ") + String((char)cmdByte) + helpMessage + " (" + numArgs + " args, got " + args.size() + ")");
    }

    return RESULT_ERROR;
  }

  // ...and execute.
  rd::SerialInterface::Result res = execInput(cmd, args);
  if (res == RESULT_ERROR) {
    if (binary) Serial.write(CMD_ERROR | 0x80);
    else Serial.println("Error.");
  } else {
    if(binary) Serial.write(CMD_NOP | 0x80);
    else Serial.println("OK.");
  }

  return res;
}

std::vector<String> rd::SerialInterface::split(const String& str, unsigned char delim) {
  std::vector<String> words;
  unsigned int start = 0, end = 0;
  bool quotes = false;

  while (end < str.length()) {
    if (quotes == true) {
      if (str[end] != '"') {
        end++;
        if (end == str.length()) {
          String substr = str.substring(start, end);
          if (substr.length()) words.push_back(substr);
        }
      } else if (end >= start) {
        quotes = false;
        String substr = str.substring(start, end);
        if (substr.length()) words.push_back(substr);
        start = end + 1;
        end = end + 1;
      }
    } else {
      if (str[end] == '"') {
        quotes = true;
        start++;
        end++;
        continue;
      }

      if (str[end] != delim) {
        end++;
        if (end == str.length()) {
          String substr = str.substring(start, end);
          if (substr.length()) words.push_back(substr);
        }
      } else if (end >= start) {
        String substr = str.substring(start, end);
        if (substr.length()) words.push_back(substr);
        start = end + 1;
        end = end + 1;
      }
    }
  }

  return words;
}
