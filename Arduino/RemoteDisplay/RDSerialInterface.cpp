#include "RDSerialInterface.h"
#include "GFX4dIoD9.h"

extern GFX4dIoD9 gfx;

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

rd::SerialInterface::Result rd::SerialInterface::handleInput() {
  if(!Serial.available()) {
    return RESULT_NOTHING_TO_READ;
  }

	String str;
  
  if(readStringUntil('\n', str) == false) return RESULT_ERROR;

	str.trim();
	std::vector<String> words = split(str);

	if(words.size() == 0) {
    Serial.println("OK.");
		return RESULT_OK;
	}

  if(words[0] == "logo") {
    if(words.size()!=2) {
      Serial.println("Usage: logo backlight|nobacklight");
      return RESULT_ERROR;
    }
    if(words[1] == "backlight") {
      gfx.BacklightOn(true); 
    } else {
      gfx.BacklightOn(false);
    }
    Serial.println("OK.");
    return RESULT_RUN_LOGO_SCREEN;
  }

  else if(words[0] == "cls") {
    gfx.Cls();
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "print") {
    if(words.size()!=5) {
      Serial.println("Usage: print X Y COLOR STRING");
      return RESULT_ERROR;
    }
    int x = words[1].toInt();
    int y = words[2].toInt();
    uint16_t color;
    if(words[3].startsWith("0x")) {
      color = strtol(words[3].c_str(), NULL, 16);
    } else {
      color = words[3].toInt();
    }

    gfx.MoveTo(x, y);
    gfx.TextColor(color);
    gfx.print(words[4]);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "hline") {
    if(words.size()!=5) {
      Serial.println("Usage: hline X Y WIDTH COLOR");
      return RESULT_ERROR;
    }
    int x = words[1].toInt();
    int y = words[2].toInt();
    int width = words[3].toInt();
    uint16_t color;
    if(words[4].startsWith("0x")) {
      color = strtol(words[4].c_str(), NULL, 16);
    } else {
      color = words[4].toInt();
    }

    gfx.Hline(x, y, width, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "vline") {
    if(words.size()!=5) {
      Serial.println("Usage: vline X Y HEIGHT COLOR");
      return RESULT_ERROR;
    }
    int x = words[1].toInt();
    int y = words[2].toInt();
    int height = words[3].toInt();
    uint16_t color;
    if(words[4].startsWith("0x")) {
      color = strtol(words[4].c_str(), NULL, 16);
    } else {
      color = words[4].toInt();
    }

    gfx.Vline(x, y, height, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "line") {
    if(words.size()!=6) {
      Serial.println("Usage: line X1 Y1 X2 Y2 COLOR");
      return RESULT_ERROR;
    }
    int x1 = words[1].toInt();
    int y1 = words[2].toInt();
    int x2 = words[3].toInt();
    int y2 = words[4].toInt();
    uint16_t color;
    if(words[5].startsWith("0x")) {
      color = strtol(words[5].c_str(), NULL, 16);
    } else {
      color = words[5].toInt();
    }

    gfx.Line(x1, y1, x2, y2, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "circle") {
    if(words.size()!=5) {
      Serial.println("Usage: circle X Y RADIUS COLOR");
      return RESULT_ERROR;
    }
    int x = words[1].toInt();
    int y = words[2].toInt();
    int radius = words[3].toInt();
    uint16_t color;
    if(words[4].startsWith("0x")) {
      color = strtol(words[4].c_str(), NULL, 16);
    } else {
      color = words[4].toInt();
    }

    gfx.Circle(x, y, radius, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "circlefilled") {
    if(words.size()!=5) {
      Serial.println("Usage: circlefilled X Y RADIUS COLOR");
      return RESULT_ERROR;
    }
    int x = words[1].toInt();
    int y = words[2].toInt();
    int radius = words[3].toInt();
    uint16_t color;
    if(words[4].startsWith("0x")) {
      color = strtol(words[4].c_str(), NULL, 16);
    } else {
      color = words[4].toInt();
    }

    gfx.CircleFilled(x, y, radius, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "rect") {
    if(words.size()!=6) {
      Serial.println("Usage: rect X1 Y1 x2 y2 COLOR");
      return RESULT_ERROR;
    }
    int x1 = words[1].toInt();
    int y1 = words[2].toInt();
    int x2 = words[3].toInt();
    int y2 = words[4].toInt();
    uint16_t color;
    if(words[5].startsWith("0x")) {
      color = strtol(words[5].c_str(), NULL, 16);
    } else {
      color = words[5].toInt();
    }

    gfx.Rectangle(x1, y1, x2, y2, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "rectfilled") {
    if(words.size()!=6) {
      Serial.println("Usage: rectfilled X1 Y1 x2 y2 COLOR");
      return RESULT_ERROR;
    }
    int x1 = words[1].toInt();
    int y1 = words[2].toInt();
    int x2 = words[3].toInt();
    int y2 = words[4].toInt();
    uint16_t color;
    if(words[5].startsWith("0x")) {
      color = strtol(words[5].c_str(), NULL, 16);
    } else {
      color = words[5].toInt();
    }

    gfx.RectangleFilled(x1, y1, x2, y2, color);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "backlight") {
    if(words.size()!=2) {
      Serial.println("Usage: backlight on|off");
      return RESULT_ERROR;
    }
    if(words[1] == "on") gfx.BacklightOn(true);
    else gfx.BacklightOn(false);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else if(words[0] == "scroll") {
    if(words.size()!=2) {
      Serial.println("Usage: scroll NUMPIXELS");
      return RESULT_ERROR;
    }
    int numpixels = words[1].toInt();
    gfx.Scroll(numpixels);
    Serial.println("OK.");
    return RESULT_OK;
  }

  else Serial.println(String("Unknown command \"") + words[0] + "\".");
  return RESULT_ERROR;
}

bool rd::SerialInterface::readStringUntil(unsigned char c, String& str) { 
	unsigned char input = Serial.read();

	if(input == '\b') {
		if(curStr_.length() > 0) curStr_.remove(curStr_.length()-1);
	} else {
		curStr_ += (char)input;
	}

	Serial.flush();
	str = curStr_;
	if(input == c) {
		curStr_ = "";
		return true;
	}

	return false;
}

std::vector<String> rd::SerialInterface::split(const String& str) {
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
