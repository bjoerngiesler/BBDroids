#include <Arduino.h>
#include <vector>
#include <map>

namespace rd {

struct CmdArgDescription {
  uint8_t numArgs;
  const char* helpMessage;
};

enum Command {
  CMD_NOP          = '\n',
  CMD_ERROR        = 'E',
  CMD_LOGO         = 'L',
  CMD_CLS          = 'x',
  CMD_POINT        = 'p',
  CMD_LINE         = 'l',
  CMD_HLINE        = 'h',
  CMD_VLINE        = 'v',
  CMD_RECT         = 'r',
  CMD_FILLEDRECT   = 'R',
  CMD_CIRCLE       = 'c',
  CMD_FILLEDCIRCLE = 'C',
  CMD_TEXT         = 't'
};

#define BINCMD(c) ((uint8_t)(c|0x80))

class SerialInterface {
public:
  typedef enum {
    RESULT_OK,
    RESULT_ERROR,
    RESULT_NOTHING_TO_READ,
    RESULT_RUN_LOGO_SCREEN
  } Result;

  static SerialInterface serial;
  static std::map<Command, CmdArgDescription> cmdArgs;

  bool begin();
  Result handleInput();
  Result execInput(Command cmd, const std::vector<uint8_t> args);
  Result execText(uint8_t x, uint8_t y, uint8_t color, const String& text);

  bool available();

protected:
  uint16_t lookupColor(uint8_t color);
  Result readArgsBinary(uint8_t num, std::vector<uint8_t>& args);
  Result readStringUntil(unsigned char c, String& str);
  std::vector<String> split(const String& str, unsigned char delim=' ');
  SerialInterface();
};

};