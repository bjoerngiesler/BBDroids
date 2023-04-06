#include <Arduino.h>
#include <vector>

namespace rd {

class SerialInterface {
public:
  typedef enum {
    RESULT_OK,
    RESULT_ERROR,
    RESULT_NOTHING_TO_READ,
    RESULT_RUN_LOGO_SCREEN
  } Result;

  static SerialInterface serial;

  bool begin();
  Result handleInput();
  bool available();

protected:
  bool readStringUntil(unsigned char c, String& str);
  std::vector<String> split(const String& str);
  SerialInterface();

  String curStr_;
};

};