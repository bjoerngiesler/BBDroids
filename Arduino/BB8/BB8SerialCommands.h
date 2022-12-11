#if !defined(BB8SERIALCOMMANDS_H)
#define BB8SERIALCOMMANDS_H

#include <Arduino.h>
#include <vector>

class BB8SerialCommands {
public:
  static BB8SerialCommands sercmd;

  bool begin();
  bool handleIfAvailable();

protected:
  typedef enum {
    OK,
    FAILURE,
    PRINT_OK,
    PRINT_FAILURE
  } CommandReturn;

  CommandReturn handleHelpCommand(const std::vector<String>& words);
  CommandReturn handleWifiCommand(const std::vector<String>& words);
  CommandReturn handleServoCommand(const std::vector<String>& words);
  CommandReturn handleSoundCommand(const std::vector<String>& words);
  CommandReturn handleSetVariableCommand(const std::vector<String>& words);
  CommandReturn handleControlCommand(const std::vector<String>& words);
  CommandReturn handlePlotCommand(const std::vector<String>& words);
  CommandReturn handleCalibCommand(const std::vector<String>& words);
  CommandReturn handleShowVariablesCommand(const std::vector<String>& words);
  std::vector<String> split(const String& str);
  bool safeStringToFloat(const String& str, float& f, bool printResult = false); // does not change f if str does not describe a float
};

#endif // BB8SERIALCOMMANDS_H