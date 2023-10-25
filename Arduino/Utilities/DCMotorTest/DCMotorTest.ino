#include <LibBB.h>

using namespace bb;

DCMotor motor[2] = {DCMotor(2, 3), DCMotor(18, 19)};
EncoderControlInput input[2] = {EncoderControlInput(17, 16), EncoderControlInput(6, 7)};
DCMotorControlOutput output[2] = {DCMotorControlOutput(motor[0]), DCMotorControlOutput(motor[1])};
PIDController control[2] = {PIDController(input[0], output[0]), PIDController(input[1], output[1])};

const uint8_t MOTOR_0_FLAG = 1;
const uint8_t MOTOR_1_FLAG = 2;
int useMotor = 0;

float speedKp = 0.14, speedKi = 0.8, speedKd = 0.0;
float speedCutoff = 25;
float posKp = 0.12, posKi = 0.4, posKd = 0.0;
float posCutoff = 25;
float goal = 0.0;

const uint8_t MODE_PWM = 0;
const uint8_t MODE_SPEED =   1;
const uint8_t MODE_POSITION = 2;
int mode = 0;

const uint8_t OUTPUT_SERIALPLOTTER = 0;
const uint8_t OUTPUT_SERIALANALYZER = 0;
int outputMode;

class DCMotorTest: public bb::Subsystem {
public:
  Result initialize() {
    name_ = "dcmotortest";
    description_ = "DC Motor Test";
    help_ = 
"BBDROIDS: DC Motor Control Tester\n"\
"=================================\n"\
"Commands:\n"\
"    help                     Print this help\n"\
"    start                    Start the test\n"\
"    stop                     Stop the test\n"\
"While the test is running, information about controller state will be output in a format suited for\n"\
"the Arduino serial plotter. While it is running, you can either enter \"stop\" to stop the test, or\n"\
"enter numerical setpoints.\n";

    addParameter("useMotor", "0: none, 1: motor 0, 2: motor 1, 3: both", useMotor, 0, 3);
    addParameter("mode", "0: PWM, 1: speed, 2: position", mode, 0, 2);
    addParameter("outputMode", "0: output for Arduino Serial Plotter , 1: output for Curio Res Serial Analyzer", outputMode, 0, 1);
    addParameter("speedKp", "P constant for speed control", speedKp);
    addParameter("speedKi", "I constant for speed control", speedKi);
    addParameter("speedKd", "D constant for speed control", speedKd);
    addParameter("posKp", "P constant for position control", posKp);
    addParameter("posKi", "I constant for position control", posKi);
    addParameter("posKd", "D constant for position control", posKd);
    addParameter("goal", "PWM, speed, or position goat", goal);
    addParameter("speedCutoff", "Cutoff frequency for speed filter (Hz)", speedCutoff);
    addParameter("posCutoff", "Cutoff frequency for position filter (Hz)", posCutoff);

    return Subsystem::initialize();    
  }

  Result start(ConsoleStream *stream) {
    started_ = true;
    operationStatus_ = RES_OK;
    control[0].reset();
    control[1].reset();
    return RES_OK;
  }
  
  Result stop(ConsoleStream *stream) {
    started_ = false;
    operationStatus_ = RES_SUBSYS_NOT_STARTED;
    motor[0].setSpeed(0);
    motor[1].setSpeed(0);
    return RES_OK;
  }

  Result step() {
    for(int i=0; i<2; i++) {
      input[i].setSpeedFilterCutoff(speedCutoff);
      input[i].setPositionFilterCutoff(posCutoff);
    }

    if(mode == MODE_PWM) {
      float g = constrain(goal, -255.0, 255.0);
      if(outputMode == OUTPUT_SERIALPLOTTER)
        Console::console.printBroadcast(String("Goal:") + g);
      if(useMotor & MOTOR_0_FLAG) {
        motor[0].setSpeed(g);
        input[0].update();
        if(outputMode == OUTPUT_SERIALPLOTTER)
          Console::console.printBroadcast(String(",RawEnc0:") + input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS));
        else {
          Console::console.printBroadcast(String(input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS, true)));
          Console::console.printBroadcast(" ");
          Console::console.printBroadcast(String(input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS, false)));
        }
      }
      if(useMotor & MOTOR_1_FLAG) {
        motor[1].setSpeed(g);
        input[1].update();
        if(outputMode == OUTPUT_SERIALPLOTTER)
          Console::console.printBroadcast(String(",RawEnc1:") + input[1].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS));
        else {
          //Console::console.printBroadcast(String(input[1].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS)));
          //Console::console.printBroadcast(" ");
        }
      }
      Console::console.printlnBroadcast();
    } 
    
    else if(mode == MODE_SPEED) {
      input[0].setMode(EncoderControlInput::INPUT_SPEED);
      control[0].setControlParameters(speedKp, speedKi, speedKd);
      control[0].setGoal(goal);
      input[1].setMode(EncoderControlInput::INPUT_SPEED);
      control[1].setControlParameters(speedKp, speedKi, speedKd);
      control[1].setGoal(goal);
      if(outputMode == OUTPUT_SERIALPLOTTER) {
        Console::console.printBroadcast(String("Goal:") + goal);
      } else {
        Console::console.printBroadcast(String(goal));
      }
      if(useMotor & MOTOR_0_FLAG) {
        control[0].update();
        if(outputMode == OUTPUT_SERIALPLOTTER) {
          Console::console.printBroadcast(String(",RawEnc0:") + input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS));
        } else {
          Console::console.printBroadcast(" ");
          Console::console.printBroadcast(String(input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS, true)));
          Console::console.printBroadcast(" ");
          Console::console.printBroadcast(String(input[0].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS)));
        }
      }
      if(useMotor & MOTOR_1_FLAG) {
        control[1].update();
        if(outputMode == OUTPUT_SERIALPLOTTER) {
          Console::console.printBroadcast(String(",RawEnc0:") + input[1].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS));
        } else {
          Console::console.printBroadcast(" ");
          Console::console.printBroadcast(String(input[1].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS, true)));
          Console::console.printBroadcast(" ");
          Console::console.printBroadcast(String(input[1].present(EncoderControlInput::INPUT_SPEED, EncoderControlInput::UNIT_TICKS)));
        }
      } 
      Console::console.printlnBroadcast();
    } 
    
    else if(mode == MODE_POSITION) {
      input[0].setMode(EncoderControlInput::INPUT_POSITION);
      control[0].setControlParameters(posKp, posKi, posKd);
      control[0].setGoal(goal);
      input[1].setMode(EncoderControlInput::INPUT_POSITION);
      control[1].setControlParameters(posKp, posKi, posKd);
      control[1].setGoal(goal);
      if(useMotor & MOTOR_0_FLAG) control[0].update();
      if(useMotor & MOTOR_1_FLAG) control[1].update();
    }
    return RES_OK;
  }
  
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

    if(words[0] == "help") {
      stream->println(help_);
      printParameters(stream);
      return RES_OK;
    }

    if(words[0] == "start") {
      start(stream);
      return RES_OK;
    }

    if(words[0] == "stop") {
      stop(stream);
      return RES_OK;
    }

    if(isStarted() && words.size() == 1) {
      bool isNumber = true;
      for(int i=0; i<words[0].length(); i++) {
        if(!isDigit(words[0].charAt(i)) && words[0].charAt(i) != '.' && words[0].charAt(i) != '-') {
          isNumber = false;
          break;
        }
      }
      if(isNumber) {
        float g = words[0].toFloat();
        if(mode == MODE_PWM && (g < -255 || g > 255)) return RES_COMMON_OUT_OF_RANGE;
        goal = g;
        return RES_OK;
      }
    }

    return Subsystem::handleConsoleCommand(words, stream);
  }
};

DCMotorTest sut;

void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(0, LOW);

  Serial.begin(200000);
  while(!Serial);

  Console::console.initialize(200000);
  Runloop::runloop.initialize();
  Runloop::runloop.setCycleTime(10000);
  sut.initialize();

  Console::console.setFirstResponder(&sut);
  Console::console.start();
  Runloop::runloop.start();
}

void loop() {}