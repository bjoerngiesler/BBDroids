#include "LRBase.h"
#include "Input.h"
#include "RemoteSubsys.h"

extern bool isLeftRemote;

LRBase::LRBase() {
    runningStatus_ = false;
    help_ =
"Remote Controller Subsystem\n"
"Available commands:\n"
"\trunning_status [on|off]\tDump continuous status\n";
    transmitter_ = nullptr;
    currentProto_ = nullptr;
}

Result LRBase::step() {
    if(runningStatus_) printRunningStatus();
    return RES_OK;
}

Result LRBase::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    if(runningStatus_ == true) {
        runningStatus_ = false;
        Console::console.setFirstResponder(&(Console::console));
        return RES_OK;
    }

    if(words.size() < 1) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if(words[0] == "running_status") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
        if(words[1] == "on") {
            runningStatus_ = true;
            bb::printf("\nStarting running status. Blind-type any letter followed by Return to stop.\n");
            Console::console.setFirstResponder(this);
        } else if(words[1] == "off") {
            runningStatus_ = false;
            Console::console.setFirstResponder(&(Console::console));
        }
        else return RES_CMD_INVALID_ARGUMENT;
        return RES_OK;
    }

    return Subsystem::handleConsoleCommand(words, stream);
}

String LRBase::statusLine() {
  String str = bb::Subsystem::statusLine() + ", ";
  if(Input::inst.imuOK()) str += "IMU OK, ";
  else str += "IMU error, ";
  if(Input::inst.mcpOK()) str += "Buttons OK.";
  else str += "Buttons error.";

  return str;
}

void LRBase::printExtendedStatus(ConsoleStream* stream) {
  Runloop::runloop.excuseOverrun();

  Protocol* proto = RemoteSubsys::inst.currentProtocol();

  stream->printf(isLeftRemote ? "Left remote status\n" : "Right remote status\n");
  stream->printf("Software version: " VERSION_STRING "\n");
  stream->printf("Sequence number: %ld\n", proto->seqnum());
  stream->printf("Primary remote: %s\n", proto->transmitter()->isPrimary() ? "Yes" : "No");
  stream->printf("Joystick:\n");
  stream->printf("\tHor: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", Input::inst.joyRawH, Input::inst.joyH, Input::inst.hCalib.min, Input::inst.hCalib.center, Input::inst.hCalib.max);
  stream->printf("\tVer: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", Input::inst.joyRawV, Input::inst.joyV, Input::inst.vCalib.min, Input::inst.vCalib.center, Input::inst.vCalib.max);

  if(Input::inst.imuOK()) {
    float pitch, roll, heading, rax, ray, raz, ax, ay, az;
    Input::inst.imu().getFilteredPRH(pitch, roll, heading);
    Input::inst.imu().getAccelMeasurement(rax, ray, raz);
    Input::inst.imu().getGravCorrectedAccel(ax, ay, az);
    stream->printf("IMU: OK\n");
    stream->printf("\tRotation             Pitch: %.2f Roll: %.2f Heading: %.2f\n", pitch, roll, heading);
    stream->printf("\tRaw Acceleration     X:%f Y:%f Z:%f\n", rax, ray, raz);
    stream->printf("\tGrav-corrected accel X:%f Y:%f Z:%f\n", ax, ay, az);
  } else {
    stream->printf("IMU: Error\n");
  }

  if(Input::inst.mcpOK()) {      
    stream->printf("Buttons: 1:%c 2:%c 3:%c 4:%c Joy:%c Confirm:%c Left:%c Right:%c\n",
                  Input::inst.buttons[Input::BUTTON_1] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_2] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_3] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_4] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_JOY] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_CONFIRM] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_LEFT] ? 'X' : '_',
                  Input::inst.buttons[Input::BUTTON_RIGHT] ? 'X' : '_');
  } else {
    stream->printf("Buttons: Error\n");
  }

  stream->printf("Potentiometer 1: %d %.2f\nPotentiometer 2: %d %.2f\n", Input::inst.pot1Raw, Input::inst.pot1, Input::inst.pot2Raw, Input::inst.pot2);
  stream->printf("Battery: %.1f\n", Input::inst.battery);
}

void LRBase::printRunningStatus() {
    static const unsigned int bufsize = 255;
    static char buf[bufsize];
    memset(buf, 0, bufsize);
    memset(buf, ' ', bufsize-1);

    float pitch, roll, heading, rax, ray, raz, ax, ay, az;
    Input::inst.imu().getFilteredPRH(pitch, roll, heading);
    Input::inst.imu().getAccelMeasurement(rax, ray, raz);
    Input::inst.imu().getGravCorrectedAccel(ax, ay, az);

    snprintf(buf, bufsize, "S%ld H%4d [%4d..%4d..%4d] %+2.1f V%4d [%4d..%4d..%4d] %+2.1f P%+6.1f R%+6.1f H%+6.1f AX%+4.2f AY%+4.2f AZ%+4.2f P1%3.1f P2%3.1f Batt%3.1f B%c%c%c%c%c%c%c%c",
        seqnum_,
        Input::inst.joyRawH, Input::inst.hCalib.min, Input::inst.hCalib.center, Input::inst.hCalib.max, Input::inst.joyH,
        Input::inst.joyRawV, Input::inst.vCalib.min, Input::inst.vCalib.center, Input::inst.vCalib.max, Input::inst.joyV,
        pitch, roll, heading,
        ax, ay, az,
        Input::inst.pot1, Input::inst.pot2,
        Input::inst.battery,
        Input::inst.buttons[Input::BUTTON_1] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_2] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_3] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_4] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_JOY] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_CONFIRM] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_LEFT] ? '_' : 'X',
        Input::inst.buttons[Input::BUTTON_RIGHT] ? '_' : 'X');
    bb::printf("%s\r", buf);
}
