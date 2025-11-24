#include "Mouse.h"
#include "Config.h"
#include <LibBBRemotes.h>

Mouse Mouse::mouse;

Result Mouse::initialize() {
    bb::printf("Initializing Mouse\n");
    //pinMode(LED_BUILTIN, OUTPUT);
    //digitalWrite(LED_BUILTIN, LOW);
    name_ = "mouse";
    description_ = "Mouse droid";
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    playing_ = false;
    soundOK_ = false;
    protocol_.init("MouseDroid");
    receiver_ = protocol_.createReceiver();
    if(receiver_ == nullptr) {
        bb::printf("Could not create receiver\n");
        return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }

    uint8_t turnInput = receiver_->addInput("turn", remTurn_);
    uint8_t velInput = receiver_->addInput("vel", remVel_);
    uint8_t sndInput = receiver_->addInput("sound", [this](float val){playSoundCB(val, 0);});

    receiver_->setMix(turnInput, AxisMix(0, INTERP_LIN_CENTERED));
    receiver_->setMix(velInput, AxisMix(1, INTERP_LIN_CENTERED));
    receiver_->setMix(sndInput, AxisMix(11, INTERP_LIN_POSITIVE));

    receiver_->setDataFinishedCallback([this](const NodeAddr& addr, uint8_t seqnum) { dataFinishedCB(addr, seqnum); });
    protocol_.setCommTimeoutWatchdog(0.5, [this](Protocol* p, float s) { commTimeoutCB(p, s);});

    return Subsystem::initialize();
}

Result Mouse::start(ConsoleStream *stream) {
    bb::printf("Starting Mouse... ");
    bb::printf("sound... ");
    if(dfp.begin(Serial1) == false) {
        bb::printf("failure.\n");
        soundOK_ = false;
    } else {
        delay(5);
        dfp.volume(30);
        delay(5);
        dfp.play(1);
        bb::printf("success.\n");
        soundOK_ = true;
    }
    //digitalWrite(LED_BUILTIN, HIGH);
    return Subsystem::start(stream);
}

Result Mouse::stop(ConsoleStream *stream) {
    return Subsystem::stop(stream);
}

Result Mouse::step() {
    protocol_.step();

    bb::printf("Turn: %.2f Vel: %.2f\n", remTurn_*90.0+90.0, remVel_*90.0+90.0);
    pwm.writeServo(STEER_SERVO_PIN, remTurn_*90.0+90.0);
    pwm.writeServo(DRIVE_SERVO_PIN, remVel_*90.0+90.0);

    return RES_OK;
}

Result Mouse::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    if(words.size() == 0) return RES_CMD_INVALID_ARGUMENT;

    if(words[0] == "play") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
        dfp.play(words[1].toInt());
        return RES_OK;
    }

    return Subsystem::handleConsoleCommand(words, stream);
}

void Mouse::playSoundCB(float val, uint8_t snd) {
    static bool pressed = false;
    if(val > 0.5 && pressed == false) {
        pressed = true;
        dfp.play(snd);
    } else if(val < 0.5 && pressed == true) {
        pressed = false;
    }
}

void Mouse::commTimeoutCB(Protocol* p, float s) {
    remTurn_ = remVel_ = 0;
    //digitalWrite(LED_BUILTIN, HIGH);
}

void Mouse::dataFinishedCB(const NodeAddr& addr, uint8_t seqnum) {
    //digitalWrite(LED_BUILTIN, LOW);
}
