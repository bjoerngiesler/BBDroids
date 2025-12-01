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

    uint8_t domeInput = receiver_->addInput("dome", remDome_);
    uint8_t turnInput = receiver_->addInput("turn", remTurn_);
    uint8_t velInput = receiver_->addInput("vel", remVel_);
    uint8_t sndInput1 = receiver_->addInput("sound1", [this](float val){playSoundCB(val, 0);});
    uint8_t sndInput2 = receiver_->addInput("sound2", [this](float val){playSoundCB(val, 1);});
    uint8_t sndInput3 = receiver_->addInput("sound3", [this](float val){playSoundCB(val, 2);});
    uint8_t sndInput4 = receiver_->addInput("sound4", [this](float val){playSoundCB(val, 3);});

    receiver_->setMix(turnInput, AxisMix(0, INTERP_LIN_CENTERED_INV));
    receiver_->setMix(velInput, AxisMix(1, INTERP_LIN_CENTERED_INV));
    receiver_->setMix(domeInput, AxisMix(0+SECONDARY_ADD, INTERP_LIN_CENTERED));
    receiver_->setMix(sndInput1, AxisMix(11, INTERP_LIN_POSITIVE));
    receiver_->setMix(sndInput2, AxisMix(12, INTERP_LIN_POSITIVE));
    receiver_->setMix(sndInput3, AxisMix(13, INTERP_LIN_POSITIVE));
    receiver_->setMix(sndInput4, AxisMix(14, INTERP_LIN_POSITIVE));

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

    // For Mouse
    //bb::printf("Turn: %.2f Vel: %.2f\n", remTurn_*90.0+90.0, remVel_*90.0+90.0);
    //pwm.writeServo(STEER_SERVO_PIN, remTurn_*90.0+90.0);
    //pwm.writeServo(DRIVE_SERVO_PIN, remVel_*90.0+90.0);

    // For R2

    float MAXSPEED = 50;
    float MAXTURN = 40;
    float MIN = 30, MAX = 150;
    float servoL, servoR, servoDome;
    servoL = 90 + remVel_*MAXSPEED + remTurn_*MAXTURN;
    servoR = 90 + remVel_*MAXSPEED - remTurn_*MAXTURN;
    servoDome = 90 + remDome_*80;
    
    if(servoL < MIN) servoL = MIN; else if(servoL > MAX) servoL = MAX;
    if(servoR < MIN) servoR = MIN; else if(servoR > MAX) servoR = MAX;
    if(servoDome < 0) servoDome = 0; else if(servoDome > 180) servoDome = 180;
    bb::printf("Turn: %.2f Vel: %.2f MotL: %.1f MotR: %.1f MotDome: %.1f\n", remTurn_*90.0+90.0, remVel_*90.0+90.0, servoL, servoR, servoDome);
    pwm.writeServo(MOTL_SERVO_PIN, uint8_t(servoL));
    pwm.writeServo(MOTR_SERVO_PIN, uint8_t(servoR));
    pwm.writeServo(DOME_SERVO_PIN, uint8_t(servoDome));

    sendTelemetry();

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
    if(snd >= 4) {
        bb::printf("Unknown sound index %d\n", snd);
        return;
    }
    if(val > 0.5 && pressed[snd] == false) {
        pressed[snd] = true;
        bb::printf("Playing sound %d\n", snd);
        dfp.play(snd);
    } else if(val < 0.5 && pressed[snd] == true) {
        pressed[snd] = false;
    }
}

void Mouse::commTimeoutCB(Protocol* p, float s) {
    remTurn_ = remVel_ = remDome_ = 0;
    //digitalWrite(LED_BUILTIN, HIGH);
}

void Mouse::dataFinishedCB(const NodeAddr& addr, uint8_t seqnum) {
    //digitalWrite(LED_BUILTIN, LOW);
}

Result Mouse::sendTelemetry() {
    static unsigned long lastTelemetryMS = 0;

    if(WRAPPEDDIFF(millis(), lastTelemetryMS, ULONG_MAX) <= 100) {
        return RES_OK;
    }

    lastTelemetryMS = millis();

    Telemetry telem;
    
    telem.overallStatus = (operationStatus_ == RES_OK) ? Telemetry::STATUS_OK : Telemetry::STATUS_ERROR;
    telem.batteryStatus = Telemetry::STATUS_NA;
    telem.driveStatus = Telemetry::STATUS_OK; // not that we could tell... here's to hopin'
    telem.servoStatus = Telemetry::STATUS_OK;
    telem.driveMode = Telemetry::DRIVE_VEL; // don't have any other
    telem.imuPitch = telem.imuRoll = telem.imuHeading = 0;

    // we don't know our speed, so here's the roughest estimate that we can give. Max speed is about 10kph.
    telem.speed =  2.7*remVel_;
    
    if(protocol_.sendTelemetry(telem) == true) return RES_OK;
    return RES_SUBSYS_COMM_ERROR;
}