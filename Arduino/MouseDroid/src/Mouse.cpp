#include "Mouse.h"
#include "Config.h"

Mouse Mouse::mouse;

Result Mouse::initialize() {
    bb::printf("Initializing Mouse\n");
    name_ = "mouse";
    description_ = "Mouse droid";
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    playing_ = false;
    soundOK_ = false;
    pinMode(PULSEIN_PIN, INPUT);
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
    return Subsystem::start(stream);
}

Result Mouse::stop(ConsoleStream *stream) {
    return Subsystem::stop(stream);
}

Result Mouse::step() {
    unsigned long p = pulseIn(PULSEIN_PIN, HIGH, 24000);
    if(p == 0) { // something went wrong
        return RES_OK;
    }
    if(p > 1500 && playing_ == false) {
        dfp.play(2);
        playing_ = true;
        bb::printf("Start playback on PWM=%d\n", p);
    } else if(p < 1500 && playing_ == true) {
        dfp.stop();
        playing_ = false;
        bb::printf("Stop playback on PWM=%d\n", p);
    }
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
