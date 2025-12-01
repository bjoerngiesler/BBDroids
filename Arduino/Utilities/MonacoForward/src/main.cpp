#include <Arduino.h>
#include <LibBB.h>
#include <DFPlayerMini_Fast.h>

#include "MonacoForward.h"

void setup(void) {
    Serial.begin(2000000);
    bb::Console::console.initialize(2000000);
    bb::Runloop::runloop.initialize();
    bb::Runloop::runloop.setCycleTimeMicros(25000);
    MonacoForward::monaco.initialize();
}

void loop(void) {
    bb::Console::console.start();
    MonacoForward::monaco.start();    
    bb::Runloop::runloop.start(); // doesn't return
}