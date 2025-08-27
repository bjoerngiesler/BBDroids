#include <Arduino.h>
#include <LibBB.h>
#include <DFPlayerMini_Fast.h>

#include "Mouse.h"

void setup(void) {
    Serial.begin(2000000);
    bb::Console::console.initialize(2000000);
    bb::Runloop::runloop.initialize();
    bb::Runloop::runloop.setCycleTimeMicros(25000);
    Mouse::mouse.initialize();
}

void loop(void) {
    bb::Console::console.start();
    Mouse::mouse.start();    
    bb::Runloop::runloop.start(); // doesn't return
}