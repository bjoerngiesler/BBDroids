#include "RCrosshair.h"
#include "RDisplay.h"

RCrosshair::RCrosshair():
    hor_(2048),
    ver_(2048),
    oldHor_(2048),
    oldVer_(2048) {
}
  
Result RCrosshair::draw(ConsoleStream* stream) {
    if(needsFullRedraw_) {
        RDisplay::display.rect(x_, y_, x_+width_, y_+height_, RDisplay::BLACK, true);
        RDisplay::display.rect(x_, y_, x_+width_, y_+height_, RDisplay::WHITE, false);
        needsFullRedraw_ = false;
    }

    setHorVer(RInput::input.joyRawH, RInput::input.joyRawV);

    if(needsContentsRedraw_) {
        uint8_t x, y;

        // erase old coord cross
        horVerToScreen(oldHor_, oldVer_, x, y);
        RDisplay::display.hline(x_+1, y, width_-2, RDisplay::BLACK);
        RDisplay::display.vline(x, y_+1, height_-2, RDisplay::BLACK);

        // center line in grey
        RDisplay::display.hline(x_+1, y_+height_/2, width_-2, RDisplay::DIMGRAY);
        RDisplay::display.vline(x_+width_/2, y_+1, height_-2, RDisplay::DIMGRAY);
        
        // draw new coord cross
        horVerToScreen(hor_, ver_, x, y);
        RDisplay::display.hline(x_+1, y, width_-2, RDisplay::LIGHTSTEELBLUE);
        RDisplay::display.vline(x, y_+1, height_-2, RDisplay::LIGHTSTEELBLUE);
        needsContentsRedraw_ = false;
    }

    return RES_OK;
}

void RCrosshair::setHorVer(uint16_t h, uint16_t v) {
    oldHor_ = hor_; oldVer_ = ver_;
    hor_ = h; ver_ = v;
    setNeedsContentsRedraw();
}

void RCrosshair::horVerToScreen(uint16_t h, uint16_t v, uint8_t& x, uint8_t& y) {
    x = (h*(width_-2))/4096+x_+1; y = (v*(width_-2))/4096+y_+1;
}
