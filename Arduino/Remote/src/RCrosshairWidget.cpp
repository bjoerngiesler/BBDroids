#include "RCrosshairWidget.h"
#include "RDisplay.h"

RCrosshairWidget::RCrosshairWidget():
    hor_(2048),
    ver_(2048),
    oldHor_(2048),
    oldVer_(2048) {
    setFillsBackground();
}
  
Result RCrosshairWidget::draw(ConsoleStream* stream) {
    if(needsFullRedraw_) {
        if(fillsBg_)
            RDisplay::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        RDisplay::display.rect(x_, y_, x_+width_, y_+height_, borderCol_, false);
        needsFullRedraw_ = false;
    }

    setHorVer(RInput::input.joyRawH, RInput::input.joyRawV);

    if(needsContentsRedraw_) {
        uint8_t x, y;

        // erase old coord cross
        horVerToScreen(oldHor_, oldVer_, x, y);
        RDisplay::display.hline(x_+1, y, width_-2, bgCol_);
        RDisplay::display.vline(x, y_+1, height_-2, bgCol_);

        // center line in grey
        RDisplay::display.hline(x_+1, y_+height_/2, width_-2, RDisplay::DARKBLUE);
        RDisplay::display.vline(x_+width_/2, y_+1, height_-2, RDisplay::DARKBLUE);
        
        // draw new coord cross
        horVerToScreen(hor_, ver_, x, y);
        RDisplay::display.hline(x_+1, y, width_-2, fgCol_);
        RDisplay::display.vline(x, y_+1, height_-2, fgCol_);
    }

    return RWidget::draw(stream);
}

void RCrosshairWidget::setHorVer(uint16_t h, uint16_t v) {
    oldHor_ = hor_; oldVer_ = ver_;
    hor_ = h; ver_ = v;
    setNeedsContentsRedraw();
}

void RCrosshairWidget::horVerToScreen(uint16_t h, uint16_t v, uint8_t& x, uint8_t& y) {
    x = (h*(width_-2))/4096+x_+1; y = (v*(width_-2))/4096+y_+1;
}
