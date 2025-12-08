#include "UI/CrosshairWidget.h"
#include "UI/Display.h"

CrosshairWidget::CrosshairWidget():
    hor_(2048),
    ver_(2048),
    oldHor_(2048),
    oldVer_(2048),
    minHor_(2048),
    maxHor_(2048),
    minVer_(2048),
    maxVer_(2048),
    oldMinHor_(2048),
    oldMaxHor_(2048),
    oldMinVer_(2048),
    oldMaxVer_(2048),
    showsMinMaxRect_(false) {
    setFillsBackground();
    setDrawsFrame();
    minMaxRectCol_ = fgCol_;
}
  
Result CrosshairWidget::draw() {
    if(needsFullRedraw_) {
        if(fillsBg_)
            Display::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        if(drawsFrame_)
            Display::display.rect(x_, y_, x_+width_, y_+height_, frameCol_, false);
        needsFullRedraw_ = false;
    }

    if(needsContentsRedraw_) {
        uint8_t x, y;
        uint8_t x1, y1, x2, y2;

        if(showsMinMaxRect_) {
            horVerToScreen(oldMinHor_, oldMinVer_, x1, y1);
            horVerToScreen(oldMaxHor_, oldMaxVer_, x2, y2);
            Display::display.rect(x1, y1, x2, y2, bgCol_);
            horVerToScreen(minHor_, minVer_, x1, y1);
            horVerToScreen(maxHor_, maxVer_, x2, y2);
            Display::display.rect(x1, y1, x2, y2, minMaxRectCol_);
        }

        // erase old coord cross
        horVerToScreen(oldHor_, oldVer_, x, y);
        Display::display.hline(x_+1, y, width_-2, bgCol_);
        Display::display.vline(x, y_+1, height_-2, bgCol_);

        // center cross in grey
        Display::display.hline(x_+1, y_+height_/2, width_-2, markingCol_);
        Display::display.vline(x_+width_/2, y_+1, height_-2, markingCol_);
        
        // draw new coord cross
        horVerToScreen(hor_, ver_, x, y);
        Display::display.hline(x_+1, y, width_-2, cursorCol_);
        Display::display.vline(x, y_+1, height_-2, cursorCol_);

        oldHor_ = hor_; oldVer_ = ver_;
        oldMinHor_ = minHor_; oldMaxHor_ = maxHor_;
        oldMinVer_ = minVer_; oldMaxVer_ = maxVer_;
    }

    return Widget::draw();
}

void CrosshairWidget::setHorVer(uint16_t h, uint16_t v) {
    hor_ = h; ver_ = v;
    if(hor_ < minHor_) {
        minHor_ = hor_;
    }
    if(hor_ > maxHor_) {
        maxHor_ = hor_;
    }
    if(ver_ < minVer_) {
        minVer_ = ver_;
    }
    if(ver_ > maxVer_) {
        maxVer_ = ver_;
    }
    setNeedsContentsRedraw();
}

void CrosshairWidget::setHorVer(float h, float v) {
    oldHor_ = hor_; oldVer_ = ver_;
    setHorVer(uint16_t((h+1.0)*2048), uint16_t((v+1.0)*2048));
    setNeedsContentsRedraw();
}

void CrosshairWidget::horVerToScreen(uint16_t h, uint16_t v, uint8_t& x, uint8_t& y) {
    x = (h*(width_-2))/4096+x_+1; y = ((4096-v)*(width_-2))/4096+y_+1;
}

void CrosshairWidget::showMinMaxRect(bool show) {
    if(showsMinMaxRect_ == show) return;
    showsMinMaxRect_ = show;
    setNeedsFullRedraw();
}