#include "RWidget.h"

RWidget::RWidget() { 
    needsFullRedraw_ = needsContentsRedraw_ = true; 
    x_ = y_ = 1;
    width_ = height_ = 10;
    borderCol_ = RDisplay::WHITE;
    bgCol_ = RDisplay::DIMGRAY;
    fgCol_ = RDisplay::WHITE;
    fillsBg_ = false;
}

Result RWidget::draw(ConsoleStream* stream) {
    needsCls_ = false;
    needsContentsRedraw_ = false;
    needsFullRedraw_ = false;
    return RES_OK;
}

void RWidget::setPosition(uint8_t x, uint8_t y) { 
    x_ = x; 
    y_ = y; 
}

void RWidget::setSize(uint8_t w, uint8_t h) { 
    width_ = w; 
    height_ = h; 
    setNeedsFullRedraw(); 
    setNeedsCls(); 
}

void RWidget::setNeedsCls(bool needs) { 
    needsCls_ = needs; 
}

void RWidget::setNeedsFullRedraw(bool needs) { 
    needsFullRedraw_ = needs; 
    if(needs) needsContentsRedraw_ = true; 
}

void RWidget::setNeedsContentsRedraw(bool needs) { 
    needsContentsRedraw_ = needs; 
}

void RWidget::setBackgroundColor(uint16_t background) {
    bgCol_ = background;
    needsFullRedraw_ = true;
}

void RWidget::setForegroundColor(uint16_t foreground) {
    fgCol_ = foreground;
    needsFullRedraw_ = true;
}

void RWidget::setBorderColor(uint16_t border) {
    borderCol_ = border;
    needsFullRedraw_ = true;
}