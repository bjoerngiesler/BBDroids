#include "UI/PercentageBar.h"

PercentageBar::PercentageBar() {
    percent_ = 1.0f;
    start_ = 0.0f;
    orientation_ = HORIZONTAL;
}

Result PercentageBar::draw() {
    if(needsFullRedraw_) {
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, bgCol_, true);
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, frameCol_, false);
        needsFullRedraw_ = false;
        needsContentsRedraw_ = true;
    }

    if(!needsContentsRedraw_) return RES_OK;

    // Clear the inside
    Display::display.rect(x_+1, y_+1, x_+width_-2, y_+height_-2, bgCol_, true);

    // Draw the filled portion
    if(orientation_ == HORIZONTAL) {
        int startX = x_ + 1 + (width_-2) * constrain(start_, 0.0f, 1.0f);
        int endX = x_ + 1 + (width_-2) * constrain(start_ + percent_, 0.0f, 1.0f);
        Display::display.rect(startX, y_+1, endX-1, y_+height_-2, fgCol_, true);
    } else { // VERTICAL
        int startY = y_ + 1 + (height_-2) * constrain(start_, 0.0f, 1.0f);
        int endY = y_ + 1 + (height_-2) * constrain(start_ + percent_, 0.0f, 1.0f);
        Display::display.rect(x_+1, startY, x_+width_-2, endY-1, fgCol_, true);
    }

    needsContentsRedraw_ = false;
    return RES_OK;
}

void PercentageBar::setPercentage(float percent) {
    percent_ = constrain(percent, 0.0f, 1.0f);
    setNeedsContentsRedraw();
}

void PercentageBar::setStart(float percent) {
    start_ = constrain(percent, 0.0f, 1.0f);
    setNeedsContentsRedraw();
}

void PercentageBar::setOrientation(Orientation orient) {
    orientation_ = orient;
    setNeedsContentsRedraw();
}