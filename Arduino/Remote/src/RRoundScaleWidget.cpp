#include "RRoundScaleWidget.h"

RRoundScaleWidget::RRoundScaleWidget() {
    angle_ = oldAngle_ = 0;
    start_ = 0;
    end_ = 360;
    min_ = 0; max_ = infinityf();
    infinite_ = true;
}

Result RRoundScaleWidget::draw(ConsoleStream* stream) {
    int x, y, radius;
    if(width_ > height_) {
        radius = height_ / 2;
    } else {
        radius = width_ / 2;
    }
    x = width_ / 2 + x_;
    y = height_ / 2 + y_;

    if(needsFullRedraw_) {
        RDisplay::display.circle(x, y, radius, bgCol_, true);
        RDisplay::display.circle(x, y, radius, frameCol_, false);
        needsFullRedraw_ = false;
        needsContentsRedraw_ = true;
    }

    if(!needsContentsRedraw_) return RES_OK;

    // make sure we don't destroy the outline
    if(radius > 5) radius -= 1;
    if(radius > 3) radius -= 1;

    RDisplay::display.line(x, y, x+radius*cos(oldAngle_), y+radius*sin(oldAngle_), bgCol_);
    if(!infinite_) {
        RDisplay::display.line(x, y, x+radius*cos(start_), y+radius*sin(start_), markingCol_);
        RDisplay::display.line(x, y, x+radius*cos(end_), y+radius*sin(end_), markingCol_);
    }
    RDisplay::display.line(x, y, x+radius*cos(angle_), y+radius*sin(angle_), cursorCol_);
    oldAngle_ = angle_;
    needsContentsRedraw_ = false;

    return RES_OK;
}

void RRoundScaleWidget::setAngle(float angle, bool radians) {
    if(radians) angle_ = angle;
    else angle_ = DEG_TO_RAD*angle;
    setNeedsContentsRedraw();
}

void RRoundScaleWidget::setStartEndAngle(float start, float end) {
    start_ = DEG_TO_RAD*start; end_ = DEG_TO_RAD*end; 
    infinite_ = false;
    setNeedsContentsRedraw();
}

void RRoundScaleWidget::setValue(float value) {
    value = constrain(value, min_, max_);
    float angle = start_ + (value-min_)/(max_-min_) * (end_-start_);
    setAngle(angle, true);
}

void RRoundScaleWidget::setMinMaxValue(float min, float max) {
    min_ = min; max_ = max;
}

    
void RRoundScaleWidget::setInfinite() {
    infinite_ = true;
    setNeedsContentsRedraw();
}

