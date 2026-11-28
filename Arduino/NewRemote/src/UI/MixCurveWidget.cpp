#include "UI/MixCurveWidget.h"
#include "UI/Display.h"

void MixCurveWidget::setInterpolator(const Interpolator& inter) {
    inter_ = inter;
    setNeedsFullRedraw();
}

int8_t MixCurveWidget::pointValue(uint8_t num) {
    switch(num) {
    case 0: return inter_.i0;
    case 1: return inter_.i25;
    case 2: return inter_.i50;
    case 3: return inter_.i75;
    default: return inter_.i100;
    }
}

void MixCurveWidget::setPointValue(uint8_t num, int value) {
    if(value < MIN_VALUE) value = MIN_VALUE;
    if(value > MAX_VALUE) value = MAX_VALUE;
    switch(num) {
    case 0: inter_.i0 = value; break;
    case 1: inter_.i25 = value; break;
    case 2: inter_.i50 = value; break;
    case 3: inter_.i75 = value; break;
    default: inter_.i100 = value; break;
    }
    setNeedsFullRedraw();
}

// The points sit 4 pixels inside the frame so the node boxes (+-3) stay inside it.
int MixCurveWidget::pointX(unsigned int num) {
    float innerW = width_-8;
    return x_ + 4 + int(rintf((innerW/(NUM_POINTS-1))*num));
}

int MixCurveWidget::pointY(int value) {
    float innerH = height_-8;
    return y_ + height_/2 - int(rintf((innerH/(MAX_VALUE-MIN_VALUE))*value));
}

Result MixCurveWidget::draw() {
    if(needsFullRedraw_) {
        setNeedsContentsRedraw();
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, bgCol_, true);
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, highlighted_ ? hlCol_ : fgCol_, false);
        needsFullRedraw_ = false;
    }

    if(needsContentsRedraw_) {
        drawCalibLines();
        for(unsigned int i=0; i<NUM_POINTS-1; i++) {
            Display::display.line(pointX(i), pointY(pointValue(i)), pointX(i+1), pointY(pointValue(i+1)), fgCol_);
        }
        // Nodes after lines, so the cursor node isn't overdrawn.
        for(unsigned int i=0; i<NUM_POINTS; i++) {
            drawNode(i, pointValue(i));
        }
        needsContentsRedraw_ = false;
    }

    return RES_OK;
}

void MixCurveWidget::drawNode(unsigned int num, int8_t position) {
    int xpos = pointX(num), ypos = pointY(position);

    if(showsCursor_ && cursor_ == num) {
        uint8_t col = editing_ ? Display::YELLOW : hlCol_;
        Display::display.rect(xpos-3, ypos-3, xpos+3, ypos+3, col, true);
    } else {
        Display::display.rect(xpos-2, ypos-2, xpos+2, ypos+2, fgCol_, false);
    }
}

void MixCurveWidget::drawCalibLines() {
    // zero line, and +-100% lines
    Display::display.line(x_+1, pointY(0), x_+width_-2, pointY(0), Display::LIGHTBLUE2);
    Display::display.line(x_+1, pointY(100), x_+width_-2, pointY(100), Display::LIGHTRED1);
    Display::display.line(x_+1, pointY(-100), x_+width_-2, pointY(-100), Display::LIGHTRED1);
}

void MixCurveWidget::setShowsCursor(bool shows) {
    if(showsCursor_ == shows) return;
    showsCursor_ = shows;
    setNeedsFullRedraw();
}

void MixCurveWidget::setCursor(uint8_t cursor) {
    cursor %= NUM_POINTS;
    if(cursor_ == cursor) return;
    cursor_ = cursor;
    setNeedsFullRedraw();
}

void MixCurveWidget::setEditing(bool editing) {
    if(editing_ == editing) return;
    editing_ = editing;
    setNeedsFullRedraw();
}
