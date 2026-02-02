#include "UI/MixCurveWidget.h"

void MixCurveWidget::setInterpolator(const Interpolator& inter) {
    inter_ = inter;
    setNeedsFullRedraw();
}


Result MixCurveWidget::draw() {
    if(needsFullRedraw_) {
        //bb::rmt::printf("Dimensions: %dx%d+%d+%d\n", width_, height_, x_, y_);
        setNeedsContentsRedraw();
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, bgCol_, true);
        Display::display.rect(x_, y_, x_+width_-1, y_+height_-1, fgCol_, false);
        needsFullRedraw_ = false;
    }

    if(needsContentsRedraw_) {
        drawCalibLines();

        drawNode(0, inter_.i0);
        drawNode(1, inter_.i25);
        drawNode(2, inter_.i50);
        drawNode(3, inter_.i75);
        drawNode(4, inter_.i100);
        drawLineFrom(0, inter_.i0, inter_.i25);
        drawLineFrom(1, inter_.i25, inter_.i50);
        drawLineFrom(2, inter_.i50, inter_.i75);
        drawLineFrom(3, inter_.i75, inter_.i100);

        needsContentsRedraw_ = false;
    }

    return RES_OK;
}

void MixCurveWidget::drawNode(unsigned int num, int8_t position) {
    float innerW = width_-8, innerH = height_-8;
    unsigned int xpos = x_+int(rintf((innerW/4)*num))+4;
    unsigned int ypos = y_+height_/2-int(rintf((innerH/251)*position));

    //bb::rmt::printf("Position for point %d (%d): %d,%d\n", num, position, xpos, ypos);

    if(showsCursor_ && cursor_ == num) {
        Display::display.rect(xpos-3, ypos-3, xpos+3, ypos+3, hlCol_, true);
    } else {
        Display::display.rect(xpos-2, ypos-2, xpos+2, ypos+2, fgCol_, false);
    }
}

void MixCurveWidget::drawLineFrom(unsigned int num, int8_t pos1, int8_t pos2) {
    float innerW = width_-8, innerH = height_-8;
    unsigned int x1 = x_+int(rintf((innerW/4)*num))+4;
    unsigned int x2 = x_+int(rintf((innerW/4)*(num+1)))+4;
    unsigned int y1 = y_+height_/2-int(rintf((innerH/251)*pos1));
    unsigned int y2 = y_+height_/2-int(rintf((innerH/251)*pos2));
    Display::display.line(x1, y1, x2, y2, fgCol_);
}

void MixCurveWidget::drawCalibLines() {
    float innerW = width_-8, innerH = height_-8;
    Display::display.line(x_, y_+height_/2, x_+width_-1, y_+height_/2, Display::LIGHTBLUE2);
    unsigned int y1 = y_+int(rintf((innerH/251)*100))+height_/2;
    unsigned int y2 = y_-int(rintf((innerH/251)*100))+height_/2;
    Display::display.line(x_, y1, x_+width_-1, y1, Display::LIGHTRED1);
    Display::display.line(x_, y2, x_+width_-1, y2, Display::LIGHTRED1);
}

void MixCurveWidget::setShowsCursor(bool shows) {
    if(showsCursor_ == shows) return;
    showsCursor_ = shows;
    setNeedsFullRedraw();
}

void MixCurveWidget::setCursor(uint8_t cursor) {
    if(cursor_ == cursor) return;
    cursor_ = cursor % 5;
    setNeedsFullRedraw();
}
