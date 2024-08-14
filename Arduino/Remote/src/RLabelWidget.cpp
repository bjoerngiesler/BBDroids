#include "RLabelWidget.h"

RLabelWidget::RLabelWidget() {
    title_ = "(null)";
    hor_ = HOR_CENTERED;
    ver_ = VER_CENTERED;
    autoscroll_ = true;
    leftwait_ = 1.0; pixelwait_ = 0.1; rightwait_ = 1.0;
    fillsBg_ = true;
    xdelta_ = 0;
}

Result RLabelWidget::draw(ConsoleStream *stream) {
    if(needsFullRedraw_ || needsContentsRedraw_) {
        if(fillsBg_) {
            RDisplay::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        }
        if(frameType_ == FRAME_ALL) {
            RDisplay::display.rect(x_, y_, x_+width_, y_+height_, borderCol_, false);
        } else {
            if(frameType_ & FRAME_BOTTOM) RDisplay::display.hline(x_, y_+height_, width_, borderCol_);
            if(frameType_ & FRAME_TOP) RDisplay::display.hline(x_, y_, width_, borderCol_);
            if(frameType_ & FRAME_LEFT) RDisplay::display.vline(x_, y_, height_, borderCol_);
            if(frameType_ & FRAME_RIGHT) RDisplay::display.vline(x_+width_, y_, height_, borderCol_);
        }
        needsFullRedraw_ = false;
        needsContentsRedraw_ = true;
    }

    if(!needsContentsRedraw_) return RES_OK;

    unsigned int pixelwidth = title_.length() * RDisplay::CHAR_WIDTH;
    int x=0, y=0;
    int availWidth = width_;
    if(frameType_ & FRAME_LEFT) availWidth-=1;
    if(frameType_ & FRAME_RIGHT) availWidth-=1;

    switch(hor_) {
    case TOP_JUSTIFIED:
        if(frameType_&FRAME_TOP) y++;
        break;
    case HOR_CENTERED:
        y = (height_-RDisplay::CHAR_HEIGHT)/2;
        break;
    case BOTTOM_JUSTIFIED:
    default:
        y = height_-RDisplay::CHAR_HEIGHT;
        if(frameType_&FRAME_BOTTOM) y--;
        break;
    }

    if(pixelwidth <= availWidth) { // we fit - don't bother with autoscroll
        switch(ver_) {
        case LEFT_JUSTIFIED:
            if(frameType_&FRAME_LEFT) x++;
            break;
        case VER_CENTERED:
            x = (availWidth-pixelwidth)/2;
            break;
        case RIGHT_JUSTIFIED:
        default:
            x = width_-pixelwidth;
            if(frameType_&FRAME_RIGHT) x--;
            break;
        }
    } else { // handle autoscroll here

    }

    if(title_.length()>0) RDisplay::display.text(x_+x, y_+y, fgCol_, title_);

    needsContentsRedraw_ = false;

    return RES_OK;
}

void RLabelWidget::setTitle(const String& title) {
    title_ = title;
    needsFullRedraw_ = true;
}

void RLabelWidget::setJustification(RLabelWidget::HorizontalJustification hor, RLabelWidget::VerticalJustification ver) {
    hor_ = hor; ver_ = ver;
    needsContentsRedraw_ = true;
}

void RLabelWidget::setAutoscroll(bool autoscroll) {
    autoscroll_ = autoscroll;
    needsContentsRedraw_ = true;
}

void RLabelWidget::setAutoscrollTiming(float leftwait, float pixelwait, float rightwait) {
    leftwait_ = leftwait; 
    pixelwait_ = pixelwait; 
    rightwait_ = rightwait;
}