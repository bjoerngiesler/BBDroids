#include "RLabelWidget.h"

RLabelWidget::RLabelWidget() {
    title_ = "(null)";
    hor_ = HOR_CENTERED;
    ver_ = VER_CENTERED;
    autoscroll_ = true;
    leftwait_ = 1.0; pixelwait_ = 0.1; rightwait_ = 1.0;
    xdelta_ = 0;
}

Result RLabelWidget::draw(ConsoleStream *stream) {
    if(needsFullRedraw_ || needsContentsRedraw_) {
        if(fillsBg_)
            RDisplay::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        if(drawsFrame_)
            RDisplay::display.rect(x_, y_, x_+width_, y_+height_, borderCol_, false);
        needsFullRedraw_ = false;
    }


    unsigned int pixelwidth = title_.length() * RDisplay::CHAR_WIDTH;
    int x, y;
    int availWidth = width_;
    if(drawsFrame_) availWidth-=2;

    switch(hor_) {
    case TOP_JUSTIFIED:
        y = 0; 
        if(drawsFrame_) y++;
        break;
    case HOR_CENTERED:
        y = (height_-RDisplay::CHAR_HEIGHT)/2;
        break;
    case BOTTOM_JUSTIFIED:
    default:
        y = height_-RDisplay::CHAR_HEIGHT;
        if(drawsFrame_) y--;
        break;
    }

    if(pixelwidth <= availWidth) { // we fit - don't bother with autoscroll
        switch(ver_) {
        case LEFT_JUSTIFIED:
            x = 0; 
            if(drawsFrame_) x++;
            break;
        case VER_CENTERED:
            x = (availWidth-pixelwidth)/2;
            break;
        case RIGHT_JUSTIFIED:
        default:
            x = width_-pixelwidth;
            if(drawsFrame_) x--;
            break;
        }
    } else { // handle autoscroll here

    }

    RDisplay::display.text(x_+x, y_+y, fgCol_, title_);

    needsContentsRedraw_ = false;

    return RWidget::draw(stream);
}

void RLabelWidget::setTitle(const String& title) {
    title_ = title;
    needsContentsRedraw_ = true;
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