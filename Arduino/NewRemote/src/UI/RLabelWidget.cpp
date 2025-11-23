#include "UI/RLabelWidget.h"

std::vector<String> RLabelWidget::splitLines(const String& str, unsigned int maxwidth) {
    std::vector<String> retval;
    unsigned int first=0, current=0, lastwsp=0;
    if(str.length() == 0) return retval;

    while(current < str.length()) {
        // found a newline? Push line, increase and continue
        if(str[current] == '\n') {
            retval.push_back(str.substring(first, current));
            first = current+1;
        } 
        
        // found a space? mark its location, increase and continue
        else if(str[current] == ' ') {
            lastwsp = current;
        }

        // string too long? Split at the location of last whitespace, or 
        else if(current-first >= maxwidth) {
            if(lastwsp > first) {
                retval.push_back(str.substring(first, lastwsp));
                first = lastwsp + 1;
                lastwsp = first;
            } else {
                retval.push_back(str.substring(first, current));
                first = current;
            }
        }

        current++;
    }

    if((current-first)>0) {
        retval.push_back(str.substring(first, current));
    }

    return retval;
}

RLabelWidget::RLabelWidget() {
    title_ = "(null)";
    hor_ = HOR_CENTERED;
    ver_ = VER_CENTERED;
    autoscroll_ = true;
    linebreak_ = false;
    leftwait_ = 1.0; pixelwait_ = 0.1; rightwait_ = 1.0;
    fillsBg_ = true;
    frameType_ = FRAME_NONE;
    xdelta_ = 0;
    autosize_ = false;
    linebreak_ = false;
}

Result RLabelWidget::draw(ConsoleStream *stream) {
    if(needsFullRedraw_ || needsContentsRedraw_) {
        if(fillsBg_) {
            Display::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        }
        if(frameType_ == FRAME_ALL) {
            Display::display.rect(x_, y_, x_+width_, y_+height_, frameCol_, false);
        } else {
            if(frameType_ & FRAME_BOTTOM) Display::display.hline(x_, y_+height_, width_, frameCol_);
            if(frameType_ & FRAME_TOP) Display::display.hline(x_, y_, width_, frameCol_);
            if(frameType_ & FRAME_LEFT) Display::display.vline(x_, y_, height_, frameCol_);
            if(frameType_ & FRAME_RIGHT) Display::display.vline(x_+width_, y_, height_, frameCol_);
        }
        needsFullRedraw_ = false;
        needsContentsRedraw_ = true;
    }

    if(!needsContentsRedraw_) return RES_OK;

    int x=0, y=0;
    int availWidth = width_;
    if(frameType_ & FRAME_LEFT) availWidth-=1;
    if(frameType_ & FRAME_RIGHT) availWidth-=1;

    switch(ver_) {
    case TOP_JUSTIFIED:
        if(frameType_&FRAME_TOP) y++;
        break;
    case VER_CENTERED:
        y = rint(float(height_ - lines_.size()*Display::CHAR_HEIGHT)/2.0f);
        break;
    case BOTTOM_JUSTIFIED:
    default:
        y = height_ - lines_.size()*Display::CHAR_HEIGHT;
        if(frameType_&FRAME_BOTTOM) y--;
        break;
    }

    for(unsigned int i=0; i<lines_.size(); i++) {
        if(lines_[i].length() == 0) { 
            y += Display::CHAR_HEIGHT;
            continue;
        }
        if(y + Display::CHAR_HEIGHT > height_) continue;

        unsigned int pixelwidth = lines_[i].length() * Display::CHAR_WIDTH;
        switch(hor_) {
        case LEFT_JUSTIFIED:
            if(frameType_&FRAME_LEFT) x++;
            break;
        case HOR_CENTERED:
            x = rint(float(availWidth-pixelwidth)/2.0f);
            break;
        case RIGHT_JUSTIFIED:
        default:
            x = width_-pixelwidth;
            if(frameType_&FRAME_RIGHT) x--;
            break;
        }

        Display::display.text(x_+x+1, y_+y+1, highlighted_ ? hlCol_ : fgCol_, lines_[i]);
        y += Display::CHAR_HEIGHT;
    }

    needsContentsRedraw_ = false;

    return RES_OK;
}

void RLabelWidget::setTitle(const String& title) {
    if(title_ == title) return;
    
    title_ = title;

    if(linebreak_) {
        if(autosize_) lines_ = RLabelWidget::splitLines(title, (Display::DISPLAY_WIDTH-4)/Display::CHAR_WIDTH);
        else lines_ = RLabelWidget::splitLines(title, width_/Display::CHAR_WIDTH);
    } else {
        lines_.clear();
        lines_.push_back(title_);
    }

    if(autosize_) {
        unsigned int maxLineLen = 0;
        for(auto& str: lines_) {
            if(str.length() > maxLineLen) maxLineLen = str.length();
        }

        unsigned int width = maxLineLen*Display::CHAR_WIDTH+4;
        if(width > Display::DISPLAY_WIDTH) width = Display::DISPLAY_WIDTH;
        unsigned int height = lines_.size()*Display::CHAR_HEIGHT+4;
        if(height > Display::DISPLAY_HEIGHT) height = Display::DISPLAY_HEIGHT;
        setSize(width, height);
    }

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