#include "RSeqnumWidget.h"

#define NUMROWS 2
#define NUMCOLS 4

RSeqnumWidget::RSeqnumWidget(): RWidget() {
    for(int c=0; c<NUMCOLS; c++) for(int r=0; r<NUMROWS; r++) {
        Square s = {true, bgCol_};
        squares_.push_back(s);
    }
}

Result RSeqnumWidget::draw(ConsoleStream* stream) {
    if(needsFullRedraw_) {
        RDisplay::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
    }

    bool dirty = needsContentsRedraw_;
    for(auto& s: squares_) {
        if(s.dirty) {
            dirty = true;
            break;
        }
    }
    if(dirty == false) return RES_OK;
     
    int sqw = width_ / NUMCOLS, sqh = height_ / NUMROWS;
    for(int c=0; c<NUMCOLS; c++) {
        for(int r=0; r<NUMROWS; r++) {
            if(squares_[r*NUMCOLS+c].dirty == false && needsContentsRedraw_ == false) continue;
            RDisplay::display.rect(x_+c*sqw, y_+r*sqh, x_+(c+1)*sqw, y_+(r+1)*sqh, squares_[r*NUMCOLS+c].color, true);
        }
    }
    setNeedsContentsRedraw(false);
    setNeedsFullRedraw(false);
    return RES_OK;
}

void RSeqnumWidget::setSquareColor(uint8_t s, uint8_t color) {
    if(s > NUMROWS*NUMCOLS) return;

    if(squares_[s].color == color) return;
    squares_[s].color = color;
    squares_[s].dirty = true;
}