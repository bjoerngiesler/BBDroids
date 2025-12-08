#include "UI/MultiWidget.h"

using namespace std;

void MultiWidget::setNeedsContentsRedraw(bool needs) {
    for(auto& w: widgets_) w->setNeedsContentsRedraw(needs);
    needsContentsRedraw_ = needs;
}

void MultiWidget::setNeedsFullRedraw(bool needs) {
    for(auto& w: widgets_) w->setNeedsFullRedraw(needs);
    needsFullRedraw_ = needs;
}

void MultiWidget::setPosition(int x, int y) {
    int deltaX = x - x_, deltaY = y - y_;
    for(auto& w: widgets_) {
        w->setPosition(constrain(w->x() + deltaX, 0, 255), constrain(w->y() + deltaY, 0, 255));
    }
    Widget::setPosition(x, y);
}

Result MultiWidget::draw() {
    if(fillsBg_ && needsFullRedraw_) {
        Display::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
    }
    if(drawsFrame_ && needsFullRedraw_) {
        Display::display.rect(x_, y_, x_+width_, y_+height_, frameCol_, false);
    }
    needsFullRedraw_ = false;
    for(auto w: widgets_) w->draw();
    return RES_OK;
}

void MultiWidget::highlightWidgetsWithTag(int tag) {
  if(widgets_.size() == 0) return;
  for(auto& w: widgets_) {
    if(w->tag() == tag) w->setHighlighted(true);
    else w->setHighlighted(false);
  }
}
