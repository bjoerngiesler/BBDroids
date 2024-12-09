#include "RMultiWidget.h"

bool RMultiWidget::addWidget(RWidget* w) {
    if(std::find(widgets_.begin(), widgets_.end(), w) != widgets_.end()) {
        return false;
    }
    widgets_.push_back(w);
    return true;
}

bool RMultiWidget::removeWidget(RWidget* w) {
    std::deque<RWidget*>::iterator iter = std::find(widgets_.begin(), widgets_.end(), w);
    if(iter == widgets_.end()) {
        return false;
    }
    widgets_.erase(iter);
    return true;
}

bool RMultiWidget::hasWidget(RWidget* w) {
    return (std::find(widgets_.begin(), widgets_.end(), w) != widgets_.end());
}

void RMultiWidget::setNeedsContentsRedraw(bool needs) {
    for(auto& w: widgets_) w->setNeedsContentsRedraw(needs);
    needsContentsRedraw_ = needs;
}

void RMultiWidget::setNeedsFullRedraw(bool needs) {
    for(auto& w: widgets_) w->setNeedsFullRedraw(needs);
    needsFullRedraw_ = needs;
}

void RMultiWidget::setPosition(int x, int y) {
    int deltaX = x - x_, deltaY = y - y_;
    for(auto& w: widgets_) {
        w->setPosition(constrain(w->x() + deltaX, 0, 255), constrain(w->y() + deltaY, 0, 255));
    }
    RWidget::setPosition(x, y);
}

Result RMultiWidget::draw(ConsoleStream* stream) {
    if(fillsBg_ && needsFullRedraw_) {
        RDisplay::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
        needsFullRedraw_ = false;
    }
    for(auto w: widgets_) w->draw(stream);
    return RES_OK;
}

void RMultiWidget::highlightWidgetsWithTag(int tag) {
  if(widgets_.size() == 0) return;
  for(auto& w: widgets_) {
    if(w->tag() == tag) w->setHighlighted(true);
    else w->setHighlighted(false);
  }
}
