#include "RMultiWidget.h"

bool RMultiWidget::addWidget(RWidget* w) {
    Console::console.printfBroadcast("MW: Looking up widget 0x%x\n", w);
    if(std::find(widgets_.begin(), widgets_.end(), w) != widgets_.end()) {
        return false;
    }
    Console::console.printfBroadcast("MW: Adding widget 0x%x\n", w);
    widgets_.push_back(w);
    Console::console.printfBroadcast("MW: Added widget 0x%x\n", w);
    return true;
}

bool RMultiWidget::removeWidget(RWidget* w) {
    std::deque<RWidget*>::iterator iter = std::find(widgets_.begin(), widgets_.end(), w);
    if(iter == widgets_.end()) {
        return false;
    }
    Console::console.printfBroadcast("Removing widget 0x%x\n", *iter);
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