#include "RWidget.h"

RWidget::RWidget(): name_("Widget") { 
    needsFullRedraw_ = needsContentsRedraw_ = true; 
    x_ = y_ = 1;
    width_ = height_ = 10;
    borderCol_ = RDisplay::WHITE;
    bgCol_ = RDisplay::BLACK;
    fgCol_ = RDisplay::WHITE;
    cursorCol_ = RDisplay::LIGHTGREEN1;
    markingCol_ = RDisplay::BLUE;
    fillsBg_ = false;
    drawsFrame_ = false;
    action_ = nullptr;
}

Result RWidget::draw(ConsoleStream* stream) {
    needsContentsRedraw_ = false;
    needsFullRedraw_ = false;
    return RES_OK;
}

void RWidget::setPosition(int x, int y) { 
    x_ = x; 
    y_ = y; 
}

void RWidget::setSize(uint8_t w, uint8_t h) { 
    width_ = w; 
    height_ = h; 
    setNeedsFullRedraw(); 
}

void RWidget::setNeedsFullRedraw(bool needs) { 
    needsFullRedraw_ = needs; 
    if(needs) needsContentsRedraw_ = true; 
}

void RWidget::setNeedsContentsRedraw(bool needs) { 
    needsContentsRedraw_ = needs; 
}

void RWidget::setBackgroundColor(uint8_t background) {
    if(bgCol_ == background) return;
    bgCol_ = background;
    needsFullRedraw_ = true;
}

void RWidget::setForegroundColor(uint8_t foreground) {
    if(fgCol_ == foreground) return;
    fgCol_ = foreground;
    needsFullRedraw_ = true;
}

void RWidget::setCursorColor(uint8_t cursor) {
    if(cursorCol_ == cursor) return;
    cursorCol_ = cursor;
    needsFullRedraw_ = true;
}

void RWidget::setMarkingColor(uint8_t marking) {
    if(markingCol_ == marking) return;
    markingCol_ = marking;
    needsFullRedraw_ = true;
}

void RWidget::setBorderColor(uint8_t border) {
    if(borderCol_ == border) return;
    borderCol_ = border;
    needsFullRedraw_ = true;
}

void RWidget::setName(const String& name) {
    name_ = name;
}

void RWidget::takeInputFocus() {
Console::console.printfBroadcast("Taking input focus in Base Widget\n");
    RInput::input.clearCallbacks();
}

void RWidget::setAction(std::function<void(void)> cb) {
    action_ = cb;
}

std::function<void(void)> RWidget::action() {
    return action_;
}

void RWidget::triggerAction() {
    if(action_ == nullptr) return;
    action_();
}

