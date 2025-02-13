#include "RWidget.h"

RWidget::RWidget(): name_("Widget") { 
    needsFullRedraw_ = needsContentsRedraw_ = true; 
    x_ = y_ = 1;
    width_ = height_ = 10;
    frameCol_ = RDisplay::WHITE;
    bgCol_ = RDisplay::BLACK;
    fgCol_ = RDisplay::WHITE;
    hlCol_ = RDisplay::LIGHTBLUE2;
    cursorCol_ = RDisplay::LIGHTGREEN1;
    markingCol_ = RDisplay::BLUE;
    fillsBg_ = false;
    drawsFrame_ = false;
    action_ = nullptr;
    highlighted_ = false;
    tag_ = 0;
}

RWidget::~RWidget() {
}

Result RWidget::draw(ConsoleStream* stream) {
    needsContentsRedraw_ = false;
    needsFullRedraw_ = false;
    return RES_OK;
}

void RWidget::setPosition(int x, int y) { 
    if(x == x_ && y == y_) return;
    x_ = x; 
    y_ = y; 
    setNeedsFullRedraw();
}

void RWidget::setSize(uint8_t w, uint8_t h) { 
    if(w == width_ && h == height_) return;
    width_ = w; 
    height_ = h; 
    setNeedsFullRedraw(); 
}

void RWidget::centerOnDisplay() {
    setPosition(RDisplay::MAIN_X + (RDisplay::DISPLAY_WIDTH-width())/2, 
                RDisplay::MAIN_Y + (RDisplay::DISPLAY_HEIGHT-height())/2);
}

void RWidget::centerOnMain() {
    setPosition(RDisplay::MAIN_X + (RDisplay::MAIN_WIDTH-width())/2, 
                RDisplay::MAIN_Y + (RDisplay::MAIN_HEIGHT-height())/2);
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

void RWidget::setFrameColor(uint8_t frame) {
    if(frameCol_ == frame) return;
    frameCol_ = frame;
    needsFullRedraw_ = true;
}

void RWidget::setHighlightColor(uint8_t highlight) {
    if(hlCol_ == highlight) return;
    hlCol_ = highlight;
    needsFullRedraw_ = true;
}

void RWidget::setHighlighted(bool yesno) {
    if(yesno == highlighted_) return;
    highlighted_ = yesno;
    needsFullRedraw_ = true;
}

void RWidget::setName(const String& name) {
    name_ = name;
}

void RWidget::takeInputFocus() {
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

