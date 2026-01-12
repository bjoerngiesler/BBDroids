#include "UI/Widget.h"

Widget::Widget(): name_("Widget") { 
    needsFullRedraw_ = needsContentsRedraw_ = true; 
    x_ = y_ = 1;
    width_ = height_ = 10;
    frameCol_ = Display::WHITE;
    bgCol_ = Display::BLACK;
    fgCol_ = Display::WHITE;
    hlCol_ = Display::LIGHTBLUE2;
    cursorCol_ = Display::LIGHTGREEN1;
    markingCol_ = Display::BLUE;
    fillsBg_ = false;
    drawsFrame_ = false;
    action_ = nullptr;
    highlighted_ = false;
    tag_ = 0;
}

Widget::~Widget() {
    bb::printf("Deleting widget 0x%x \"%s\"\n", this, name_.c_str());
    bb::printf("Free heap now: %d\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
}

Result Widget::draw() {
    needsContentsRedraw_ = false;
    needsFullRedraw_ = false;
    return RES_OK;
}

void Widget::setPosition(int x, int y) { 
    if(x == x_ && y == y_) return;
    x_ = x; 
    y_ = y; 
    setNeedsFullRedraw();
}

void Widget::setSize(uint8_t w, uint8_t h) { 
    if(w == width_ && h == height_) return;
    width_ = w; 
    height_ = h; 
    setNeedsFullRedraw(); 
}

void Widget::centerOnDisplay() {
    setPosition(Display::MAIN_X + (Display::DISPLAY_WIDTH-width())/2, 
                Display::MAIN_Y + (Display::DISPLAY_HEIGHT-height())/2);
}

void Widget::centerOnMain() {
    setPosition(Display::MAIN_X + (Display::MAIN_WIDTH-width())/2, 
                Display::MAIN_Y + (Display::MAIN_HEIGHT-height())/2);
}

void Widget::setNeedsFullRedraw(bool needs) { 
    needsFullRedraw_ = needs; 
    if(needs) needsContentsRedraw_ = true; 
}

void Widget::setNeedsContentsRedraw(bool needs) { 
    needsContentsRedraw_ = needs; 
}

void Widget::setBackgroundColor(uint8_t background) {
    if(bgCol_ == background) return;
    bgCol_ = background;
    needsFullRedraw_ = true;
}

void Widget::setForegroundColor(uint8_t foreground) {
    if(fgCol_ == foreground) return;
    fgCol_ = foreground;
    needsFullRedraw_ = true;
}

void Widget::setCursorColor(uint8_t cursor) {
    if(cursorCol_ == cursor) return;
    cursorCol_ = cursor;
    needsFullRedraw_ = true;
}

void Widget::setMarkingColor(uint8_t marking) {
    if(markingCol_ == marking) return;
    markingCol_ = marking;
    needsFullRedraw_ = true;
}

void Widget::setFrameColor(uint8_t frame) {
    if(frameCol_ == frame) return;
    frameCol_ = frame;
    needsFullRedraw_ = true;
}

void Widget::setHighlightColor(uint8_t highlight) {
    if(hlCol_ == highlight) return;
    hlCol_ = highlight;
    needsFullRedraw_ = true;
}

void Widget::setHighlighted(bool yesno) {
    if(yesno == highlighted_) return;
    highlighted_ = yesno;
    needsFullRedraw_ = true;
}

void Widget::setName(const String& name) {
    name_ = name;
}

void Widget::takeInputFocus() {
}

void Widget::setAction(std::function<void(Widget*)> cb) {
    action_ = cb;
}

std::function<void(Widget*)> Widget::action() {
    return action_;
}

void Widget::triggerAction() {
    if(action_ == nullptr) return;
    action_(this);
}

