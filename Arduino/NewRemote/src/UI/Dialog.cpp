#include "UI/Dialog.h"
#include "UI/UI.h"
#include "Todo/RRemote.h"

using namespace std;

Dialog::Dialog() {
    titleLabel_ = make_shared<Label>();
    valueLabel_ = make_shared<Label>();
    cursorLabel_ = make_shared<Label>();
    okLabel_ = make_shared<Label>();
    cancelLabel_ = make_shared<Label>();
    okLabel_->setAutosize();
    okLabel_->setTitle("OK");
    okLabel_->setDrawsFrame();
    cancelLabel_->setAutosize();
    cancelLabel_->setTitle("Cancel");
    cancelLabel_->setDrawsFrame();

    titleLabel_->setAutosize();
    titleLabel_->setLinebreak();
    valueLabel_->setAutosize();
    cursorLabel_->setAutosize();

    okLabel_->setAction([this](Widget* w) { ok(w); });
    cancelLabel_->setAction([this](Widget* w) { cancel(w); });

    addWidget(titleLabel_);
    addWidget(valueLabel_);
    addWidget(cursorLabel_);
    addWidget(okLabel_);
    addWidget(cancelLabel_);
    reposition();

    focusedWidget_ = valueLabel_.get();
    focusedWidget_->setHighlighted(true);

    min_ = 0; max_ = 10; value_ = 5;
    maxlen_ = 10;
    setTitle("Dialog");
    setRange(0, 10);
    setValue(5);
    setSuffix("");

    setDrawsFrame();
    setFillsBackground();

    okCallback_ = nullptr;
    cancelCallback_ = nullptr;
    type_ = INTEGER;

}

void Dialog::setTitle(const String& title) {
    titleLabel_->setTitle(title);
    reposition();
}

void Dialog::setValue(int value) {
    type_ = INTEGER;
    if(value < min_) value = min_;
    if(value > max_) value = max_;
    value_ = value;
    valueLabel_->setTitle(String(value_) + suffix_);
}

void Dialog::setValue(const String& value) {
    bb::printf("setValue()\n");
    delay(1);
    type_ = TEXT;
    delay(1);
    if(maxlen_ == 0) maxlen_ = value.length();
    delay(1);
    makeValueString(value);
    delay(1);
    valueLabel_->setTitle(String("[") + valueString_ + "]");
    delay(1);
    cursor_ = 0;
    delay(1);
    cursorLabel_->setTitle(cursorString());
    delay(1);
    reposition();
    delay(1);
    bb::printf("setValue() done\n");
}

void Dialog::setMaxLen(unsigned int maxlen) {
    bb::printf("setMaxLen()\n");
    type_ = TEXT;
    maxlen_ = maxlen;
    cursor_ = 0;
    makeValueString(valueString_);
    cursorLabel_->setTitle(cursorString());
    valueLabel_->setTitle(String("[") + valueString_ + "]");
    reposition();
    bb::printf("setMaxLen() done\n");
}

void Dialog::setRange(int min, int max) {
    min_ = min; max_ = max;
    if(value_ < min_) value_ = min_;
    if(value_ > max_) value_ = max_;
    reposition();
}

void Dialog::setSuffix(const String& s) {
    suffix_ = s;
    valueLabel_->setTitle(String(value_) + suffix_);
}

void Dialog::takeInputFocus() {
    focusedWidget_ = valueLabel_.get();
    focusedWidget_->setHighlighted(true);
    cursorLabel_->setHighlighted(true);
    cursor_ = 0;
    Input::inst.setConfirmShortPressCallback([this](){focusedWidget_->triggerAction();});
    Input::inst.setLeftPressCallback([this](){focusPrev();});
    Input::inst.setRightPressCallback([this](){focusNext();});
    Input::inst.setEncTurnCallback([this](float enc){encInput(enc);});
}

void Dialog::focusNext() {
    focusedWidget_->setHighlighted(false);
    if(focusedWidget_ == nullptr || focusedWidget_ == cancelLabel_.get()) {
        focusedWidget_ = valueLabel_.get();
        cursor_ = 0;
        cursorLabel_->setTitle(cursorString());
        cursorLabel_->setHighlighted();
    } else if(focusedWidget_ == valueLabel_.get()) {
        if(cursor_ < maxlen_-1) {
            cursor_++;
            cursorLabel_->setTitle(cursorString());
            cursorLabel_->setHighlighted();
        } else {
            focusedWidget_ = okLabel_.get();
            cursorLabel_->setHighlighted(false);
        }
    } else if(focusedWidget_ == okLabel_.get()) {
        focusedWidget_ = cancelLabel_.get();
        cursorLabel_->setHighlighted(false);
    } else {
        focusedWidget_ = valueLabel_.get();
        cursorLabel_->setHighlighted();
    }
    focusedWidget_->setHighlighted();
}

void Dialog::focusPrev() {
    focusedWidget_->setHighlighted(false);
    if(focusedWidget_ == nullptr || focusedWidget_ == valueLabel_.get()) {
        if(cursor_ > 0) {
            cursor_--;
            cursorLabel_->setTitle(cursorString());
            cursorLabel_->setHighlighted();
        } else {
            focusedWidget_ = cancelLabel_.get();
            cursorLabel_->setHighlighted(false);
        }
    } else if(focusedWidget_ == cancelLabel_.get()) {
        focusedWidget_ = okLabel_.get();
        cursorLabel_->setHighlighted(false);
    } else {
        focusedWidget_ = valueLabel_.get();
        cursor_ = maxlen_-1;
        cursorLabel_->setTitle(cursorString());
        cursorLabel_->setHighlighted(true);
    }
    focusedWidget_->setHighlighted(true);
}

void Dialog::setOKCallback(std::function<void(Dialog*)> cb) {
    okCallback_ = cb;
}

void Dialog::setCancelCallback(std::function<void(Dialog*)> cb) {
    cancelCallback_ = cb;
}

void Dialog::reposition() {
    uint8_t w = okLabel_->width() + cancelLabel_->width() + 2;
    if(w < titleLabel_->width()+6) w = titleLabel_->width() + 2;
    if(w < valueLabel_->width()+6) w = valueLabel_->width() + 2;
    if(w < cursorLabel_->width()+6) w = valueLabel_->width() + 2;

    uint8_t h = titleLabel_->height() + valueLabel_->height() + cursorLabel_->height() + okLabel_->height() + 10;

    setSize(w, h);
    centerOnMain();

    titleLabel_->setPosition(x() + (width()-titleLabel_->width())/2, y() + 2);
    valueLabel_->setPosition(x() + (width()-valueLabel_->width())/2, titleLabel_->y() + titleLabel_->height() + 2);
    cursorLabel_->setPosition(x() + (width()-cursorLabel_->width())/2, valueLabel_->y() + valueLabel_->height() + 2);
    
    okLabel_->setPosition(x() + 2, y() + height() - okLabel_->height() - 2);
    bb::printf("moving cancellabel to %d+%d-%d-3, %d\n", x(), width(), cancelLabel_->width(), okLabel_->y());
    cancelLabel_->setPosition(x() + width() - cancelLabel_->width() - 2, okLabel_->y());
}

String Dialog::cursorString() {
    char buf[maxlen_ + 1];
    memset(buf, 0, maxlen_+1);
    for(unsigned int i=0; i<maxlen_; i++) {
        if(i == cursor_) buf[i] = '^';
        else buf[i] = ' ';  
    }
    return String(buf);
}

String Dialog::makeValueString(const String& v) {
    bb::printf("making Value String from '%s' (maxlen is %d)\n", v.c_str(), maxlen_);
    delay(1);
    char buf[maxlen_ + 1];
    unsigned int len = v.length();
    memset(buf, 0, maxlen_+1);
    for(int i=0; i<maxlen_; i++) {
        if(i < len) buf[i] = v.c_str()[i];
        else buf[i] = ' ';
    }
    valueString_ = buf;
    bb::printf("Value string set to '%s'\n", valueString_.c_str());
    delay(1);
    return valueString_;
}

void Dialog::encInput(float enc) {
    if(type_ == INTEGER) {
        currentEnc_ += enc;
        int v = value_;
        while(currentEnc_ > 50.0) {
            v--;
            if(v < min_) v = min_;
            currentEnc_ -= 50.0;
        }
        while(currentEnc_ < -50.0) {
            v++;
            if(v > max_) v = max_;
            currentEnc_ += 50.0;
        }
        setValue(v);
    } else if(type_ == TEXT) {
        currentEnc_ += enc;
        unsigned char c = valueString_[cursor_];
        if(currentEnc_ > 20.0) {
            c = c+1;
            if(c > '9' && c < 'A') c = 'A';
            if(c > 'Z' && c < 'a') c = 'a';
            if(c > ' ' && c < '0') c = '0';
            if(c > 'z') c = ' ';
            currentEnc_ = 0;
        } else if(currentEnc_ < -20.0) {
            c = c-1;
            if(c < 'A' && c > '9') c = '9';
            if(c < 'a' && c > 'Z') c = 'Z';
            if(c < '0' && c > ' ') c = ' ';
            if(c < ' ') c = 'z'; 
            currentEnc_ = 0;
        }
        valueString_[cursor_] = c;
        valueLabel_->setTitle(String("[") + valueString_ + "]");
    }
}


void Dialog::ok(Widget* w) {
    if(okCallback_ != nullptr) okCallback_(this);
    UI::ui.hideDialog();
}

void Dialog::cancel(Widget* w) {
    if(cancelCallback_ != nullptr) cancelCallback_(this);
    UI::ui.hideDialog();
}