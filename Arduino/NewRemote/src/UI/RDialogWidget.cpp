#include "UI/RDialogWidget.h"
#include "UI/RUI.h"
#include "Todo/RRemote.h"

RDialogWidget::RDialogWidget() {
    titleLabel_.setAutosize();
    titleLabel_.setLinebreak();
    valueLabel_.setAutosize();
    valueLabel_.setForegroundColor(RDisplay::BLACK);
    valueLabel_.setBackgroundColor(RDisplay::WHITE);
    addWidget(&titleLabel_);
    addWidget(&valueLabel_);
    min_ = 0; max_ = 10; value_ = 5;
    setTitle("Dialog");
    setRange(0, 10);
    setValue(5);
    setSuffix("");

    setDrawsFrame();
    setFillsBackground();

    okCallback_ = nullptr;
    cancelCallback_ = nullptr;
}

void RDialogWidget::setTitle(const String& title) {
    titleLabel_.setTitle(title);
    reposition();
}

void RDialogWidget::setValue(int value) {
    value_ = value;
    valueLabel_.setTitle(String(value_) + suffix_);
}

void RDialogWidget::setSuffix(const String& s) {
    suffix_ = s;
    valueLabel_.setTitle(String(value_) + suffix_);
}

void RDialogWidget::setRange(int min, int max) {
    min_ = min; max_ = max;
}

void RDialogWidget::takeInputFocus() {
    RInput::input.setConfirmShortPressCallback([=](){ok();});
    RInput::input.setLeftPressCallback([=](){cancel();});
    RInput::input.setEncTurnCallback([=](float enc){encInput(enc);});
}

void RDialogWidget::setOKCallback(std::function<void(int)> cb) {
    okCallback_ = cb;
}

void RDialogWidget::setCancelCallback(std::function<void(void)> cb) {
    cancelCallback_ = cb;
}

void RDialogWidget::reposition() {
    uint8_t w = titleLabel_.width();
    if(w < valueLabel_.width()) w = valueLabel_.width();
    uint8_t h = titleLabel_.height() + valueLabel_.height();

    setSize(w+4, h+4);
    centerOnMain();
    titleLabel_.setPosition(x() + (width()-titleLabel_.width())/2, y() + 2);
    valueLabel_.setPosition(x() + (width()-valueLabel_.width())/2, y() + titleLabel_.height() + 2);
}

void RDialogWidget::encInput(float enc) {
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
}


void RDialogWidget::ok() {
    if(okCallback_ != nullptr) okCallback_(value_);
    RUI::ui.hideDialog();
}

void RDialogWidget::cancel() {
    if(cancelCallback_ != nullptr) cancelCallback_();
    RUI::ui.hideDialog();
}