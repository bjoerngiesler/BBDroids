#include "UI/MixCurveDialog.h"    
#include "UI/UI.h"

MixCurveDialog::MixCurveDialog() {    
    titleLabel_ = make_shared<Label>();
    mix_ = make_shared<MixCurveWidget>();
    okLabel_ = make_shared<Label>();
    cancelLabel_ = make_shared<Label>();
    okLabel_->setAutosize();
    okLabel_->setTitle("OK");
    okLabel_->setDrawsFrame();
    cancelLabel_->setAutosize();
    cancelLabel_->setTitle("Cancel");
    cancelLabel_->setDrawsFrame();

    okLabel_->setAction([this](Widget* w) { ok(w); });
    cancelLabel_->setAction([this](Widget* w) { cancel(w); });

    addWidget(titleLabel_);
    addWidget(mix_);
    addWidget(okLabel_);
    addWidget(cancelLabel_);
    reposition();

    focusedWidget_ = mix_.get();
    focusedWidget_->setHighlighted(true);

    titleLabel_->setTitle("Mix");

    setDrawsFrame();
    setFillsBackground();

    okCallback_ = nullptr;
    cancelCallback_ = nullptr;
    currentEnc_ = 0;
}

void MixCurveDialog::takeInputFocus() {
    focusedWidget_ = mix_.get();
    focusedWidget_->setHighlighted(true);
    mix_->setCursor(0);
    mix_->setShowsCursor(true);

    updateTitle();

    Input::inst.clearCallbacks();
    Input::inst.setConfirmShortPressCallback([this](){focusedWidget_->triggerAction();});
    Input::inst.setLeftPressCallback([this](){focusPrev();});
    Input::inst.setRightPressCallback([this](){focusNext();});
    Input::inst.setEncTurnCallback([this](float enc){encInput(enc);});
}

void MixCurveDialog::focusNext() {
    if(focusedWidget_ == mix_.get()) {
        if(mix_->cursor() < 4) {
            mix_->setCursor(mix_->cursor()+1);
            mix_->setShowsCursor();
            updateTitle();

            return;
        }
        mix_->setHighlighted(false);
        mix_->setShowsCursor(false);
        focusedWidget_ = okLabel_.get();
    } else if(focusedWidget_ == okLabel_.get()) {
        focusedWidget_->setHighlighted(false);
        focusedWidget_ = cancelLabel_.get();
    } else {
        focusedWidget_->setHighlighted(false);
        focusedWidget_ = mix_.get();
        mix_->setCursor(0);
        mix_->setShowsCursor();
    }

    focusedWidget_->setHighlighted(true);
    updateTitle();
}

void MixCurveDialog::focusPrev() {
    if(focusedWidget_ == mix_.get()) {
        if(mix_->cursor() > 0) {
            mix_->setCursor(mix_->cursor()-1);
            mix_->setShowsCursor();
            updateTitle();

            return;
        }
        mix_->setHighlighted(false);
        mix_->setShowsCursor(false);
        focusedWidget_ = cancelLabel_.get();
    } else if(focusedWidget_ == cancelLabel_.get()) {
        focusedWidget_->setHighlighted(false);
        focusedWidget_ = okLabel_.get();
    } else {
        focusedWidget_->setHighlighted(false);
        focusedWidget_ = mix_.get();
        mix_->setCursor(4);
        mix_->setShowsCursor();
    }
    focusedWidget_->setHighlighted(true);
    updateTitle();
}

void MixCurveDialog::setOKCallback(std::function<void(MixCurveDialog*)> cb) {
    okCallback_ = cb;
}

void MixCurveDialog::setCancelCallback(std::function<void(MixCurveDialog*)> cb) {
    cancelCallback_ = cb;
}

void MixCurveDialog::reposition() {
    setSize(Display::MAIN_WIDTH-4, Display::MAIN_HEIGHT-20);
    centerOnMain();

    titleLabel_->setPosition(x()+1, y()+1);
    titleLabel_->setSize(width_-2, Display::CHAR_HEIGHT+2);

    mix_->setSize(width_-4, width_-4);
    mix_->setPosition(4, titleLabel_->y()+titleLabel_->height()+2);
    
    okLabel_->setPosition(x() + 2, y() + height() - okLabel_->height() - 2);
    cancelLabel_->setPosition(x() + width() - cancelLabel_->width() - 2, okLabel_->y());
}

void MixCurveDialog::encInput(float enc) {
    if(focusedWidget_ != mix_.get()) return;

    Interpolator inter = mix_->interpolator();
    int8_t v;
    switch(mix_->cursor()) {
        case 0: v = inter.i0; break;
        case 1: v = inter.i25; break;
        case 2: v = inter.i50; break;
        case 3: v = inter.i75; break;
        case 4: default: v = inter.i100; break;
    }

    currentEnc_ += enc;
    while(currentEnc_ > 10.0) {
        v++;
        if(v > 125) v = 125;
        currentEnc_ -= 10.0;
    }
    while(currentEnc_ < -10.0) {
        v--;
        if(v < -125) v = -125;
        currentEnc_ += 10.0;
    }

    switch(mix_->cursor()) {
        case 0: inter.i0 = v; break;
        case 1: inter.i25 = v; break;
        case 2: inter.i50 = v; break;
        case 3: inter.i75 = v; break;
        case 4: inter.i100 = v; break;
    }
    mix_->setInterpolator(inter);
    updateTitle();
}


void MixCurveDialog::ok(Widget* w) {
    if(okCallback_ != nullptr) okCallback_(this);
    UI::ui.hideDialog();
}

void MixCurveDialog::cancel(Widget* w) {
    if(cancelCallback_ != nullptr) cancelCallback_(this);
    UI::ui.hideDialog();
}

void MixCurveDialog::updateTitle() {
    if(focusedWidget_ != mix_.get()) {
        titleLabel_->setTitle("Mix");
        return;
    }

    char buf[255];
    Interpolator interp = mix_->interpolator();

    switch(mix_->cursor()) {
    case 0:
        sprintf(buf, "Mix 0%%>%d%%", interp.i0);
        break;
    case 1:
        sprintf(buf, "Mix 25%%>%d%%", interp.i25);
        break;
    case 2:
        sprintf(buf, "Mix 50%%>%d%%", interp.i50);
        break;
    case 3:
        sprintf(buf, "Mix 75%%>%d%%", interp.i75);
        break;
    case 4: default: 
        sprintf(buf, "Mix 100%%>%d%%", interp.i100);
        break;
    }
    titleLabel_->setTitle(buf);
}
