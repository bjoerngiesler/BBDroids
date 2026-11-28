#include "UI/MixCurveDialog.h"
#include "UI/UI.h"

static const int SMALL_STEP = 5, LARGE_STEP = 25;
static const float ENC_STEP = 10.0;

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
    titleLabel_->setTitle("Mix");

    setDrawsFrame();
    setFillsBackground();

    okCallback_ = nullptr;
    cancelCallback_ = nullptr;
    currentEnc_ = 0;
}

void MixCurveDialog::setInterpolator(const Interpolator& inter) {
    mix_->setInterpolator(inter);
    updateTitle();
}

void MixCurveDialog::takeInputFocus() {
    okLabel_->setHighlighted(false);
    cancelLabel_->setHighlighted(false);
    focusedWidget_ = mix_.get();
    mix_->setHighlighted(true);
    mix_->setCursor(0);
    mix_->setEditing(false);
    mix_->setShowsCursor(true);
    currentEnc_ = 0;
    updateTitle();

    Input::inst.clearCallbacks();
    Input::inst.setConfirmShortPressCallback([this](){confirm();});
    Input::inst.setLeftShortPressCallback([this](){ if(mix_->editing()) changeValue(-SMALL_STEP); else focusPrev(); });
    Input::inst.setRightShortPressCallback([this](){ if(mix_->editing()) changeValue(SMALL_STEP); else focusNext(); });
    Input::inst.setLeftLongPressCallback([this](){ if(mix_->editing()) changeValue(-LARGE_STEP); });
    Input::inst.setRightLongPressCallback([this](){ if(mix_->editing()) changeValue(LARGE_STEP); });
    Input::inst.setEncTurnCallback([this](float enc){encInput(enc);});
}

void MixCurveDialog::confirm() {
    if(focusedWidget_ == mix_.get()) {
        mix_->setEditing(!mix_->editing());
        currentEnc_ = 0;
        updateTitle();
    } else {
        focusedWidget_->triggerAction();
    }
}

// Order: P0..P4 -> OK -> Cancel -> P0
void MixCurveDialog::focusNext() {
    if(focusedWidget_ == mix_.get()) {
        if(mix_->cursor() < MixCurveWidget::NUM_POINTS-1) {
            mix_->setCursor(mix_->cursor()+1);
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
        mix_->setCursor(MixCurveWidget::NUM_POINTS-1);
        mix_->setShowsCursor();
    }
    focusedWidget_->setHighlighted(true);
    updateTitle();
}

void MixCurveDialog::changeValue(int delta) {
    if(focusedWidget_ != mix_.get()) return;
    mix_->setPointValue(mix_->cursor(), mix_->pointValue(mix_->cursor()) + delta);
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

    titleLabel_->setPosition(x()+2, y()+2);
    titleLabel_->setSize(width_-4, Display::CHAR_HEIGHT+2);

    okLabel_->setPosition(x() + 2, y() + height() - okLabel_->height() - 2);
    cancelLabel_->setPosition(x() + width() - cancelLabel_->width() - 2, okLabel_->y());

    int curveY = titleLabel_->y() + titleLabel_->height() + 2;
    mix_->setPosition(x()+2, curveY);
    mix_->setSize(width_-4, okLabel_->y() - curveY - 2);
}

void MixCurveDialog::encInput(float enc) {
    if(focusedWidget_ != mix_.get() || !mix_->editing()) return;

    currentEnc_ += enc;
    int delta = 0;
    while(currentEnc_ > ENC_STEP) { delta++; currentEnc_ -= ENC_STEP; }
    while(currentEnc_ < -ENC_STEP) { delta--; currentEnc_ += ENC_STEP; }
    if(delta != 0) changeValue(delta);
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
        titleLabel_->setTitle("Mix curve");
        return;
    }

    static const char* inPct[] = {"0", "25", "50", "75", "100"};
    char buf[32];
    uint8_t c = mix_->cursor();
    if(mix_->editing()) snprintf(buf, sizeof(buf), "%s>[%d]", inPct[c], mix_->pointValue(c));
    else snprintf(buf, sizeof(buf), "%s>%d%%", inPct[c], mix_->pointValue(c));
    titleLabel_->setTitle(buf);
}
