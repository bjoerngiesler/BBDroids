#if !defined(MIXCURVEDIALOG_H)
#define MIXCURVEDIALOG_H

#include "MixCurveWidget.h"
#include "MultiWidget.h"
#include "Label.h"

//! Dialog for editing an Interpolator.
//!
//! Navigation: Left/Right move through the points P0..P4, then OK and Cancel. Confirm on a point enters edit
//! mode (point drawn yellow): Left/Right short press changes the value by -/+5, long press by -/+25, the encoder
//! by 1. Confirm leaves edit mode. Confirm on OK/Cancel closes the dialog.
class MixCurveDialog: public MultiWidget {
public:
    MixCurveDialog();

    void setInterpolator(const Interpolator& inter);
    Interpolator interpolator() { return mix_->interpolator(); }

    virtual void takeInputFocus();

    virtual void setOKCallback(std::function<void(MixCurveDialog*)> cb);
    virtual void setCancelCallback(std::function<void(MixCurveDialog*)> cb);

protected:
    void reposition();
    void focusNext();
    void focusPrev();
    void confirm();
    void changeValue(int delta);
    void updateTitle();
    void encInput(float enc);
    void ok(Widget* w);
    void cancel(Widget* w);

    shared_ptr<Label> titleLabel_;
    shared_ptr<Label> okLabel_, cancelLabel_;
    shared_ptr<MixCurveWidget> mix_;
    Widget* focusedWidget_;
    float currentEnc_;

    std::function<void(MixCurveDialog*)> okCallback_;
    std::function<void(MixCurveDialog*)> cancelCallback_;
};

#endif // MIXCURVEDIALOG_H
