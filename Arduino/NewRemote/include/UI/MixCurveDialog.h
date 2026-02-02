#if !defined(MIXCURVEDIALOG_H)
#define MIXCURVEDIALOG_H

#include "MixCurveWidget.h"
#include "MultiWidget.h"
#include "Label.h"

class MixCurveDialog: public MultiWidget {
public:
    MixCurveDialog();

    void takeInputFocus();

    void reposition();

    void focusNext();
    void focusPrev();

    void updateTitle();

    virtual void setOKCallback(std::function<void(MixCurveDialog*)> cb);
    virtual void setCancelCallback(std::function<void(MixCurveDialog*)> cb);

protected:
    shared_ptr<Label> titleLabel_, valueLabel_, cursorLabel_;
    shared_ptr<Label> okLabel_, cancelLabel_;
    shared_ptr<MixCurveWidget> mix_;
    Widget* focusedWidget_;
    float currentEnc_;


    std::function<void(MixCurveDialog*)> okCallback_;
    std::function<void(MixCurveDialog*)> cancelCallback_;

    void encInput(float enc);
    void ok(Widget* w);
    void cancel(Widget* w);
};

#endif // MIXCURVEDIALOG_H