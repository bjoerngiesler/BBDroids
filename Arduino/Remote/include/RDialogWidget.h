#if !defined(RDIALOGWIDGET_H)
#define RDIALOGWIDGET_H

#include "RMultiWidget.h"
#include "RLabelWidget.h"

class RDialogWidget: public RMultiWidget {
public:
    RDialogWidget();
    virtual void setTitle(const String& title);
    virtual void setValue(int value);
    virtual void setRange(int max, int min);
    virtual void setSuffix(const String& suffix);
    virtual void takeInputFocus();
    virtual void setOKCallback(std::function<void(int)> cb);
    virtual void setCancelCallback(std::function<void(void)> cb);

protected:
    RLabelWidget titleLabel_, valueLabel_;    
    void reposition();
    void encInput(float enc);
    void ok();
    void cancel();
    int value_, max_, min_;
    float currentEnc_;
    std::function<void(int)> okCallback_;
    std::function<void(void)> cancelCallback_;
    String suffix_;
};

#endif // RDIALOGWIDGET_H