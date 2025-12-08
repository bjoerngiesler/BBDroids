#if !defined(DIALOG_H)
#define DIALOG_H

#include "UI/MultiWidget.h"
#include "UI/Label.h"

using namespace std;

class Dialog: public MultiWidget {
public:
    enum DialogType {
        INTEGER,
        FLOAT,
        TEXT
    };

    Dialog();
    virtual void setTitle(const String& title);

    virtual void setValue(const String& value);
    virtual void setMaxLen(unsigned int maxlen);
    
    virtual void setValue(int value);
    virtual void setRange(int max, int min);
    
    virtual void setSuffix(const String& suffix);
    virtual void takeInputFocus();
    virtual void setOKCallback(std::function<void(Dialog*)> cb);
    virtual void setCancelCallback(std::function<void(Dialog*)> cb);

    const String& valueString() { return valueString_; }
    int value() { return value_; }

protected:
    String cursorString();
    String makeValueString(const String& v);
    void focusNext(), focusPrev();

    DialogType type_;
    shared_ptr<Label> titleLabel_, valueLabel_, cursorLabel_;
    shared_ptr<Label> okLabel_, cancelLabel_;
    void reposition();
    void encInput(float enc);
    void ok(Widget* w);
    void cancel(Widget* w);
    int value_, max_, min_;
    String valueString_;
    float currentEnc_;
    std::function<void(Dialog*)> okCallback_;
    std::function<void(Dialog*)> cancelCallback_;
    String suffix_;

    unsigned int cursor_, maxlen_;

    Widget* focusedWidget_;
};

#endif // DIALOG_H