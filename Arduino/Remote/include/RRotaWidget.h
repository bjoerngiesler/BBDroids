#if !defined(RROTAWIDGET_H)
#define RROTAWIDGET_H

#include "RMultiWidget.h"
#include "RInput.h"

class RRotaWidget: public RMultiWidget {
public:
    RRotaWidget() { widgetsChanged_ = true; }

    virtual Result draw(ConsoleStream *stream);
    virtual void takeInputFocus();

    void next();
    void previous();

    virtual RWidget::CursorHint cursorHint();
    virtual const String& name();
protected:
    int index_;
};

#endif // RROTAWIDGET_H