#if !defined(RROTAWIDGET_H)
#define RROTAWIDGET_H

#include "RMultiWidget.h"
#include "RInput.h"

class RRotaWidget: public RMultiWidget {
public:
    RRotaWidget() { widgetsChanged_ = true; }

    virtual Result draw(ConsoleStream *stream);
    virtual void takeInputFocus();

    void showIndex(unsigned int i);
    void showFirst() { showIndex(0); }
    void showNext() { if(widgets_.size() > 0 && index_ < widgets_.size()-1) showIndex(index_+1); }
    void showPrevious() { if(index_>0) showIndex(index_-1); }

    virtual RWidget::CursorHint cursorHint();
    virtual const String& name();
protected:
    unsigned int index_;
};

#endif // RROTAWIDGET_H