#if !defined(ROTAWIDGET_H)
#define ROTAWIDGET_H

#include "UI/MultiWidget.h"
#include "Input.h"

class RotaWidget: public MultiWidget {
public:
    RotaWidget() { widgetsChanged_ = true; }

    virtual Result draw();
    virtual void takeInputFocus();

    void showIndex(unsigned int i);
    void showFirst() { showIndex(0); }
    void showNext() { if(widgets_.size() > 0 && index_ < widgets_.size()-1) showIndex(index_+1); }
    void showPrevious() { if(index_>0) showIndex(index_-1); }

    virtual Widget::CursorHint cursorHint();
    virtual const String& name();
protected:
    unsigned int index_;
};

#endif // RROTAWIDGET_H