#if !defined(RMULTIWIDGET_H)
#define RMULTIWIDGET_H

#include "RWidget.h"
#include <deque>

class RMultiWidget: public RWidget {
public:
    bool addWidget(RWidget* w);
    bool removeWidget(RWidget* w);
    bool hasWidget(RWidget* w);
    void clearWidgets() { widgets_.clear(); }

    virtual void setNeedsContentsRedraw(bool needs = true);
    virtual void setNeedsFullRedraw(bool needs = true);

    virtual void setPosition(int x, int y);

    virtual Result draw(ConsoleStream* stream = NULL);
protected:
    std::deque<RWidget*> widgets_;
    bool widgetsChanged_;
};

#endif // RMULTIWIDGET_H