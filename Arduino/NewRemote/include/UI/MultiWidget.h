#if !defined(MULTIWIDGET_H)
#define MULTIWIDGET_H

#include "UI/Widget.h"
#include "UI/WidgetGroup.h"
#include <deque>

using namespace std;

class MultiWidget: public Widget, public WidgetGroup {
public:
    virtual void setNeedsContentsRedraw(bool needs = true);
    virtual void setNeedsFullRedraw(bool needs = true);

    virtual void setPosition(int x, int y);

    virtual Result draw();

    virtual void highlightWidgetsWithTag(int tag);
};

#endif // RMULTIWIDGET_H