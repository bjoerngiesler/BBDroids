#if !defined(WIDGETGROUP_H)
#define WIDGETGROUP_H

#include <memory>
#include "Widget.h"

using namespace std;

class WidgetGroup {
public:
    bool addWidget(const shared_ptr<Widget>& w);
    bool removeWidget(const shared_ptr<Widget>& w);
    bool hasWidget(const shared_ptr<Widget>& w);
    void clearWidgets() { widgets_.clear(); }
protected:
    deque<shared_ptr<Widget>> widgets_;
    bool widgetsChanged_;
}; 

#endif // WIDGETGROUP_H