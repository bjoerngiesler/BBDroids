#include "UI/WidgetGroup.h"

bool WidgetGroup::addWidget(const shared_ptr<Widget>& w) {
    if(std::find(widgets_.begin(), widgets_.end(), w) != widgets_.end()) {
        return false;
    }
    widgets_.push_back(w);
    return true;
}

bool WidgetGroup::removeWidget(const shared_ptr<Widget>& w) {
    std::deque<shared_ptr<Widget>>::iterator iter = std::find(widgets_.begin(), widgets_.end(), w);
    if(iter == widgets_.end()) {
        return false;
    }
    widgets_.erase(iter);
    return true;
}

bool WidgetGroup::hasWidget(const shared_ptr<Widget>& w) {
    return (std::find(widgets_.begin(), widgets_.end(), w) != widgets_.end());
}

