#include "UI/RotaWidget.h"
#include "Todo/RRemote.h"
#include "UI/UI.h"

void RotaWidget::showIndex(unsigned int i) {
    if(i >= widgets_.size()) return;
    index_ = i;
    UI::ui.setTopTitle(name());
    takeInputFocus();
    widgetsChanged_ = true;
}

Widget::CursorHint RotaWidget::cursorHint() {
    Widget::CursorHint hint = CURSOR_NONE;
    if(index_ != 0 && widgets_.size() > 1) hint = (Widget::CursorHint)(hint|CURSOR_LEFT);
    if(index_ < widgets_.size()-1) hint = (Widget::CursorHint)(hint|CURSOR_RIGHT);
    return hint;
}

Result RotaWidget::draw() {
    if(widgets_.size() == 0) return RES_OK;
    if(index_ >= widgets_.size()) index_ = widgets_.size()-1;

    if(widgetsChanged_) {
        widgets_[index_]->setNeedsFullRedraw();
        widgetsChanged_ = false;
    }
    return widgets_[index_]->draw();
}

void RotaWidget::takeInputFocus() {
    Input::inst.setLeftShortPressCallback([this]{ showPrevious(); });
    Input::inst.setRightShortPressCallback([this]{ showNext(); });

    if(widgets_.size() == 0 || index_ >= widgets_.size()) return;
    widgets_[index_]->takeInputFocus();
}

const String& RotaWidget::name() {
    if(widgets_.size() == 0 || index_ >= widgets_.size()) return name_;
    return widgets_[index_]->name();
}
