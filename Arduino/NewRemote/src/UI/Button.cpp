#include "UI/Label.h"
#include "UI/Button.h"
#include "UI/WidgetGroup.h"
#include "UI/Display.h"

void ToggleButton::setToggleCallback(std::function<void(ToggleButton*,bool)> cb) {
    toggleCB_ = cb;
}

Result ToggleButton::draw() {
    return Button::draw();
}

void ToggleButton::triggerAction() {
    setOn(!on_);
}

void ToggleButton::setOn(bool on) { 
    if(on_ == on) return;
    if(!on) return; 
    on_ = on; 
    if(toggleCB_ != nullptr) toggleCB_(this, on_);
    setNeedsContentsRedraw();
}

bool ToggleButton::isOn() { return on_; }

void RadioButton::setOn(bool on) {
    if(on_ == on) return;
    if(!on) return; // can't set a toggle button to off - that's done via the group

    ToggleButton::setOn(true);
    for(auto b: group_) b->setOff();
}

Result RadioButton::draw() {
    return Button::draw();
}

void RadioButton::setOff() {
    on_ = false;
    if(toggleCB_ != nullptr) toggleCB_(this, on_);
}

void RadioButton::setRadioGroup(unordered_set<shared_ptr<RadioButton>> group) {
    group_ = group;
}

void MenuButton::triggerAction() {
    if(menu_ != nullptr) {
        menu_->enter();
    }
}
    
void MenuButton::setMenu(const std::shared_ptr<Menu> menu) {
    menu_ = menu;
}

std::shared_ptr<Menu> MenuButton::menu() {
    return menu_;
}
