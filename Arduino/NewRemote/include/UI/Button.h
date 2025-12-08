#if !defined(BUTTON_H)
#define BUTTON_H

#include "UI/Label.h"
#include "UI/WidgetGroup.h"
#include "UI/Menu.h"

#include <unordered_set>
#include <memory>

class Button: virtual public Label {
public:
protected:
};

class ToggleButton: virtual public Button {
public:
    void setToggleCallback(std::function<void(ToggleButton*,bool)> cb);
    virtual Result draw();

    virtual void triggerAction();

    virtual void setOn(bool on);
    virtual bool isOn();
protected:
    std::function<void(ToggleButton*,bool)> toggleCB_ = nullptr;
    bool on_;
};

class RadioButton: virtual public ToggleButton {
public:
    virtual Result draw();
   
    void setRadioGroup(unordered_set<shared_ptr<RadioButton>> group);

    virtual void setOn(bool on);
protected:
    void setOff();
    unordered_set<shared_ptr<RadioButton>> group_;
};

class Menu;

class MenuButton: virtual public Button {
public:
    virtual void triggerAction();
    void setMenu(const std::shared_ptr<Menu> menu);
    std::shared_ptr<Menu> menu();
protected:
    std::shared_ptr<Menu> menu_;
};

#endif // BUTTON_H