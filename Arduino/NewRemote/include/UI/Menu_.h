#if !defined(MENU_H)
#define MENU_H

#include "Widget.h"

class Menu_; // forward declaration

class MenuItem_: public Widget {
public:
    MenuItem(std::string label, std::function<void(const MenuItem&)> action = nullptr);
    MenuItem(std::string label, std::unique_ptr<Menu> submenu);

    bool hasSubmenu() const { return submenu != nullptr; }
    void trigger();  // run action or open submenu

    const std::string& getLabel() const { return label; }
    Menu* getSubmenu() const { return submenu.get(); }

private:
    std::string label;
    std::function<void()> action;
    Menu* submenu;
};

class Menu_: public Widget {
public:
    Menu();

    virtual Result draw();
    void addItem(MenuItem item);
    void becomeActive();

private:
    std::string title_;
    std::vector<MenuItem> items_;
};

#endif // MENU_H