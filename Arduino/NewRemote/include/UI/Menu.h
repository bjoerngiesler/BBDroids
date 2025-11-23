#if !defined(MENU_H)
#define MENU_H

#include "RWidget.h"

class Menu; // forward declaration

class MenuItem {
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
    std::unique_ptr<Menu> submenu;
};

class Menu {
public:
    Menu(std::string title);

    void addItem(MenuItem item);
    void show();               // display menu
    void handleInput();        // take user input (simple example)

    const std::string& getTitle() const { return title; }

private:
    std::string title;
    std::vector<MenuItem> items;
};

#endif // MENU_H