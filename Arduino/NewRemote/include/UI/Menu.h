#if !defined(MENUWIDGET_H)
#define MENUWIDGET_H

#include <vector>
#include <functional>
#include <LibBB.h>

#include "Config.h"
#include "UI/Display.h"
#include "UI/MultiWidget.h"
#include "UI/Label.h"
#include "UI/Button.h"
#include "UI/PercentageBar.h"
#include "Input.h"

using namespace bb;

class Menu; // fwd decl
class Button; // fwd decl

class Menu: public MultiWidget {
public:
  Menu();

  std::shared_ptr<Button> addEntry(const String& title, std::function<void(Widget*)> callback, bool backAfterCB=false);
  std::shared_ptr<Menu> addSubmenu(const String& title, std::function<void(Menu*)> enterCB=nullptr, std::function<void(Menu*)> leaveCB=nullptr);

  virtual Result draw();
  void clear();
  void up() {setCursor(cursor_-1);};
  void down() {setCursor(cursor_+1);}
  void setCursor(int cursor);
  void encTurn(float enc);
  void resetCursor();

  virtual void takeInputFocus();

  void enter();
  void back(Widget* w = nullptr);
  void select();

  void setEnterCallback(std::function<void(Menu*)> enterCB) { enterCB_ = enterCB; }
  void setLeaveCallback(std::function<void(Menu*)> leaveCB) { leaveCB_ = leaveCB; }
  void setParent(Menu* parent) { parent_ = parent; }
  Menu* parent() { return parent_; }

  virtual void setSize(uint8_t w, uint8_t h);
  virtual void setPosition(int x, int y);

protected:
  void addEntry(const std::shared_ptr<Button>& entry);
  void moveWidgets();

  int cursor_;
  int top_;
  float currentEnc_;
  std::function<void(Menu*)> enterCB_, leaveCB_;

  std::shared_ptr<Button> backEntry_;
  std::shared_ptr<PercentageBar> scrollBar_;

  Menu* parent_ = nullptr;

  static const uint8_t SCROLLBAR_WIDTH = 4;
};


#endif