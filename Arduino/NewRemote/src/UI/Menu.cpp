#include "UI/Menu.h"
#include "UI/UI.h"

Menu::Menu() {
  cursor_ = 0;
  top_ = 0;
  x_ = Display::MAIN_X;
  y_ = Display::MAIN_Y;
  width_ = Display::MAIN_WIDTH;
  height_ = Display::MAIN_HEIGHT;
  setFillsBackground();

  backEntry_ = make_shared<Button>();
  backEntry_->setTitle("<< Back <<");
  backEntry_->setAction([this](Widget* w){this->back(w);});
  backEntry_->setBackgroundColor(fgCol_);
  backEntry_->setForegroundColor(bgCol_);
  backEntry_->setPosition(0, (widgets_.size()-top_)*Display::CHAR_HEIGHT + y_);

  scrollBar_ = make_shared<PercentageBar>();
  scrollBar_->setPosition(x_ + width_ - SCROLLBAR_WIDTH, y_);
  scrollBar_->setSize(SCROLLBAR_WIDTH, height_);
  scrollBar_->setOrientation(PercentageBar::VERTICAL);
  scrollBar_->setBackgroundColor(bgCol_);
  scrollBar_->setForegroundColor(Display::LIGHTGREY);
}

std::shared_ptr<Button> Menu::addEntry(const String& title, std::function<void(Widget*)> cb, int tag) {
  shared_ptr<Button> entry = make_shared<Button>();
  entry->setTag(tag);
  entry->setTitle(title);
  entry->setAction(cb);
  addEntry(entry);
  return entry;
}

std::shared_ptr<Menu> Menu::addSubmenu(const String& title) {
  shared_ptr<Menu> menu = make_shared<Menu>();
  menu->setName(title);
  menu->setParent(this);
  shared_ptr<MenuButton> entry = make_shared<MenuButton>();
  entry->setTitle(title);
  entry->setMenu(menu);
  addEntry(entry);
  return menu;
}

void Menu::addEntry(const shared_ptr<Button>& entry) {
  entry->setFillsBackground();

  if(widgets_.size() == 0) { // this is the one under the cursor
    uint16_t col = entry->backgroundColor();
    entry->setBackgroundColor(entry->foregroundColor());
    entry->setForegroundColor(col);
    backEntry_->setBackgroundColor(bgCol_);
    backEntry_->setForegroundColor(fgCol_);
    cursor_ = 0;
  }

  entry->setSize(width(), Display::CHAR_HEIGHT);
  entry->setPosition(0, (widgets_.size()-top_)*Display::CHAR_HEIGHT + y_);
  entry->setJustification(Label::LEFT_JUSTIFIED, Label::VER_CENTERED);

  addWidget(entry);

  backEntry_->setPosition(0, (widgets_.size()-top_)*Display::CHAR_HEIGHT + y_);
}

Result Menu::draw() {
  if(needsFullRedraw_) {
    scrollBar_->setNeedsFullRedraw();
    backEntry_->setNeedsFullRedraw();
    for(auto& w: widgets_) {
      w->setNeedsFullRedraw();
    }
    Display::display.rect(x_, y_, x_+width_, y_+height_, bgCol_, true);
    needsFullRedraw_ = false;
  }
  unsigned int num = height() / Display::CHAR_HEIGHT;

  for(unsigned int i=top_; i<top_+num && i<=widgets_.size(); i++) {
    if(i < widgets_.size()) widgets_[i]->draw();
    else backEntry_->draw();
  }

  if(widgets_.size()+1 > num) {
    scrollBar_->draw();
  }

  return RES_OK;
}

void Menu::enter() {
  if(enterCB_) enterCB_(this);
  
  moveWidgets();
  UI::ui.setMainWidget(this);
  UI::ui.setTopTitle(name_);
  for(auto w: widgets_) w->setNeedsFullRedraw();
  backEntry_->setNeedsFullRedraw();
  takeInputFocus();
}

void Menu::back(Widget* w) {
  if(parent_ != nullptr) parent_->enter();
  else UI::ui.showMain();
}

void Menu::clear() {
  for(auto w: widgets_) {
    removeWidget(w);
  }
  MultiWidget::clearWidgets();
  cursor_ = top_ = 0;
}

void Menu::setCursor(int cursor) {
  if(cursor == cursor_ || cursor < 0 || cursor > widgets_.size()) return;

  if(cursor_ < widgets_.size()) {
    uint16_t col = widgets_[cursor_]->backgroundColor();
    widgets_[cursor_]->setBackgroundColor(widgets_[cursor_]->foregroundColor());
    widgets_[cursor_]->setForegroundColor(col);
  } else if(cursor_ == widgets_.size()) {
    uint16_t col = backEntry_->backgroundColor();
    backEntry_->setBackgroundColor(backEntry_->foregroundColor());
    backEntry_->setForegroundColor(col);
  }

  int num = height_/Display::CHAR_HEIGHT;
  if(top_ + cursor >= num && cursor > cursor_) {
    top_ = cursor-num+1;
    moveWidgets();
  } else if(top_ > cursor && cursor < cursor_) {
    top_ = cursor;
    moveWidgets();
  }
  cursor_ = cursor;

  if(cursor_ < widgets_.size()) {
    uint16_t col = widgets_[cursor_]->backgroundColor();
    widgets_[cursor_]->setBackgroundColor(widgets_[cursor_]->foregroundColor());
    widgets_[cursor_]->setForegroundColor(col);
  } else if(cursor_ == widgets_.size()) {
    uint16_t col = backEntry_->backgroundColor();
    backEntry_->setBackgroundColor(backEntry_->foregroundColor());
    backEntry_->setForegroundColor(col);
  }
}

void Menu::encTurn(float enc) {
  currentEnc_ += enc;
  while(currentEnc_ > 50.0) {
    down();
    currentEnc_ -= 50.0;
  }
  while(currentEnc_ < -50.0) {
    up();
    currentEnc_ += 50.0;
  }
}

void Menu::select() {
  if(cursor_ < widgets_.size()) widgets_[cursor_]->triggerAction();
  else if(cursor_ == widgets_.size()) backEntry_->triggerAction();
}

void Menu::takeInputFocus() {
  Input::inst.clearCallbacks();
  Input::inst.setLeftShortPressCallback([this]{this->up();});
  Input::inst.setRightShortPressCallback([this]{this->down();});
  Input::inst.setRightLongPressCallback([this]{this->select();});
  Input::inst.setConfirmShortPressCallback([this]{this->select();});
  Input::inst.setEncTurnCallback([this](float enc) {this->encTurn(enc);});
}

void Menu::setPosition(int x, int y) {
  Widget::setPosition(x, y);
  moveWidgets();
}


void Menu::setSize(uint8_t w, uint8_t h) {
  Widget::setSize(w, h);
  moveWidgets();
}

void Menu::resetCursor() { 
  setCursor(0);
}

void Menu::moveWidgets() {
  int y = -top_*Display::CHAR_HEIGHT + y_;
  int num = height_/Display::CHAR_HEIGHT;

  scrollBar_->setPosition(x_ + width_ - SCROLLBAR_WIDTH, y_);
  scrollBar_->setSize(SCROLLBAR_WIDTH, height_);
  scrollBar_->setPercentage(float(num) / (widgets_.size()+1));
  scrollBar_->setStart(float(top_) / (widgets_.size()+1));

  int w = width_;
  if(widgets_.size()+1 > num) {
    w = w - SCROLLBAR_WIDTH-2;
  }
  for(auto& widget: widgets_) {
      widget->setSize(w, Display::CHAR_HEIGHT);
  }
  backEntry_->setSize(w, Display::CHAR_HEIGHT);

  for(uint8_t i=0; i<widgets_.size(); i++) {
    widgets_[i]->setPosition(x_, y);
    y = y + widgets_[i]->height();
  }
  backEntry_->setPosition(x_, y);
}

