#include "RMenuWidget.h"

RMenuWidget::RMenuWidget() {
  cursor_ = 0;
  top_ = 0;
  width_ = RDisplay::DISPLAY_WIDTH;
  setFillsBackground();
}

void RMenuWidget::addEntry(const String& title, std::function<void(void)> callback, int tag) {
  RLabelWidget *label = new RLabelWidget;
  label->setTitle(title);
  label->setFillsBackground();
  label->setTag(tag);

  if(widgets_.size() == 0) {
    label->setBackgroundColor(fgCol_);
    label->setForegroundColor(bgCol_);
    cursor_ = 0;
  } else {
    label->setBackgroundColor(bgCol_);
    label->setForegroundColor(fgCol_);
  }
  label->setSize(width(), RDisplay::CHAR_HEIGHT);
  label->setPosition(0, (widgets_.size()-top_)*RDisplay::CHAR_HEIGHT + y_);
  label->setJustification(RLabelWidget::LEFT_JUSTIFIED, RLabelWidget::VER_CENTERED);

  label->setAction(callback);

  addWidget(label);
}

void RMenuWidget::clear() {
  for(RWidget* w: widgets_) {
    removeWidget(w);
    delete w;
  }
  RMultiWidget::clearWidgets();
  cursor_ = 0;
}

void RMenuWidget::up() {
  if(cursor_ == 0 || widgets_.size() == 0) return;
  widgets_[cursor_]->setBackgroundColor(bgCol_);
  widgets_[cursor_]->setForegroundColor(fgCol_);
  cursor_--;
  widgets_[cursor_]->setBackgroundColor(fgCol_);
  widgets_[cursor_]->setForegroundColor(bgCol_);
}

void RMenuWidget::down() {
  if(cursor_ >= widgets_.size()-1 || widgets_.size() == 0) return;
  widgets_[cursor_]->setBackgroundColor(bgCol_);
  widgets_[cursor_]->setForegroundColor(fgCol_);
  cursor_++;
  widgets_[cursor_]->setBackgroundColor(fgCol_);
  widgets_[cursor_]->setForegroundColor(bgCol_);
}
  
void RMenuWidget::select() {
  widgets_[cursor_]->triggerAction();
}

void RMenuWidget::takeInputFocus() {
  RInput::input.clearCallbacks();
  RInput::input.setTopLeftShortPressCallback([this]{this->up();});
  RInput::input.setTopRightShortPressCallback([this]{this->down();});
  RInput::input.setTopRightLongPressCallback([this]{this->select();});
  RInput::input.setConfirmShortPressCallback([this]{this->select();});
  resetCursor();
}

void RMenuWidget::setSize(uint8_t w, uint8_t h) {
    int deltaW = w - width();
    for(auto& w: widgets_) {
        w->setSize(constrain(w->width()+deltaW, 0, 255), w->height());
    }
    RWidget::setSize(w, h);
}

void RMenuWidget::resetCursor() { 
  if(widgets_.size() == 0) {
    cursor_ = 0;
    return;
  }

  if(cursor_ <= widgets_.size()-1) {
    widgets_[cursor_]->setBackgroundColor(bgCol_);
    widgets_[cursor_]->setForegroundColor(fgCol_);
  }
  cursor_ = 0;
  widgets_[cursor_]->setBackgroundColor(fgCol_);
  widgets_[cursor_]->setForegroundColor(bgCol_);
}

