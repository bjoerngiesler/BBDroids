#if !defined(MESSAGEWIDGET_H)
#define MESSAGEWIDGET_H

#include <LibBB.h>

#include "Config.h"
#include "UI/Display.h"
#include "UI/Label.h"

class MessageWidget: public Label {
public:
  virtual void setTitle(const String& title);
};

#endif // MESSAGEWIDGET_H