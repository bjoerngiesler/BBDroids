#if !defined(RMESSAGEWIDGET_H)
#define RMESSAGEWIDGET_H

#include <LibBB.h>

#include "Config.h"
#include "UI/RDisplay.h"
#include "UI/RLabelWidget.h"

class RMessageWidget: public RLabelWidget {
public:
  virtual void setTitle(const String& title);
};

#endif // RMESSAGE_H