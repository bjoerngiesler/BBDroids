#if !defined(RIMUWIDGET_H)
#define RIMUWIDGET_H

#include "RWidget.h"

class RIMUWidget: public RWidget {
public:
    RIMUWidget();

    virtual Result draw(ConsoleStream* stream = NULL);
    virtual void setRPH(float r, float p, float h);
    virtual void setAccel(float x, float y, float z);
    virtual void setShowsText(bool shows=true);
    virtual void setCursorColor(uint8_t col);
protected:
    float roll_, pitch_, heading_, accelX_, accelY_, accelZ_;
    uint8_t horizX1Old_, horizY1Old_, horizX2Old_, horizY2Old_;
    uint8_t headingX1Old_, headingY1Old_, headingX2Old_, headingY2Old_;
    uint8_t axOld_, ayOld_, azOld_;
    
    static const uint8_t BUFLEN=16;
    char buf_[BUFLEN];
    bool showsText_;
    uint8_t cursorCol_;
};

#endif // RWIDGET_H