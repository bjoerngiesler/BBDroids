#if !defined(RSEQNUMWIDGET_H)
#define RSEQNUMWIDGET_H

#include "RWidget.h"

class RSeqnumWidget: public RWidget {
public:
    RSeqnumWidget();
    virtual Result draw(ConsoleStream *stream = NULL);

    void setSquareColor(uint8_t square, uint8_t color);

protected:
    struct Square {
        bool dirty;
        uint8_t color;
    };
    std::vector<Square> squares_;
};

#endif // RSEQNUMWIDGET_H