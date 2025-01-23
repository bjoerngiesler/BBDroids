#if !defined(RSEQNUMWIDGET_H)
#define RSEQNUMWIDGET_H

#include "RWidget.h"

class RSeqnumWidget: public RWidget {
public:
    RSeqnumWidget();
    virtual Result draw(ConsoleStream *stream = NULL);

    void setChar(char c);
    void setSquareColor(uint8_t square, uint8_t color);
    void setNoComm(bool yesno);

protected:
    struct Square {
        bool dirty;
        uint8_t color;
    };
    std::vector<Square> squares_;
    char c_;
    bool nocomm_;
};

#endif // RSEQNUMWIDGET_H