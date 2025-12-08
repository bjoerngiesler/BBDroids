#if !defined(SEQNUMWIDGET_H)
#define SEQNUMWIDGET_H

#include "UI/Widget.h"

class SeqnumWidget: public Widget {
public:
    SeqnumWidget();
    virtual Result draw();

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