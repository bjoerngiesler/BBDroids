#if !defined(PERCENTAGEBAR_H)
#define PERCENTAGEBAR_H

#include <LibBB.h>
#include "UI/Widget.h"

class PercentageBar: public Widget {
public:
    PercentageBar();

    enum Orientation {
        HORIZONTAL,
        VERTICAL
    };

    virtual Result draw();
    void setPercentage(float percent);
    void setStart(float percent);
    void setOrientation(Orientation orient);
    Orientation orientation() { return orientation_; }

protected:
    float percent_;
    float start_;
    Orientation orientation_;
};

#endif // PERCENTAGEBAR_H