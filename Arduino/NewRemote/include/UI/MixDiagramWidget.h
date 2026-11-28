#if !defined(MIXDIAGRAMWIDGET_H)
#define MIXDIAGRAMWIDGET_H

#include "UI/Widget.h"
#include "UI/MixCurveWidget.h"
#include "UI/Menu.h"
#include <BBRTransmitter.h>
#include <BBRMixManager.h>
#include <BBRProtocol.h>

using namespace bb::rmt;

//! Block diagram of the mix for one input: two parallel chains (axis -> curve), routed into the mix
//! operation (or straight down from chain 1 if there is none), into the live result.
//!
//! Left/Right select a block (row by row: Ax1, Ax2, Curve1, Curve2, Mix, OK, Cancel). Confirm:
//! - Axis: open a menu with all axes (and "-"); Back in that menu returns without change.
//! - Curve: open the mix curve dialog.
//! - Mix: cycle None ("=") -> Add ("+") -> Mult ("x").
//! - OK (or long press Confirm anywhere): store the mix, send it to the droid if it mixes
//!   receiver-side, and go back to the parent menu. Nothing is stored or sent before that.
//! - Cancel: discard all changes and go back to the parent menu.
class MixDiagramWidget: public Widget {
public:
    MixDiagramWidget();

    void setup(Menu* parent, Protocol* proto, Transmitter* tx, MixManager* mgr, InputID inp, const String& inputName);

    virtual Result draw();
    virtual void takeInputFocus();

protected:
    enum Block { AX1 = 0, AX2, IP1, IP2, MIX, OK, CANCEL, NUM_BLOCKS };

    void next();
    void prev();
    void confirm();
    void showAxisMenu(bool first);
    void returnFromAxisMenu();
    void commitAndLeave();
    void leave();

    void layout();
    void drawBox(int x, int y, int w, int h, Block b, uint8_t col);
    void drawCenteredText(int x, int y, int w, uint8_t col, const String& str);
    void drawResult(bool force);
    void setDirty(Block b) { dirty_ |= (1 << b); }
    void select(Block b);
    void clearRegion(int x1, int y1, int x2, int y2);
    void drawAxisBlock(int col);
    void drawCurveBlock(int col);
    void drawAxisCurveConnector(int col);
    void drawRouting();
    void drawResultBlock();
    void drawButton(Block b);
    String axisLabel(AxisID a);

    Menu* parent_ = nullptr;
    Protocol* proto_ = nullptr;
    Transmitter* tx_ = nullptr;
    MixManager* mgr_ = nullptr;
    InputID inp_ = INPUT_INVALID;
    String inputName_;

    AxisMix mix_, origMix_;
    Block selected_ = AX1;
    shared_ptr<Menu> axisMenu_;
    int lastResult_ = 0x7fff;
    uint8_t dirty_ = 0;   // bit per Block that needs redrawing; full redraw is needsFullRedraw_

    MixCurveWidget curve1_, curve2_;
};

#endif // MIXDIAGRAMWIDGET_H
