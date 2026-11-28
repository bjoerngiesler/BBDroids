#include "UI/MixDiagramWidget.h"
#include "UI/UI.h"
#include "UI/Display.h"
#include "Input.h"

// Layout, relative to the widget origin. Designed for the 80x132 main area.
static const int COL_X[2] = {1, 41};
static const int COL_W = 38;
static const int AX_Y = 2, AX_H = 13;
static const int IP_Y = 19, IP_H = 34;
static const int ROUTE_Y = 58;                  // horizontal routing line between curves and mix block
static const int MIX_Y = 63, MIX_H = 13, MIX_OP_W = 26;
static const int OUT_Y = 86, OUT_H = 25, OUT_W = 52;
static const int BTN_Y = 116, BTN_H = 13;
static const int OK_X = 2, OK_W = 32, CANCEL_X = 38, CANCEL_W = 40;

MixDiagramWidget::MixDiagramWidget() {
    curve1_.setShowsNodes(false);
    curve2_.setShowsNodes(false);
    axisMenu_ = make_shared<Menu>();
    axisMenu_->setBackCallback([this]() { returnFromAxisMenu(); });
}

void MixDiagramWidget::setup(Menu* parent, Protocol* proto, Transmitter* tx, MixManager* mgr, InputID inp, const String& inputName) {
    parent_ = parent;
    proto_ = proto;
    tx_ = tx;
    mgr_ = mgr;
    inp_ = inp;
    inputName_ = inputName;
    mix_ = origMix_ = mgr->mixForInput(inp);
    selected_ = AX1;
    setNeedsFullRedraw();
}

void MixDiagramWidget::takeInputFocus() {
    // Called again when a curve dialog closes, so don't reset state here.
    Input::inst.clearCallbacks();
    Input::inst.setLeftShortPressCallback([this]() { prev(); });
    Input::inst.setRightShortPressCallback([this]() { next(); });
    Input::inst.setConfirmShortPressCallback([this]() { confirm(); });
    Input::inst.setConfirmLongPressCallback([this]() { commitAndLeave(); });
}

void MixDiagramWidget::select(Block b) {
    setDirty(selected_);
    selected_ = b;
    setDirty(selected_);
}

void MixDiagramWidget::next() {
    select(Block((selected_ + 1) % NUM_BLOCKS));
}

void MixDiagramWidget::prev() {
    select(Block((selected_ + NUM_BLOCKS - 1) % NUM_BLOCKS));
}

void MixDiagramWidget::confirm() {
    switch(selected_) {
    case AX1:
    case AX2:
        showAxisMenu(selected_ == AX1);
        break;
    case IP1:
    case IP2: {
        bool first = (selected_ == IP1);
        UI::ui.showMixCurveDialog(first ? mix_.interp1 : mix_.interp2, [this, first](const Interpolator& i) {
            if(first) mix_.interp1 = i; else mix_.interp2 = i;
            setNeedsFullRedraw();
        });
        break;
    }
    case MIX:
        if(mix_.mixType == MIX_NONE) mix_.mixType = MIX_ADD;
        else if(mix_.mixType == MIX_ADD) mix_.mixType = MIX_MULT;
        else mix_.mixType = MIX_NONE;
        // Routing changes, and chain 2 is greyed out or not.
        setDirty(MIX); setDirty(AX2); setDirty(IP2);
        break;
    case OK:
        commitAndLeave();
        break;
    case CANCEL:
    default:
        leave();
        break;
    }
}

// Menu with "-" and all axes except the one used by the other chain; the current one is highlighted.
void MixDiagramWidget::showAxisMenu(bool first) {
    if(tx_ == nullptr) return;
    AxisID cur = first ? mix_.axis1 : mix_.axis2;
    AxisID other = first ? mix_.axis2 : mix_.axis1;

    axisMenu_->clear();
    axisMenu_->setName(first ? "Axis 1" : "Axis 2");
    auto entry = axisMenu_->addEntry("-", [this, first](Widget*) {
        if(first) mix_.axis1 = AXIS_INVALID; else mix_.axis2 = AXIS_INVALID;
        returnFromAxisMenu();
    });
    entry->setHighlighted(cur == AXIS_INVALID);
    for(AxisID a=0; a<tx_->numAxes(); a++) {
        if(a == other) continue;
        entry = axisMenu_->addEntry(tx_->axisName(a).c_str(), [this, first, a](Widget*) {
            if(first) mix_.axis1 = a; else mix_.axis2 = a;
            returnFromAxisMenu();
        });
        entry->setHighlighted(a == cur);
    }
    axisMenu_->enter();
}

void MixDiagramWidget::returnFromAxisMenu() {
    UI::ui.setMainWidget(this);
    UI::ui.setTopTitle(inputName_);
    setNeedsFullRedraw();
}

void MixDiagramWidget::commitAndLeave() {
    if(mgr_ != nullptr && memcmp(&mix_, &origMix_, sizeof(AxisMix)) != 0) {
        mgr_->setMix(inp_, mix_);
        if(proto_ != nullptr && proto_->receiverSideMixing()) {
            UI::ui.showMessage("Sending mixes...", 0);
            if(proto_->sendMixes() == false) UI::ui.showMessage("Sending failed", 2000, Display::LIGHTRED1);
        }
        origMix_ = mix_;
    }
    leave();
}

void MixDiagramWidget::leave() {
    if(parent_ != nullptr) parent_->enter();
    else UI::ui.showMain();
}

String MixDiagramWidget::axisLabel(AxisID a) {
    if(a == AXIS_INVALID || tx_ == nullptr) return "-";
    String s = tx_->axisName(a).c_str();
    // Too long: abbreviate in the middle, e.g. "Button1" -> "Bu~n1" for 5 chars.
    unsigned int maxChars = (COL_W - 4) / Display::CHAR_WIDTH;
    if(s.length() > maxChars) {
        unsigned int head = (maxChars - 1) / 2, tail = maxChars - 1 - head;
        s = s.substring(0, head) + "~" + s.substring(s.length() - tail);
    }
    return s;
}

void MixDiagramWidget::layout() {
    curve1_.setPosition(x_ + COL_X[0], y_ + IP_Y);
    curve1_.setSize(COL_W, IP_H);
    curve2_.setPosition(x_ + COL_X[1], y_ + IP_Y);
    curve2_.setSize(COL_W, IP_H);
}

// Unselected blocks get a thin frame in col. The selected block gets a double frame in the highlight color.
void MixDiagramWidget::drawBox(int x, int y, int w, int h, Block b, uint8_t col) {
    x += x_; y += y_;
    if(b == selected_) {
        uint8_t selCol = hlCol_;
        Display::display.rect(x-1, y-1, x+w, y+h, selCol, false);
        Display::display.rect(x, y, x+w-1, y+h-1, selCol, false);
    } else {
        Display::display.rect(x, y, x+w-1, y+h-1, col, false);
    }
}

void MixDiagramWidget::drawCenteredText(int x, int y, int w, uint8_t col, const String& str) {
    int tx = x + (w - int(str.length()) * Display::CHAR_WIDTH) / 2;
    if(tx < x) tx = x;
    Display::display.text(x_ + tx, y_ + y, col, str);
}

void MixDiagramWidget::drawResult(bool force) {
    int val = 0;
    if(tx_ != nullptr) val = int(rintf(tx_->computeMix(mix_) * 100.0f));
    if(!force && val == lastResult_) return;
    lastResult_ = val;

    char buf[16];
    snprintf(buf, sizeof(buf), "%+d%%", val);
    // Clear only the value line; must stay clear of the name above and the frame below.
    int outX = (COL_X[0] + COL_X[1] + COL_W - OUT_W) / 2;
    int ry = OUT_Y + 2 + Display::CHAR_HEIGHT + 1;
    Display::display.rect(x_ + outX + 1, y_ + ry - 1, x_ + outX + OUT_W - 2, y_ + ry + Display::CHAR_HEIGHT - 1, bgCol_, true);
    drawCenteredText(outX, ry, OUT_W, Display::WHITE, buf);
}

void MixDiagramWidget::clearRegion(int x1, int y1, int x2, int y2) {
    Display::display.rect(x_ + x1, y_ + y1, x_ + x2, y_ + y2, bgCol_, true);
}

// Each block below clears its own region, including the 1px outside its box where the selection frame goes.
// Regions don't overlap; the axis->curve connector crosses the border between axis and curve region, so
// both draw it.

void MixDiagramWidget::drawAxisCurveConnector(int col) {
    bool hasOp = (mix_.mixType != MIX_NONE);
    uint8_t lineCol = (col == 0 || hasOp) ? Display::LIGHTGREY : Display::GREY;
    int c = COL_X[col] + COL_W/2;
    Display::display.line(x_ + c, y_ + AX_Y + AX_H + 1, x_ + c, y_ + IP_Y - 2, lineCol);
}

void MixDiagramWidget::drawAxisBlock(int col) {
    bool hasOp = (mix_.mixType != MIX_NONE);
    uint8_t fg = (col == 0 || hasOp) ? Display::WHITE : Display::GREY;
    clearRegion(COL_X[col] - 1, AX_Y - 1, COL_X[col] + COL_W, AX_Y + AX_H);
    drawBox(COL_X[col], AX_Y, COL_W, AX_H, col == 0 ? AX1 : AX2, fg);
    drawCenteredText(COL_X[col], AX_Y + 2, COL_W, fg, axisLabel(col == 0 ? mix_.axis1 : mix_.axis2));
    drawAxisCurveConnector(col);
}

void MixDiagramWidget::drawCurveBlock(int col) {
    bool hasOp = (mix_.mixType != MIX_NONE);
    Block b = (col == 0) ? IP1 : IP2;
    MixCurveWidget& curve = (col == 0) ? curve1_ : curve2_;
    clearRegion(COL_X[col] - 1, IP_Y - 1, COL_X[col] + COL_W, IP_Y + IP_H);
    curve.setForegroundColor((col == 0 || hasOp) ? Display::WHITE : Display::GREY);
    curve.setInterpolator(col == 0 ? mix_.interp1 : mix_.interp2);
    curve.draw();
    if(selected_ == b) drawBox(COL_X[col], IP_Y, COL_W, IP_H, b, Display::WHITE);
    drawAxisCurveConnector(col);
}

// Everything between the curves and the result block: routing lines, mix block, connector to the result.
void MixDiagramWidget::drawRouting() {
    clearRegion(0, IP_Y + IP_H + 1, width_ - 1, OUT_Y - 1);

    bool hasOp = (mix_.mixType != MIX_NONE);
    uint8_t lineCol = Display::LIGHTGREY;
    int c1 = COL_X[0] + COL_W/2, c2 = COL_X[1] + COL_W/2;
    int top = IP_Y + IP_H + 1;
    int mixX, mixW, mixC;
    if(hasOp) {
        mixW = MIX_OP_W;
        mixX = (COL_X[0] + COL_X[1] + COL_W - mixW) / 2;
        mixC = mixX + mixW/2;
        Display::display.line(x_ + c1, y_ + top, x_ + c1, y_ + ROUTE_Y, lineCol);
        Display::display.line(x_ + c2, y_ + top, x_ + c2, y_ + ROUTE_Y, lineCol);
        Display::display.line(x_ + c1, y_ + ROUTE_Y, x_ + c2, y_ + ROUTE_Y, lineCol);
        Display::display.line(x_ + mixC, y_ + ROUTE_Y, x_ + mixC, y_ + MIX_Y - 2, lineCol);
    } else {
        mixW = COL_W;
        mixX = COL_X[0];
        mixC = c1;
        Display::display.line(x_ + c1, y_ + top, x_ + c1, y_ + MIX_Y - 2, lineCol);
    }
    drawBox(mixX, MIX_Y, mixW, MIX_H, MIX, Display::WHITE);
    const char* op = (mix_.mixType == MIX_ADD) ? "+" : (mix_.mixType == MIX_MULT) ? "x" : "=";  // "=": pass axis 1 through unchanged
    drawCenteredText(mixX, MIX_Y + 2, mixW, Display::WHITE, op);

    int outC = (COL_X[0] + COL_X[1] + COL_W) / 2;
    int midY = (MIX_Y + MIX_H + OUT_Y) / 2;
    Display::display.line(x_ + mixC, y_ + MIX_Y + MIX_H + 1, x_ + mixC, y_ + midY, lineCol);
    Display::display.line(x_ + mixC, y_ + midY, x_ + outC, y_ + midY, lineCol);
    Display::display.line(x_ + outC, y_ + midY, x_ + outC, y_ + OUT_Y - 1, lineCol);
}

// Input name and live value. Not selectable; the value is updated by drawResult().
void MixDiagramWidget::drawResultBlock() {
    int outX = (COL_X[0] + COL_X[1] + COL_W - OUT_W) / 2;
    clearRegion(outX, OUT_Y, outX + OUT_W - 1, OUT_Y + OUT_H - 1);
    Display::display.rect(x_ + outX, y_ + OUT_Y, x_ + outX + OUT_W - 1, y_ + OUT_Y + OUT_H - 1, Display::LIGHTGREEN1, false);
    String name = inputName_;
    unsigned int maxChars = (OUT_W - 4) / Display::CHAR_WIDTH;
    if(name.length() > maxChars) name = name.substring(0, maxChars);
    drawCenteredText(outX, OUT_Y + 2, OUT_W, Display::LIGHTGREEN1, name);
    drawResult(true);
}

void MixDiagramWidget::drawButton(Block b) {
    int bx = (b == OK) ? OK_X : CANCEL_X;
    int bw = (b == OK) ? OK_W : CANCEL_W;
    clearRegion(bx - 1, BTN_Y - 1, bx + bw, BTN_Y + BTN_H);
    drawBox(bx, BTN_Y, bw, BTN_H, b, Display::WHITE);
    drawCenteredText(bx, BTN_Y + 2, bw, Display::WHITE, b == OK ? "OK" : "Cancel");
}

Result MixDiagramWidget::draw() {
    if(needsFullRedraw_) {
        needsFullRedraw_ = false;
        needsContentsRedraw_ = false;
        layout();
        Display::display.rect(x_, y_, x_ + width_ - 1, y_ + height_ - 1, bgCol_, true);
        dirty_ = 0xff;
        drawResultBlock();
    }

    if(dirty_ & (1 << AX1)) drawAxisBlock(0);
    if(dirty_ & (1 << AX2)) drawAxisBlock(1);
    if(dirty_ & (1 << IP1)) drawCurveBlock(0);
    if(dirty_ & (1 << IP2)) drawCurveBlock(1);
    if(dirty_ & (1 << MIX)) drawRouting();
    if(dirty_ & (1 << OK)) drawButton(OK);
    if(dirty_ & (1 << CANCEL)) drawButton(CANCEL);
    dirty_ = 0;

    drawResult(false);
    return RES_OK;
}
