#include "UI/GraphsWidget.h"
#include "Todo/RRemote.h"

GraphsWidget::GraphsWidget() {
  cursor_[TOP] = cursor_[MIDDLE] = cursor_[BOTTOM] = 0;
  title_[TOP] = "Left";
  title_[MIDDLE] = "Right";
  title_[BOTTOM] = "Droid";
}

Result GraphsWidget::draw() {
  int y, titlex;
  
  y = GRAPH0_Y;
  Display::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, Display::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[TOP])*Display::CHAR_WIDTH)/2;
  Display::display.text(titlex, y+2, Display::WHITE, title_[TOP]);

  y += GRAPH_HEIGHT + GRAPH_DIST;
  Display::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, Display::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[MIDDLE])*Display::CHAR_WIDTH)/2;
  Display::display.text(titlex, y+2, Display::WHITE, title_[MIDDLE]);

  y += GRAPH_HEIGHT + GRAPH_DIST;
  Display::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, Display::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[BOTTOM])*Display::CHAR_WIDTH)/2;
  Display::display.text(titlex, y+2, Display::WHITE, title_[BOTTOM]);

  return Widget::draw();
}
  
void GraphsWidget::setTitle(Graph g, const char* t) {
  title_[g] = t;
}

void GraphsWidget::plotAxisData(Graph g, float a0, float a1, float a2, float a3, float a4) {
  int x, y;
  switch(g) {
    case TOP: y = GRAPH0_Y; break;
    case MIDDLE: y = GRAPH0_Y + GRAPH_HEIGHT + GRAPH_DIST; break;
    case BOTTOM: default: y = GRAPH0_Y + 2*GRAPH_HEIGHT + 2*GRAPH_DIST; break;
  }
  y += GRAPH_HEIGHT/2;
  x = GRAPH_X + cursor_[g];

  Display::display.plot(x, y+(a0*GRAPH_HEIGHT/2.2), Display::RED);
  Display::display.plot(x, y+(a1*GRAPH_HEIGHT/2.2), Display::GREEN);
  Display::display.plot(x, y+(a2*GRAPH_HEIGHT/2.2), Display::WHITE);
  Display::display.plot(x, y+(a3*GRAPH_HEIGHT/2.2), Display::BLUE);
  Display::display.plot(x, y+(a4*GRAPH_HEIGHT/2.2), Display::YELLOW);
}

void GraphsWidget::plotControlPacket(Graph g, const bb::ControlPacket& packet) {
  plotAxisData(g, packet.getAxis(0), packet.getAxis(1), packet.getAxis(2), packet.getAxis(3), packet.getAxis(4));
}

void GraphsWidget::advanceCursor(Graph g) { 
  cursor_[g]++; 
  if(cursor_[g]>GRAPH_WIDTH) {
    cursor_[g]=0;
    int titlex, y;
    switch(g) {
      case TOP: y = GRAPH0_Y; break;
      case MIDDLE: y = GRAPH0_Y + GRAPH_HEIGHT + GRAPH_DIST; break;
      case BOTTOM: default: y = GRAPH0_Y + 2*GRAPH_HEIGHT + 2*GRAPH_DIST; break;
    }
    titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[g])*Display::CHAR_WIDTH)/2;

    Display::display.rect(GRAPH_X, y+2, GRAPH_X+GRAPH_WIDTH, y+GRAPH_HEIGHT-2, Display::BLACK, true);
    Display::display.text(titlex, y+2, Display::WHITE, title_[g]);
  }
}

void GraphsWidget::buttonTopLeftPressed() {
  //RRemote::remote.showMainMenu();
}

void GraphsWidget::buttonTopRightPressed() {
  //RRemote::remote.showMainMenu();
}

void GraphsWidget::buttonConfirmPressed() {
  //RRemote::remote.showMainMenu();
}

