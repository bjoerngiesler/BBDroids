#include "RGraphsWidget.h"
#include "RRemote.h"

RGraphsWidget::RGraphsWidget() {
  cursor_[TOP] = cursor_[MIDDLE] = cursor_[BOTTOM] = 0;
  title_[TOP] = "Left";
  title_[MIDDLE] = "Right";
  title_[BOTTOM] = "Droid";
}

Result RGraphsWidget::draw(ConsoleStream* stream) {
  int y, titlex;
  
  y = GRAPH0_Y;
  RDisplay::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, RDisplay::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[TOP])*RDisplay::CHAR_WIDTH)/2;
  RDisplay::display.text(titlex, y+2, RDisplay::WHITE, title_[TOP]);

  y += GRAPH_HEIGHT + GRAPH_DIST;
  RDisplay::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, RDisplay::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[MIDDLE])*RDisplay::CHAR_WIDTH)/2;
  RDisplay::display.text(titlex, y+2, RDisplay::WHITE, title_[MIDDLE]);

  y += GRAPH_HEIGHT + GRAPH_DIST;
  RDisplay::display.rect(GRAPH_X-1, y, GRAPH_X+GRAPH_WIDTH+1, y+GRAPH_HEIGHT, RDisplay::WHITE);
  titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[BOTTOM])*RDisplay::CHAR_WIDTH)/2;
  RDisplay::display.text(titlex, y+2, RDisplay::WHITE, title_[BOTTOM]);

  return RWidget::draw(stream);
}
  
void RGraphsWidget::setTitle(Graph g, const char* t) {
  title_[g] = t;
}

void RGraphsWidget::plotAxisData(Graph g, float a0, float a1, float a2, float a3, float a4) {
  int x, y;
  switch(g) {
    case TOP: y = GRAPH0_Y; break;
    case MIDDLE: y = GRAPH0_Y + GRAPH_HEIGHT + GRAPH_DIST; break;
    case BOTTOM: default: y = GRAPH0_Y + 2*GRAPH_HEIGHT + 2*GRAPH_DIST; break;
  }
  y += GRAPH_HEIGHT/2;
  x = GRAPH_X + cursor_[g];

  RDisplay::display.plot(x, y+(a0*GRAPH_HEIGHT/2.2), RDisplay::RED);
  RDisplay::display.plot(x, y+(a1*GRAPH_HEIGHT/2.2), RDisplay::GREEN);
  RDisplay::display.plot(x, y+(a2*GRAPH_HEIGHT/2.2), RDisplay::WHITE);
  RDisplay::display.plot(x, y+(a3*GRAPH_HEIGHT/2.2), RDisplay::BLUE);
  RDisplay::display.plot(x, y+(a4*GRAPH_HEIGHT/2.2), RDisplay::YELLOW);
}

void RGraphsWidget::plotControlPacket(Graph g, const bb::ControlPacket& packet) {
  plotAxisData(g, packet.getAxis(0), packet.getAxis(1), packet.getAxis(2), packet.getAxis(3), packet.getAxis(4));
}

void RGraphsWidget::advanceCursor(Graph g) { 
  cursor_[g]++; 
  if(cursor_[g]>GRAPH_WIDTH) {
    cursor_[g]=0;
    int titlex, y;
    switch(g) {
      case TOP: y = GRAPH0_Y; break;
      case MIDDLE: y = GRAPH0_Y + GRAPH_HEIGHT + GRAPH_DIST; break;
      case BOTTOM: default: y = GRAPH0_Y + 2*GRAPH_HEIGHT + 2*GRAPH_DIST; break;
    }
    titlex = GRAPH_X-1 + (GRAPH_WIDTH-strlen(title_[g])*RDisplay::CHAR_WIDTH)/2;

    RDisplay::display.rect(GRAPH_X, y+2, GRAPH_X+GRAPH_WIDTH, y+GRAPH_HEIGHT-2, RDisplay::BLACK, true);
    RDisplay::display.text(titlex, y+2, RDisplay::WHITE, title_[g]);
  }
}

void RGraphsWidget::buttonTopLeftPressed() {
  RRemote::remote.showMainMenu();
}

void RGraphsWidget::buttonTopRightPressed() {
  RRemote::remote.showMainMenu();
}

void RGraphsWidget::buttonConfirmPressed() {
  RRemote::remote.showMainMenu();
}

