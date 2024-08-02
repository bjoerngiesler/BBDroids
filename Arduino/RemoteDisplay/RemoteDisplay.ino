#include <GFX4dIoD9.h>
#include "QRCode.h"
#include "Bav_Builders_Black.h"
#include "RDSerialInterface.h"

GFX4dIoD9 gfx = GFX4dIoD9();
static const uint8_t DISPLAY_WIDTH = 80;
static const uint8_t DISPLAY_HEIGHT = 160;

void showBitmap(unsigned int xoffs, unsigned int yoffs, const char *bmp_data, unsigned int bmp_width, unsigned int bmp_height, bool greyscale) {
  #define RGBTO16BIT(r,g,b) ((r>>3)<<11 | (g>>2)<<5 | ((b>>3)&0x1f))

  for(size_t y=0; y<bmp_height; y++) {
    for(size_t x=0; x<bmp_width; x++) {
      if(greyscale) {
        uint8_t p;
        HEADER_PIXEL_GREYSCALE(bmp_data, p);
        gfx.PutPixel(x+xoffs, y+yoffs, RGBTO16BIT(p, p, p));
      } else {
        uint8_t p[3];
        HEADER_PIXEL(bmp_data, p);

        gfx.PutPixel(x+xoffs, y+yoffs, RGBTO16BIT(p[0], p[1], p[2]));
      }
    }
  }
}

void printCentered(int y, const String& str) {
  unsigned int x = (DISPLAY_WIDTH-str.length()*6)/2;
  gfx.MoveTo(x, y);
  gfx.print(str.c_str());
}

void showLogoScreen(const char* bmp, unsigned int bmp_width, unsigned int bmp_height, bool greyscale) {
  unsigned int y=0;
  gfx.TextColor(WHITE); 

  printCentered(y, "Droid Remote");
  y+=10;
  gfx.Hline(0, y, DISPLAY_WIDTH, WHITE);
  y+=4;

  gfx.TextColor(LIGHTGREY);
  printCentered(y, "(c) 2023");
  y+=22;

  showBitmap((DISPLAY_WIDTH-bmp_width)/2, y, bmp, bmp_width, bmp_height, greyscale);

  y += bmp_height + 3;
  printCentered(y, "URL:t.ly/Xhtw");
  y += 22;
  printCentered(y, "HW: F.Beyer");
  y+=8;
  printCentered(y, "SW: B.Giesler");
}

void runLogoScreen() {
  gfx.Cls();
  while(true) {
    showLogoScreen(LOGO_DATA, LOGO_WIDTH, LOGO_HEIGHT, false);
    int timeout = 5000;
    while(timeout-- > 0) {
      if(rd::SerialInterface::serial.available() == true) {
        gfx.Scroll(160);
        return;
      }
      delay(1);
    }

    showLogoScreen(QRCODE_DATA, QRCODE_WIDTH, QRCODE_HEIGHT, true);
    timeout = 5000;
    while(timeout-- > 0) {
      if(rd::SerialInterface::serial.available() == true) {
        gfx.Scroll(160);
        return;
      };
      delay(1);
    }
  }
}

void setup() {
  rd::SerialInterface::serial.begin();

  gfx.begin(); // Initialize the display
  gfx.Cls();
  gfx.ScrollEnable(true);
  //gfx.BacklightOn(true);
  gfx.Orientation(PORTRAIT_R); 
  gfx.SmoothScrollSpeed(5);
  gfx.TextColor(WHITE); 
  gfx.Font(1); 
  gfx.TextSize(1);

  runLogoScreen();

  gfx.Cls();
}

void loop() {
  rd::SerialInterface::Result res = rd::SerialInterface::serial.handleInput();
  if(res == rd::SerialInterface::RESULT_RUN_LOGO_SCREEN) {
    runLogoScreen();
    gfx.Cls();
  }
}
