#include "RIMUWidget.h"

using namespace bb;

RIMUWidget::RIMUWidget() {
    accelX_ = accelY_ = accelZ_ = 0;
    roll_ = pitch_ = heading_ = 0;
    showsText_ = false;
    showsAccel_ = false;
    snprintf(buf_, BUFLEN, "R? P? H?");
}

Result RIMUWidget::draw(ConsoleStream* stream) {
    int diam, x, y;

    if(height_ < width_/2) {
        diam = height_;
        x = diam/2;
    } else {
        diam = width_;
        x = diam/2;
    }
    
    float rad = diam/2;
    y = rad;

    x += x_;
    y += y_;

    if(needsFullRedraw_) {
        Console::console.printfBroadcast("Full redraw\n");
        RDisplay::display.circle(x, y, rad, bgCol_, true);
        RDisplay::display.circle(x, y, rad, fgCol_, false);
        needsFullRedraw_ = false;
    }

    if(!needsContentsRedraw_) return RES_OK;

    RDisplay::display.circle(x, y, rad, fgCol_, false);

    float r = rad-3;

    int textx = x - (strlen(buf_) * RDisplay::CHAR_WIDTH)/2;
    int texty = y - RDisplay::CHAR_HEIGHT/2;

    // erase old stuff
    if(showsText_) RDisplay::display.text(textx, texty, bgCol_, buf_);
    RDisplay::display.line(headingX1Old_, headingY1Old_, headingX2Old_, headingY2Old_, bgCol_);
    RDisplay::display.line(horizX1Old_, horizY1Old_, horizX2Old_, horizY2Old_, bgCol_);


    // horizon line
    // y = ax+b
    float a = tan(DEG_TO_RAD*heading_);
    float b = -sin(DEG_TO_RAD*pitch_) * r;

    // circle equation: r^2 = x^2+y^2
    // put y=ax+b into this and multiply out
    float p = (2*a*b)/(1+a*a);
    float q = (b*b-r*r)/(1+a*a);

    // quadratic formula
    float xinters1 = -1*(p/2)-sqrt((p*p)/4-q);
    float xinters2 = -1*(p/2)+sqrt((p*p)/4-q);

    // line equation
    float yinters1 = a*xinters1+b;
    float yinters2 = a*xinters2+b;

    int horizX1 = x + xinters1;
    int horizY1 = y + yinters1;
    int horizX2 = x + xinters2;
    int horizY2 = y + yinters2;

    RDisplay::display.line(horizX1, horizY1, horizX2, horizY2, cursorCol_);
    horizX1Old_ = horizX1; horizY1Old_ = horizY1;
    horizX2Old_ = horizX2; horizY2Old_ = horizY2;

    float headingposx = sin(DEG_TO_RAD*roll_)*cos(DEG_TO_RAD*heading_)*r;
    float headingposy = a*headingposx + b;

    // hy = a'hx+b' ==> b = hy - a'hx
    float ainv = -1/a;
    float binv = headingposy - ainv*headingposx;

    // quadratic formula again
    p = (2*ainv*binv)/(1+ainv*ainv);
    q = (binv*binv-r*r)/(1+ainv*ainv);

    // quadratic formula
    xinters1 = -1*(p/2)-sqrt((p*p)/4-q);
    xinters2 = -1*(p/2)+sqrt((p*p)/4-q);
    yinters1 = ainv*xinters1 + binv;
    yinters2 = ainv*xinters2 + binv;

    if(!isnan(xinters1) && !isnan(xinters2)) {
        int headingX1, headingX2, headingY1, headingY2;
        if(yinters2 > yinters1) {
            headingX1 = x + xinters2;
            headingY1 = y + yinters2;
        } else {
            headingX1 = x + xinters1;
            headingY1 = y + yinters1;
        }
        headingX2 = x + headingposx;
        headingY2 = y + headingposy;
        RDisplay::display.line(headingX1, headingY1, headingX2, headingY2, cursorCol_);
        headingX1Old_ = headingX1; headingY1Old_ = headingY1;
        headingX2Old_ = headingX2; headingY2Old_ = headingY2;
    }

    if(showsAccel_) {
        int axisX = x, axisY = y-rad/3;
        int axisMax = rad/2;
        
        RDisplay::display.line(axisX, axisY, axOld_, axisY, bgCol_);
        RDisplay::display.line(axisX, axisY, axisX, ayOld_, bgCol_);
        RDisplay::display.line(axisX, axisY, azOld_+axisX, axisY-azOld_, bgCol_);
        int ax = constrain(accelX_, -1.0, 1.0)*axisMax + axisX;
        int ay = constrain(accelY_, -1.0, 1.0)*axisMax + axisY;
        int az = (constrain(accelZ_, -1.0, 1.0)/1.414)*axisMax;
        RDisplay::display.line(axisX, axisY, ax, axisY, RDisplay::LIGHTRED2);
        RDisplay::display.line(axisX, axisY, axisX, ay, RDisplay::LIGHTGREEN2);
        RDisplay::display.line(axisX, axisY, az+axisX, axisY-az, RDisplay::LIGHTBLUE2);
        axOld_ = ax;
        ayOld_ = ay;
        azOld_ = az;
    }

    if(showsText_) {
        snprintf(buf_, BUFLEN, "R%d P%d H%d", int(roll_), int(pitch_), int(heading_));
        textx = x - (strlen(buf_) * RDisplay::CHAR_WIDTH)/2;
        texty = y - RDisplay::CHAR_HEIGHT/2;
        RDisplay::display.text(textx, texty, fgCol_, buf_);
    }

    return RES_OK;
}   

void RIMUWidget::setRPH(float r, float p, float h) {
    roll_ = -r;
    pitch_ = -p;
    heading_ = h;

    //Console::console.printfBroadcast("R:%.2f P:%.2f H:%.2f\n", r, p, h);

    setNeedsContentsRedraw();
}

void RIMUWidget::setAccel(float x, float y, float z) {
    accelX_ = x;
    accelY_ = y;
    accelZ_ = z;
    //Console::console.printfBroadcast("Ax:%f Ay:%f Az:%f\n", accelX_, accelY_, accelZ_);
    setNeedsContentsRedraw();
}

void RIMUWidget::setShowsText(bool shows) {
    showsText_ = shows;
    if(showsText_ == false) setNeedsFullRedraw(); // so that we paint over any residual text
    setNeedsContentsRedraw();
}

void RIMUWidget::setShowsAccel(bool shows) {
    showsAccel_ = shows;
    setNeedsContentsRedraw();
}
