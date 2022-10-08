#include "BB8Config.h"
#include "BB8WifiServer.h"
#include "BB8StatePacket.h"
#include "BB8DCMotor.h"
#include "BB8Sound.h"
#include "BB8IMU.h"
#include "BB8SerialTX.h"
#include <wiring_private.h>
#include <DynamixelShield.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;
int16_t driveTicks;

BB8WifiServer *server;
BB8DCMotor *driveMotor, *yawMotor;
BB8Sound sound;
BB8IMU domeIMU, bodyIMU;
BB8SerialTX serialTX;
BB8StatePacket packet;

bool imusOK;
bool networkOK;
bool motorsOK;
bool dynamixelsOK;


DynamixelShield dxl;
Encoder driveEncoder(P_DRIVEENC_A, P_DRIVEENC_B);
Adafruit_NeoPixel statusPixel(3, P_STATUS_NEOPIXEL, NEO_GRB + NEO_KHZ800);

unsigned long last_millis_;

void setup() {
  Serial.begin(115200);
  Serial.println(); 
  Serial.println("BB8 Main Board");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  server = NULL;
  driveMotor = NULL;
  yawMotor = NULL;
  dynamixelSerial = NULL;
  dfplayerSerial = NULL;
  serialTXSerial = NULL;

  driveTicks = 0;

  setupBoardComm();
  setupNeopixels();
  sound.begin(*dfplayerSerial);
  
  setupUDPServer();
  setupMotors();
  setupDynamixels();
  setupIMUs();

  last_millis_ = millis();

  Serial.println("Entering main loop.");
}

bool setupBoardComm() {
  dynamixelSerial = &Serial1;

  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

  dfplayerSerial = new Uart(&sercom1, P_DFPLAYER_RX, P_DFPLAYER_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_DFPLAYER_RX, PIO_SERCOM);
  pinPeripheral(P_DFPLAYER_TX, PIO_SERCOM);

  return serialTX.begin(serialTXSerial);
}

void SERCOM1_Handler() {
  dfplayerSerial->IrqHandler();
}

void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}

bool setupNeopixels() {
  statusPixel.begin();
  statusPixel.clear();
  statusPixel.setPixelColor(STATUSPIXEL_OVERALL, statusPixel.Color(150, 150, 0));
  statusPixel.setPixelColor(STATUSPIXEL_NETWORK, statusPixel.Color(150, 150, 0));
  statusPixel.setPixelColor(STATUSPIXEL_MOTORS, statusPixel.Color(150, 150, 0));
  statusPixel.show();
}

bool setupUDPServer() {
  networkOK = false;
  
  server = new BB8WifiServer;
  int i;

  Serial.print("Trying to start Access Point...");
  for(i=0; i<100; i++) {
    if(server->isAPStarted()) break;
    Serial.print(".");
    server->tryToStartAP();
    delay(10);
  }
  if(i==100) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }
  
  Serial.print("Trying to start UDP Server...");
  for(i=0; i<100; i++) {
    if(server->isUDPServerStarted()) break;
    Serial.print(".");
    server->startUDPServer();
    delay(10);
  }
  if(i==100) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }

  networkOK = true;
  return true;
}

bool setupMotors() {
  Serial.println("Setting up motors.");

  motorsOK = false;
  
  driveMotor = new BB8DCMotor(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM);
  yawMotor = new BB8DCMotor(P_YAW_A, P_YAW_B, P_YAW_PWM);
  BB8DCMotor::setEnablePin(P_DRIVE_EN);
  BB8DCMotor::setEnabled(true);

  motorsOK = true;
  return true;
}

bool setupDynamixels() {
  Serial.print("Setting up Dynamixels... ");
  dynamixelsOK = false;
  bool retval = true;

  dxl.begin(DYNAMIXEL_BPS);
  for(int i=1; i<=4; i++) {
    if(dxl.ping(i)) {
      Serial.print("#"); Serial.print(i); Serial.print(": model #"); Serial.print(dxl.getModelNumber(i)); Serial.print("... ");
    } else {
      Serial.print("#"); Serial.print(i); Serial.print(" not found! ");
      retval = false;    
    }
  }
  
  if(!retval) {
    Serial.println("failed!");
    return false;
  }
    
  Serial.println("success.");

  for(int i=1; i<4; i++) {
    if(!dxl.torqueOn(i)) { Serial.print("Couldn't turn on torque on #"); Serial.println(i); }
    dxl.setGoalPosition(i, 180.0, UNIT_DEGREE);
  }

  dynamixelsOK = true;
  return true;
}

bool setupIMUs() {
  Serial.print("Setting up IMUs - dome... ");
  bool success = true;

  if(!domeIMU.begin(DOME_IMU_ADDR)) {
    success = false;
    Serial.print("failed! ");
  } else {
    Serial.print("ok, ");
  }
  Serial.print("body... ");
  if(!bodyIMU.begin(BODY_IMU_ADDR)) {
    success = false;
    Serial.print("failed!");
  } else {
    Serial.print("ok.");
  }
  Serial.println();
  if(success) {
    imusOK = true;
    domeIMU.printStats("Dome IMU:");
    bodyIMU.printStats("Body IMU:");
    return true;
  } else {
    imusOK = false;
    return false;
  }
}

void printStatus() {
  int16_t x, y, z;
  if(domeIMU.readVector(x, y, z)) {
    Serial.print("Dome IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
  }
  if(bodyIMU.readVector(x, y, z)) {
    Serial.print("Body IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
  }

  Serial.print("Drive Encoder: "); Serial.println(driveEncoder.read());  

  if(dynamixelsOK) {
    Serial.print("Servo positions: ");
    for(int i=1; i<=4; i++) {
      float pos = dxl.getPresentPosition(i, UNIT_DEGREE);
      Serial.print(i); Serial.print(": "); Serial.print(pos); Serial.print(" ");
    }
    Serial.println();
  }
}

void runEverySecond() {
#if 0
  static bool on = false;
  if(on) {
    Serial.println("On");
    servo_set(SERVO_BROADCAST_ID, SERVO_REGISTER_LED_IS_ON, 0.0, 100);
    on = false;
  } else {
    Serial.println("Off");
    servo_set(SERVO_BROADCAST_ID, SERVO_REGISTER_LED_IS_ON, 1.0, 100);
    on = true; 
  }
#endif

  if(networkOK) statusPixel.setPixelColor(STATUSPIXEL_NETWORK, statusPixel.Color(0, 150, 0));
  else statusPixel.setPixelColor(STATUSPIXEL_NETWORK, statusPixel.Color(150, 0, 0));
  if(motorsOK && dynamixelsOK) statusPixel.setPixelColor(STATUSPIXEL_MOTORS, statusPixel.Color(0, 150, 0));
  else statusPixel.setPixelColor(STATUSPIXEL_MOTORS, statusPixel.Color(150, 0, 0));
  statusPixel.show();

  printStatus();
}

void setMotorSpeedByJoystickAxis(BB8DCMotor *motor, int16_t axis) {
  uint8_t speed;
  BB8DCMotor::Direction dir;
  
  if(axis >= DEADBAND_MIN && axis <= DEADBAND_MAX) {
    dir = BB8DCMotor::DCM_IDLE;
    speed = 0;
  } else if(axis < DEADBAND_MIN) {
    dir = BB8DCMotor::DCM_BACKWARD;
    speed = map(axis, -511, 0, 255, 0);
  } else {
    dir = BB8DCMotor::DCM_FORWARD;
    speed = map(axis, 0, 512, 0, 255);
  }

  Serial.print("dir: "); Serial.print(dir);
  Serial.print(" speed: "); Serial.println(speed);
  if(speed == 0) dir = BB8DCMotor::DCM_IDLE;
  motor->setDirectionAndSpeed(dir, speed);
}

void balanceControl() {
  if(!imusOK) return;

  int16_t x, y, z;

  domeIMU.readVector(x, y, z);
  float deg = (atan2(z, y) * 180.0)/M_PI;
  float setDeg = 180.0 + 90.0 - deg;
  float curDeg = dxl.getPresentPosition(3, UNIT_DEGREE);
  float damp = .5;
  float step = (setDeg - curDeg) / damp;
  float curGoal = curDeg + step;
  Serial.print("Set: "); Serial.print(setDeg);
  Serial.print(" Cur: "); Serial.print(curDeg);
  Serial.print(" Step: "); Serial.print(step);
  Serial.print(" CurGoal: "); Serial.println(curGoal);
  
  if(!dxl.setGoalPosition(3, curGoal, UNIT_DEGREE)) { Serial.println("Failure"); }
}

void controlBySerialTX() {
  if(!driveMotor->isEnabled()) driveMotor->setEnabled(true);
  if(!yawMotor->isEnabled()) yawMotor->setEnabled(true);

  serialTX.printStatus();
  for(int i=1; i<=3; i++) {
    float deg = (float)map(serialTX.channelValue(i), -511, 512, 0, 360);
    dxl.torqueOn(i+1);

    dxl.setGoalPosition(i+1, deg, UNIT_DEGREE);
  }
}

void controlByUDPClient() {
  digitalWrite(LED_BUILTIN, HIGH);
  if(!driveMotor->isEnabled()) driveMotor->setEnabled(true);
  if(!yawMotor->isEnabled()) yawMotor->setEnabled(true);
      
  int bytes_read = server->readDataIfAvailable((uint8_t*)&packet, sizeof(packet));
  if(bytes_read == 0) return;
  if(bytes_read != sizeof(packet)) {
    Serial.print("Unknown packet size ");
    Serial.println(bytes_read);
    return;
  }
    
  Serial.print("Packet received! Seqnum: ");
  Serial.print(packet.sequence_num_);
  Serial.print(" X: "); Serial.print(packet.joystick_horizontal_);
  Serial.print(" Y: "); Serial.print(packet.joystick_vertical_);
  Serial.println(); 

  setMotorSpeedByJoystickAxis(driveMotor, packet.joystick_vertical_);
  setMotorSpeedByJoystickAxis(yawMotor, packet.joystick_horizontal_);
}

void loop() {
  if(millis() - last_millis_ > 1000) {
    runEverySecond();
    last_millis_ = millis();
  }

  // SerialTX overrides UDP
  if(serialTX.available() && serialTX.read()) {
    controlBySerialTX();
  } else if(server->isUDPServerStarted()) {
    controlByUDPClient();
  }
  //rbalanceControl();

  delay(1);
}
