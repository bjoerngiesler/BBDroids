#include "Config.h"
#include "WifiServer.h"
#include "StatePacket.h"
#include "DCMotor.h"
#include "wiring_private.h"


WifiServer *server;
DCMotor *driveMotor, *yawMotor;
Uart *dynamixelSerial, *dfplayerSerial;


unsigned long last_millis_;

StatePacket packet;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BB8 Main Board");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  pinMode(LED_BUILTIN, OUTPUT);

  server = NULL;
  driveMotor = NULL;
  yawMotor = NULL;
  dynamixelSerial = NULL;
  dfplayerSerial = NULL;

  setupBoardComm();
  setupUDPServer();
  setupMotors();

  last_millis_ = millis();

  Serial.println("Entering main loop.");
}

bool setupBoardComm() {
  dynamixelSerial = new Uart(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  dfplayerSerial = &Serial1;
}

bool setupUDPServer() {
  server = new WifiServer;
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

  return true;
}

bool setupMotors() {
  Serial.println("Setting up motors.");
  driveMotor = new DCMotor(PO_EN_DRIVE, PO_A_DRIVE, PO_B_DRIVE, PO_PWM_DRIVE);
  yawMotor = new DCMotor(PO_EN_YAW, PO_A_YAW, PO_B_YAW, PO_PWM_YAW);
  return true;
}

void runEverySecond() {
}

void setMotorSpeedByJoystickAxis(DCMotor *motor, uint16_t axis) {
  uint8_t speed;
  DCMotor::Direction dir;
  
  if(axis >= DEADBAND_MIN && axis <= DEADBAND_MAX) {
    dir = DCMotor::DCM_IDLE;
    speed = 0;
  } else if(axis < DEADBAND_MIN) {
    dir = DCMotor::DCM_BACKWARD;
    speed = map(axis, 0, 511, 255, 0);
  } else {
    dir = DCMotor::DCM_FORWARD;
    speed = map(axis, 512, 1024, 0, 255);
  }

  Serial.print("dir: "); Serial.print(dir);
  Serial.print(" speed: "); Serial.println(speed);
  if(speed == 0) dir = DCMotor::DCM_IDLE;
  motor->setDirectionAndSpeed(dir, speed);
}

void loop() {
  if(millis() - last_millis_ > 1000) {
    runEverySecond();
    last_millis_ = millis();
  }

  if(server->isUDPServerStarted()) {
    digitalWrite(LED_BUILTIN, HIGH);
    
    if(!driveMotor->isEnabled()) driveMotor->setEnabled(true);
    if(!yawMotor->isEnabled()) yawMotor->setEnabled(true);
    
    int bytes_read = server->readDataIfAvailable((uint8_t*)&packet, sizeof(packet));
    if(bytes_read == 0) {
      // ignore
    } else if(bytes_read == sizeof(packet)) {
      Serial.print("Packet received! Seqnum: ");
      Serial.print(packet.sequence_num_);
      Serial.print(" X: "); Serial.print(packet.joystick_horizontal_);
      Serial.print(" Y: "); Serial.print(packet.joystick_vertical_);
      Serial.println(); 

      setMotorSpeedByJoystickAxis(driveMotor, packet.joystick_vertical_);
      setMotorSpeedByJoystickAxis(yawMotor, packet.joystick_horizontal_);
    } else {
      Serial.print("Unknown packet size ");
      Serial.println(bytes_read);
    }
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    if(driveMotor->isEnabled()) driveMotor->setEnabled(false);
    if(yawMotor->isEnabled()) yawMotor->setEnabled(false);
  }

  delay(1);
}
