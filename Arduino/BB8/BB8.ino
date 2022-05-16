#include "Config.h"
#include "WifiServer.h"
#include "StatePacket.h"
#include "DCMotor.h"
#include "BB8Sound.h"
#include "BB8IMU.h"
#include "BB8SerialTX.h"
#include <wiring_private.h>
#include <Dynamixel_Servo.h>
#include <Encoder.h>

WifiServer *server;
DCMotor *driveMotor, *yawMotor;
Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;
int16_t driveTicks;

BB8Sound sound;
BB8IMU domeIMU, bodyIMU;
BB8SerialTX serialTX;
Encoder driveEncoder(P_DRIVEENC_A, P_DRIVEENC_B);

unsigned long last_millis_;

StatePacket packet;

void setup() {
  Serial.begin(115200);
  while(!Serial);
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
  setupUDPServer();
  setupMotors();
  setupDynamixels();
  setupIMUs();
  
  sound.begin(*dfplayerSerial);

  last_millis_ = millis();

  Serial.println("Entering main loop.");
}

bool setupBoardComm() {
  dynamixelSerial = &Serial1;

  #if 1
  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);
  #else
  serialTXSerial = &Serial1;
  #endif
  
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
  driveMotor = new DCMotor(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM);
  yawMotor = new DCMotor(P_YAW_A, P_YAW_B, P_YAW_PWM);
  DCMotor::setEnablePin(P_MOT_EN);
  DCMotor::setEnabled(true);
  return true;
}

bool setupDynamixels() {
  Serial.println("Setting up Dynamixels.");

  servo_init(dynamixelSerial, P_DYNAMIXEL_RTS, SERVO_DEFAULT_BAUD);
  
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
  return success;
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

  int16_t x, y, z;
  domeIMU.readVector(x, y, z);
  Serial.print("Dome IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
  bodyIMU.readVector(x, y, z);
  Serial.print("Body IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);

  Serial.print("Drive Encoder: "); Serial.println(driveEncoder.read());
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

  if(serialTX.available()) {
    serialTX.read();
  }

  delay(1);
}
