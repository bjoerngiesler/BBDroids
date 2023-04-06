#include <LibBB.h>

#include "BB8.h"
#include "BB8Config.h"
#include "BB8StatusPixels.h"
#include "BB8Packet.h"
#include "BB8DCMotor.h"
#include "BB8Sound.h"
#include "BB8IMU.h"
#include "BB8Servos.h"
#include "BB8Controllers.h"
#include <math.h>
#include <limits.h>
#include <wiring_private.h>

#include <Encoder.h>
#include <WiFiNINA.h>

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;
int16_t driveTicks;


bool networkOK;
PlotMode plotMode;

Encoder driveEncoder(P_DRIVEENC_A, P_DRIVEENC_B);

unsigned long last_millis_;

using namespace bb;

void setup() {
  Serial.begin(2000000);
  Serial.println(); 
  Serial.println("BB8 Main Board");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  dynamixelSerial = NULL;
  dfplayerSerial = NULL;
  serialTXSerial = NULL;

  state.seqnum = 0;
  driveTicks = 0;

  setupBoardComm();

  bb::Runloop::runloop.initialize();
  bb::Console::console.initialize();

  bb::XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, DEFAULT_STATION_DROID, DEFAULT_STATION_LEFT_REMOTE, DEFAULT_BPS, serialTXSerial);
  bb::WifiServer::server.initialize("BB8-$MAC", DEFAULT_WPAKEY, DEFAULT_APMODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  BB8Servos::servos.initialize();
  BB8::bb8.initialize();

  bb::XBee::xbee.setPacketMode(true);
  bb::XBee::xbee.addPacketReceiver(&BB8::bb8);

  bb::WifiServer::server.start();
  bb::XBee::xbee.start();
  bb::Console::console.start();
  BB8::bb8.start();
  BB8Servos::servos.start();

  BB8StatusPixels::statusPixels.begin();
  BB8Sound::sound.begin(dfplayerSerial);
  BB8PIDController::rollController.begin();
  
  last_millis_ = millis();

  bb::Runloop::runloop.start(); // never returns
}

bool setupBoardComm() {
  dynamixelSerial = &Serial1;

  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

  dfplayerSerial = new Uart(&sercom1, P_DFPLAYER_RX, P_DFPLAYER_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_DFPLAYER_RX, PIO_SERCOM);
  pinPeripheral(P_DFPLAYER_TX, PIO_SERCOM);

  return true;
}

void SERCOM1_Handler() {
  dfplayerSerial->IrqHandler();
}

void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}

void printStatus() {
  int16_t x, y, z;
  if(state.imusOK == true) {  
    #if 0
    if(BB8IMU::dome.readVector(x, y, z)) {
      Serial.print("Dome IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.print(" ");
    }
    if(BB8IMU::body.readVector(x, y, z)) {
      Serial.print("Body IMU: "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
    }
    #endif
  } else {
    Serial.print("IMUs not initialized ");
  }

  Serial.print("Drive Encoder: "); Serial.println(driveEncoder.read());  

  if(state.servosOK) {
    Serial.print("Servo positions: ");
    for(int i=1; i<=4; i++) {
      Serial.print(i); Serial.print(": "); Serial.print(state.servo[i-1]); Serial.print(" ");
    }
  } else {
    Serial.print("Servos not initialized");
  }

  Serial.println();
}

void runEverySecond() {
  unsigned long m = millis();

  m = millis();
  if(networkOK) BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_NETWORK, BB8StatusPixels::STATUS_OK);
  else BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_NETWORK, BB8StatusPixels::STATUS_FAIL);
  if(state.motorsOK && state.servosOK) BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_MOTORS, BB8StatusPixels::STATUS_OK);
  else BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_MOTORS, BB8StatusPixels::STATUS_FAIL);
  Serial.print("Pixels:"); Serial.println(millis()-m);

  m = millis();
  printStatus();
  Serial.print("Status:"); Serial.println(millis()-m);
}

void setMotorSpeedByJoystickAxis(BB8DCMotor &motor, float axis) {
  uint8_t speed;
  BB8DCMotor::Direction dir;
  
  if(abs(axis) <= DEADBAND) {
    dir = BB8DCMotor::DCM_IDLE;
    speed = 0;
  } else if(axis < -DEADBAND) {
    dir = BB8DCMotor::DCM_BACKWARD;
    speed = (uint8_t)(-255.0f * axis);
  } else {
    dir = BB8DCMotor::DCM_FORWARD;
    speed = (uint8_t)(255.0f * axis);
  }

  motor.setDirectionAndSpeed(dir, speed);
}

void loop() {
#if 0
  unsigned long millis_start_loop = millis();

  //BB8BodyIMU::imu.step(plotMode == PLOT_BODY_IMU);
  //BB8PIDController::rollController.step(plotMode == PLOT_ROLL_CONTROLLER);

  if(millis_start_loop - last_millis_ > 1000) {
    //runEverySecond();
    last_millis_ = millis();
  }

  //sendUDPState();

  unsigned long millis_end_loop = millis();
  unsigned long looptime;
  if(millis_end_loop >= millis_start_loop)
    looptime = millis_end_loop-millis_start_loop;
  else
    looptime = ULONG_MAX - millis_start_loop + millis_end_loop;
  if(looptime < CYCLETIME) {
    //Serial.println(looptime);
    delay(CYCLETIME-looptime);
  } else {
    Serial.print(looptime); Serial.println("ms spent in loop - something is wrong!");
  }
#endif
}
