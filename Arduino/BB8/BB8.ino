#include "BB8Config.h"
#include "BB8StatusPixels.h"
#include "BB8WifiServer.h"
#include "BB8Packet.h"
#include "BB8DCMotor.h"
#include "BB8Sound.h"
#include "BB8IMU.h"
#include "BB8SerialTX.h"
#include "BB8ConfigStorage.h"
#include "BB8Servos.h"
#include "BB8SerialCommands.h"
#include "BB8Controllers.h"
#include <math.h>
#include <limits.h>
#include <wiring_private.h>

#include <Encoder.h>
#include <WiFiNINA.h>

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;
int16_t driveTicks;

BB8SerialTX serialTX;
BB8DCMotor driveMotor(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM, P_DRIVE_EN);
BB8DCMotor yawMotor(P_YAW_A, P_YAW_B, P_YAW_PWM, P_YAW_EN);

bool networkOK;
PlotMode plotMode;

Encoder driveEncoder(P_DRIVEENC_A, P_DRIVEENC_B);

unsigned long last_millis_;

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

  BB8ConfigStorage::storage.begin();
  BB8StatusPixels::statusPixels.begin();
  BB8Sound::sound.begin(dfplayerSerial);
  BB8WifiServer::server.begin();
  BB8BodyIMU::imu.begin(CYCLETIME);
  BB8PIDController::rollController.begin(BB8ConfigStorage::ROLL_CONTROLLER);
  state.servosOK = BB8Servos::servos.begin();

  
  BB8SerialCommands::sercmd.begin();

  //setupDynamixels();

  last_millis_ = millis();

  Serial.println("Entering main loop. Enter 'help' for serial commands.");
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

void controlBySerialTX() {
  if(!driveMotor.isEnabled()) driveMotor.setEnabled(true);
  if(!yawMotor.isEnabled()) yawMotor.setEnabled(true);

  serialTX.printStatus();
  for(int i=1; i<=3; i++) {
    float deg = (float)map(serialTX.channelValue(i), -511, 512, 0, 360);
    BB8Servos::servos.setGoalPosition(i+1, deg);
  }
}

void sendUDPState() {
  if(!BB8WifiServer::server.isUDPServerStarted()) return;

  if(state.imusOK) {
    //BB8IMU::dome.readVector(state.domeimuX, state.domeimuY, state.domeimuZ);
    //BB8IMU::body.readVector(state.bodyimuX, state.bodyimuY, state.bodyimuZ);
  }

  if(state.servosOK) {
    for(int i=1; i<=4; i++) {
      state.servo[i-1] = BB8Servos::servos.getPresentPosition(i);      
    }
  }

  if(!BB8WifiServer::server.sendStatePacket(state)) {
    Serial.println("Sending state failed. Shutting down Wifi.");
    BB8WifiServer::server.shutdown();
  }
  state.seqnum++;
}

void sendDroidName(const IPAddress& remoteIP) {
  BB8WifiServer::server.sendCommandReply(remoteIP, (uint8_t*)"Hello", 5);
}

void handleUDPCommand() {
  BB8CommandPacket cmd;

  IPAddress remoteIP;
  digitalWrite(LED_BUILTIN, HIGH);
  if(!driveMotor.isEnabled()) driveMotor.setEnabled(true);
  if(!yawMotor.isEnabled()) yawMotor.setEnabled(true);
 
  while(BB8WifiServer::server.readCommandPacketIfAvailable(cmd, remoteIP)) {    
    Serial.print("Packet received! Seqnum: ");
    Serial.print(cmd.seqnum);

    Serial.print(" Command: ");
    Serial.println(cmd.cmd);

    switch(cmd.cmd) {
      case CMD_SET_SERVO_NR:
        if(state.servosOK) {
          Serial.print("Setting servo ");
          Serial.print(cmd.arg.indexedFloatArg.index);
          Serial.print(" to ");
          Serial.println(cmd.arg.indexedFloatArg.param);
          BB8Servos::servos.setGoalPosition(cmd.arg.indexedFloatArg.index+1, cmd.arg.indexedFloatArg.param);
        }    
        break;

      case CMD_SET_ALL_MOTORS_FLAGS_NR:
        if(state.servosOK) {
          Serial.print("Setting servos and motors... ");
          for(int i=0; i<4; i++) {
            if(cmd.arg.flagFloatListArg.flags & (1<<i)) {
              Serial.print("S"); Serial.print(i+1); Serial.print(": "); Serial.print(cmd.arg.flagFloatListArg.param[i]); Serial.print(" ");                        
              BB8Servos::servos.setGoalPosition(i+1, cmd.arg.indexedFloatArg.param);
            }
          }
        }

        if(state.motorsOK) {
          if(cmd.arg.flagFloatListArg.flags & DRIVE_MOTOR_FLAG) {
            Serial.print("D"); Serial.print(cmd.arg.flagFloatListArg.param[DRIVE_MOTOR_INDEX]); Serial.print(" ");
            setMotorSpeedByJoystickAxis(driveMotor, cmd.arg.flagFloatListArg.param[DRIVE_MOTOR_INDEX]);
          }
          if(cmd.arg.flagFloatListArg.flags & TURN_MOTOR_FLAG) {
            Serial.print("T"); Serial.print(cmd.arg.flagFloatListArg.param[TURN_MOTOR_INDEX]); Serial.print(" ");
            setMotorSpeedByJoystickAxis(yawMotor, cmd.arg.flagFloatListArg.param[TURN_MOTOR_INDEX]);
          }
        }
        break;

      case CMD_GET_DROID_NAME:
        sendDroidName(remoteIP);
        break;
    }
  }
}

void loop() {
  unsigned long millis_start_loop = millis();

  BB8BodyIMU::imu.step(plotMode == PLOT_BODY_IMU);
  BB8PIDController::rollController.step(plotMode == PLOT_ROLL_CONTROLLER);

  BB8SerialCommands::sercmd.handleIfAvailable();

  // SerialTX overrides UDP
  if(serialTX.read()) {
    controlBySerialTX();
  } else if(BB8WifiServer::server.isUDPServerStarted()) {
    handleUDPCommand();
  } 
  
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
  if(looptime < CYCLETIME)
    delay(CYCLETIME-looptime);
  else {
    Serial.print(looptime); Serial.println("ms spent in loop - something is wrong!");
  }
}
