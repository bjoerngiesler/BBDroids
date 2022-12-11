#include "Config.h"
#include "RemoteState.h"
#include "RemoteDisplay.h"
#include "WifiComm.h"
#include "StatePacket.h"
#include "IMUFilter.h"
#include "BB8Packet.h"

RemoteState *state;
RemoteDisplay *disp;
WifiComm *comm;
IMUFilter imu;

uint8_t seqnum = 0;

unsigned long last_millis_;

void setup() {
  Serial.begin(2000000);
  Serial.println();
  Serial.println("BB8 Remote");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  disp = new RemoteDisplay;
  state = RemoteState::getSharedInstance();
  comm = new WifiComm;
  imu.begin();

  last_millis_ = millis();

  Serial.println("Entering main loop.");
}

bool connectToUDPServer() {
  int i;

  Serial.print("Trying to connect to Access Point...");
  for(i=0; i<100; i++) {
    if(comm->isConnected()) break;
    Serial.print(".");
    comm->tryToConnect();
    delay(10);
  }
  if(i==100) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }

  disp->setConnected(comm->isConnected());
  return true;
}

void runEverySecond() {
}

void loop() {
  if(millis() - last_millis_ > 1000) {
    runEverySecond();
    last_millis_ = millis();
  }
  
  state->update();
  state->printOnSerial();

  if(imu.available()) {
    float roll, pitch, yaw;
    imu.update();
    imu.getRawEulerAngles(roll, pitch, yaw);
  }

  if(comm->isConnected()) {
    BB8CommandPacket packet;
    packet.seqnum = seqnum++;
    packet.cmd = CMD_SET_ALL_MOTORS_FLAGS_NR;
    packet.arg.flagFloatListArg.flags = 0;
      
    packet.arg.flagFloatListArg.flags |= DRIVE_MOTOR_FLAG;
    packet.arg.flagFloatListArg.flags |= TURN_MOTOR_FLAG;
    packet.arg.flagFloatListArg.param[DRIVE_MOTOR_INDEX] = 0.0f;
    packet.arg.flagFloatListArg.param[TURN_MOTOR_INDEX] = 0.0f;
    
    if(state->isTopRightButtonPressed() && !state->isTopLeftButtonPressed()) {
      Serial.println("Drive config - Forward & Curve");
      packet.arg.flagFloatListArg.param[DRIVE_MOTOR_INDEX] = state->getJoystickVerticalAxis();
      packet.arg.flagFloatListArg.flags |= SERVO_4_FLAG;
      packet.arg.flagFloatListArg.param[SERVO_4_INDEX] = 180.0f + state->getJoystickHorizontalAxis() * 30.0f;
      Serial.print(packet.arg.flagFloatListArg.param[SERVO_4_INDEX]);
    } else if(state->isTopLeftButtonPressed() && !state->isTopRightButtonPressed()) {
      Serial.println("Spot turn");
      packet.arg.flagFloatListArg.param[TURN_MOTOR_INDEX] = state->getJoystickHorizontalAxis();
    } else if(state->isTopLeftButtonPressed() && state->isTopRightButtonPressed()) {
      Serial.println("Servos");
      packet.arg.flagFloatListArg.flags |= SERVO_3_FLAG;
      packet.arg.flagFloatListArg.param[SERVO_3_INDEX] = 180.0f + state->getJoystickVerticalAxis() * 30.0f;
      packet.arg.flagFloatListArg.flags |= SERVO_2_FLAG;
      packet.arg.flagFloatListArg.param[SERVO_2_INDEX] = 180.0f + state->getJoystickHorizontalAxis() * 30.0f;
    }

    comm->broadcastUDPPacket((uint8_t*)&packet, sizeof(packet));
  } else {
      connectToUDPServer();
  }

  disp->update();

  delay(25);
}
