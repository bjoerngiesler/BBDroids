#include "Config.h"
#include "WifiServer.h"
#include "StatePacket.h"
#include "MotorControl.h"

WifiServer *server;
MotorControl *control;

unsigned long last_millis_;

StatePacket packet;


void setup() {
  Serial.begin(115200);
  
  server = new WifiServer;
  control = new MotorControl;

  last_millis_ = millis();
}

void runEverySecond() {
  if(!server->isAPStarted()) {
      Serial.println("Trying to start AP");
      server->tryToStartAP();
      if(server->isAPStarted()) {
        server->startUDPServer();        
      }
  }
}

void loop() {
  if(millis() - last_millis_ > 1000) {
    runEverySecond();
    last_millis_ = millis();
  }

  if(server->isUDPServerStarted()) {
    int bytes_read = server->readDataIfAvailable((uint8_t*)&packet, sizeof(packet));
    if(bytes_read == 0) {
      // ignore
    } else if(bytes_read == sizeof(packet)) {
      Serial.print("Packet received! Seqnum: ");
      Serial.print(packet.sequence_num_);
      Serial.print("X: "); Serial.print(packet.joystick_horizontal_);
      Serial.print("Y: "); Serial.print(packet.joystick_vertical_);
      Serial.println(); 

      float speed = map(packet.joystick_vertical_, 0, 1024, -DRIVE_STEPPER_MAX_SPEED, DRIVE_STEPPER_MAX_SPEED);
      if(abs(speed) < 100) 
        control->setDriveSpeed(0);
      else 
        control->setDriveSpeed(speed);      
    } else {
      Serial.print("Unknown packet size ");
      Serial.println(bytes_read);
    }
  }

  control->control();
  delay(1);
}
