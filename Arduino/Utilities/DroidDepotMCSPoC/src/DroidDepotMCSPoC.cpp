/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

static BLEUUID SERVICE_UUID("09b600A0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID NOTIFICATION_UUID("09b600b0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID WRITE_UUID("09b600b1-3e42-41fc-b474-e9c0c8f0c801");

static uint8_t P_A_JOY_VER        = A8;
static uint8_t P_A_JOY_HOR        = A3;
static uint8_t MCP_ADDR           = 0x26;
static uint8_t BUTTON_PIN_1       = 5;
static uint8_t BUTTON_PIN_2       = 4;
static uint8_t BUTTON_PIN_3       = 3;
static uint8_t BUTTON_PIN_4       = 2;
static uint8_t BUTTON_PIN_JOY     = 1;
static uint8_t BUTTON_PIN_CONFIRM = 7;
static uint8_t BUTTON_PIN_LEFT    = 6;
static uint8_t BUTTON_PIN_RIGHT   = 0;

int scanTime = 5; //In seconds
static BLEScan* pBLEScan;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteCharacteristic;
bool doConnect = false, connected = false;

float calibVer = 0.0f, calibHor = 0.0f;

Adafruit_MCP23X17 mcp;
bool buttons[8];

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.printf("Connected\n");
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.println("Disconnected");
  }
};

void initialWrites() {
  Serial.printf("Doing initial writes\n");
  for(int i=0; i<4; i++) {
    pRemoteCharacteristic->writeValue({0x22, 0x20, 0x01}, 3);
    delay(500);
  }
  for(int i=0; i<2; i++) {
    pRemoteCharacteristic->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x1f, 0x00}, 8);
    delay(500);
    pRemoteCharacteristic->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x18, 0x02}, 8);
    delay(500);
  }
  Serial.printf("Initial writes done\n");
}

bool connectToServer() {
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(46); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(SERVICE_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(WRITE_UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(WRITE_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");
    
    initialWrites();

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Found node %s \n", advertisedDevice.toString().c_str());
//      if(advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      if(advertisedDevice.getName() == "DROID") {
        Serial.printf("Found droid!\n");
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(P_A_JOY_HOR, INPUT);
  pinMode(P_A_JOY_VER, INPUT);

  Serial.println("Calibrating...");
  for(int i=0; i<100; i++) {
    calibVer += (analogRead(P_A_JOY_VER)-2048.0f)/2048.0f;
    calibHor += (analogRead(P_A_JOY_HOR)-2048.0f)/2048.0f;
  }
  calibVer /= 100.0f;
  calibHor /= 100.0f;

  if(mcp.begin_I2C(MCP_ADDR) == false) {
    while(true) Serial.printf("MCP init failed\n");
  }
  
  for (uint8_t i = 0; i < 8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    buttons[i] = false;
  }

  Serial.println("Scanning...");
  myDevice = nullptr;

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  pBLEScan->start(5, false);
}

void loopControl() {
  // Read joystick
  uint16_t verRaw = analogRead(P_A_JOY_VER);
  uint16_t horRaw = analogRead(P_A_JOY_HOR);

  float ver = -(float(verRaw - 2048)/2048.0f - calibVer);
  float hor = float(horRaw - 2048)/2048.0f - calibHor;

  // Read buttons
  bool tempButtons[8];
  for(int i=0; i<8; i++) {
    tempButtons[i] = !mcp.digitalRead(i);
  }
#if 0
  Serial.printf("Buttons: 1%c 2%c 3%c 4%c Joy%c Confirm%c Left%c Right%c\n",
    tempButtons[BUTTON_PIN_1] ? 'X' : '_',
    tempButtons[BUTTON_PIN_2] ? 'X' : '_',
    tempButtons[BUTTON_PIN_3] ? 'X' : '_',
    tempButtons[BUTTON_PIN_4] ? 'X' : '_',
    tempButtons[BUTTON_PIN_JOY] ? 'X' : '_',
    tempButtons[BUTTON_PIN_CONFIRM] ? 'X' : '_',
    tempButtons[BUTTON_PIN_LEFT] ? 'X' : '_',
    tempButtons[BUTTON_PIN_RIGHT] ? 'X' : '_');
  #endif
  
  // Drive motor control
  float mot0 = (ver-hor)*255.0, mot1 = (ver+hor)*255.0;

  uint8_t mot0DirByte = (mot0 < 0 ? 0x80 : 0x00) | 0;
  uint8_t mot1DirByte = (mot1 < 0 ? 0x80 : 0x00) | 1;
  uint8_t mot0PowerByte = fabs(mot0);
  uint8_t mot1PowerByte = fabs(mot1);

  // Dome motor control
  uint8_t mot2DirByte;
  uint8_t mot2PowerByte;
  if(tempButtons[BUTTON_PIN_3] == true && tempButtons[BUTTON_PIN_4] == false) {
    mot2DirByte = 0x80 | 3;
    mot2PowerByte = 0xff;
    Serial.printf("Mot 2 Dir: 0x%x Power: 0x%x\n", mot2DirByte, mot2PowerByte);
  } else if(tempButtons[BUTTON_PIN_3] == false && tempButtons[BUTTON_PIN_4] == true) {
    mot2DirByte = 0x00 | 3;
    mot2PowerByte = 0xff;
    Serial.printf("Mot 2 Dir: 0x%x Power: 0x%x\n", mot2DirByte, mot2PowerByte);
  } else {
    mot2DirByte = 0x00 | 3;
    mot2PowerByte = 0;
  }

  uint8_t motCmd[] = {0x29, 0x42, 0x05, 0x46, mot0DirByte, mot0PowerByte, 0x01, 0x2C, 0x00, 0x00,
                      0x29, 0x42, 0x05, 0x46, mot1DirByte, mot1PowerByte, 0x01, 0x2C, 0x00, 0x00,
                      0x29, 0x42, 0x05, 0x46, mot2DirByte, mot2PowerByte, 0x01, 0x2C, 0x00, 0x00};
                     
  pRemoteCharacteristic->writeValue(motCmd, 30);

  // Sound control
  if(tempButtons[BUTTON_PIN_1] == true && buttons[BUTTON_PIN_1] == false) {
    uint8_t sndCmd[] = {0x27, 0x42, 0x0F, 0x44, 0x44, 0x00, 0x1F, 0x00, 0x27, 0x42, 0x0F, 0x44, 0x44, 0x00, 0x18, 0x00};
    pRemoteCharacteristic->writeValue(sndCmd, 16);
  } else if(tempButtons[BUTTON_PIN_2] == true && buttons[BUTTON_PIN_3] == false) {
    uint8_t sndCmd[] = {0x27, 0x42, 0x0F, 0x44, 0x44, 0x00, 0x1F, 0x00, 0x27, 0x42, 0x0F, 0x44, 0x44, 0x00, 0x18, 0x01};
    pRemoteCharacteristic->writeValue(sndCmd, 16);
  }

  for(unsigned int i=0; i<8; i++) {
    buttons[i] = tempButtons[i];
  }
}

void loop() {
  if(doConnect) {
    if(connectToServer()) {
      Serial.printf("Connected!\n");
    } else {
      Serial.printf("Connect failed!\n");
    }
    doConnect = false;
  }

  if(connected) {
    loopControl();
    delay(40);
  }
}