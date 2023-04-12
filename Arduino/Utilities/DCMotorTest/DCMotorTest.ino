static const uint8_t P_YAW_EN          = 15;
static const uint8_t P_YAW_A           = 16;
static const uint8_t P_YAW_B           = 17;
static const uint8_t P_YAW_PWM         = 18;
static const uint8_t P_DRIVE_PWM       = 19;
static const uint8_t P_DRIVE_EN        = 20;
static const uint8_t P_DYNAMIXEL_RTS   = 21;
static const uint8_t P_SERIALTX_TX     = 0;
static const uint8_t P_SERIALTX_RX     = 1;
static const uint8_t P_DRIVE_A         = 2;
static const uint8_t P_DRIVE_B         = 3;



void setup() {
  Serial.begin(2000000);
  
  while(!Serial);

  pinMode(P_DRIVE_A, OUTPUT);
  digitalWrite(P_DRIVE_A, LOW);
  pinMode(P_DRIVE_B, OUTPUT);
  digitalWrite(P_DRIVE_B, LOW);
  pinMode(P_DRIVE_EN, OUTPUT);
  digitalWrite(P_DRIVE_EN, LOW);
  pinMode(P_DRIVE_PWM, OUTPUT);
  analogWrite(P_DRIVE_PWM, 0);
}

void loop() {
  Serial.println("Driving forward...");
  digitalWrite(P_DRIVE_A, HIGH);
  digitalWrite(P_DRIVE_B, LOW);
  digitalWrite(P_DRIVE_EN, HIGH);
  for(int i=0; i<255; i++) {
    analogWrite(P_DRIVE_PWM, i);
    delay(50);
  }
  for(int i=255; i>0; i--) {
    analogWrite(P_DRIVE_PWM, i);
    delay(50);
  }

  digitalWrite(P_DRIVE_A, HIGH);
  digitalWrite(P_DRIVE_B, LOW);
  digitalWrite(P_DRIVE_EN, LOW);
  analogWrite(P_DRIVE_PWM, 0);

  delay(1000);

  Serial.println("Driving backward...");
  digitalWrite(P_DRIVE_A, LOW);
  digitalWrite(P_DRIVE_B, HIGH);
  digitalWrite(P_DRIVE_EN, HIGH);
  for(int i=0; i<255; i++) {
    analogWrite(P_DRIVE_PWM, i);
    delay(50);
  }
  for(int i=255; i>0; i--) {
    analogWrite(P_DRIVE_PWM, i);
    delay(50);
  }

  digitalWrite(P_DRIVE_A, HIGH);
  digitalWrite(P_DRIVE_B, LOW);
  digitalWrite(P_DRIVE_EN, LOW);
  analogWrite(P_DRIVE_PWM, 0);

  delay(1000);
}
