#include <Arduino.h>

// network config
#define WIFI_SSID   "BB8Server"
#define WIFI_WPA_KEY "BB8Server"
#define REMOTE_UDP_PORT 2000

// motor ports
#if defined(FINAL_BOARD_CONFIG)
#define STEPPER_ENABLE_PIN         5
#define DRIVE_STEPPER_STEP_PIN     0
#define DRIVE_STEPPER_DIR_PIN      1
#define SPOT_TURN_STEPPER_STEP_PIN A3
#define SPOT_TURN_STEPPER_DIR_PIN  A4
#else // !defined(FINAL_BOARD_CONFIG)
#define STEPPER_ENABLE_PIN         5
#define DRIVE_STEPPER_STEP_PIN     2
#define DRIVE_STEPPER_DIR_PIN      3
#define SPOT_TURN_STEPPER_STEP_PIN A3
#define SPOT_TURN_STEPPER_DIR_PIN  A4
#endif // defined(FINAL_BOARD_CONFIG)

#define DRIVE_STEPPER_MAX_SPEED    2000.0
#define DRIVE_STEPPER_ACCELERATION 8000.0
