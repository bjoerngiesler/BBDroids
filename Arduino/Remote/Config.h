#include <Arduino.h>

// network config
#define WifiSSID   "BB8Server"
#define WifiWPAKey "BB8Server"

#define RemoteToBB8UDPPort 2000
#define BB8ToRemoteUDPPort 2001 // currently not used

// owned by RemoteState
#define RemoteStatePinButtonTopLeft      A3
#define RemoteStatePinButtonTopRight     A2
#define RemoteStatePinButtonRightPinky   4
#define RemoteStatePinButtonRightIndex   5
#define RemoteStatePinJoystickHorizontal A6
#define RemoteStatePinJoystickVertical   A5
#define RemoteStatePinButtonJoystick     A4
#define RemoteStatePinButtonTopPCBLeft   8
#define RemoteStatePinButtonTopPCBRight  9
#define RemoteStatePinRotaryEncoder      A1
    
#define PinLEDNeopixel   7
#define PinDisplayRX    14
#define PinDisplayTX    13
#define PinDisplayReset 10

#define CalibBiasJoystickVertical 16
#define CalibBiasJoystickHorizontal -15
#define JoystickEpsilon 0.01f
