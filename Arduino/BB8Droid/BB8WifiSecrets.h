#if !defined(BB8WIFISECRETS_H)
#define BB8WIFISECRETS_H

#if 1
static const bool  WIFI_AP_MODE  = false;
static const String WIFI_SSID    = "Hogwarts";
static const String WIFI_WPA_KEY = "1s(h1pu+Bj0rn";
#else
static const bool   WIFI_AP_MODE = true;
static const String WIFI_SSID    = "BB8Wifi";
static const String WIFI_WPA_KEY = "BB8WifiKey";
#endif

#endif