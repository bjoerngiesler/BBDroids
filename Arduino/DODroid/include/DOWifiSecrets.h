#if !defined(DOWIFISECRETS_H)
#define DOWIFISECRETS_H

#if 1
static const bool   WIFI_AP_MODE = true;
static const String WIFI_SSID    = "DOWifi-$MAC";
static const String WIFI_WPA_KEY = "DOWifiKey";
#else
static const bool   WIFI_AP_MODE = false;
static const String WIFI_SSID    = "Hogwarts";
static const String WIFI_WPA_KEY = "1s(h1pu+Bj0rn";
#endif

#endif