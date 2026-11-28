// Template for Secrets.h -- the build copies this to include/Secrets.h if that doesn't exist yet.
// Put your real values into Secrets.h, never into this file: Secrets.h is in .gitignore, this file is committed.
#if !defined(SECRETS_H)
#define SECRETS_H

// WiFi for the Monaco/UDP protocol. WiFi stays off unless a UDP protocol is used, because a WiFi
// connection fixes the radio channel, which ESP-NOW droids must then share.
//
// BBR_WIFI_AP_MODE true:  the remote opens its own access point with this SSID and password.
//                         Join it with the PC; the remote is at 192.168.4.1.
// BBR_WIFI_AP_MODE false: the remote joins the existing network with this SSID and password.
#define BBR_WIFI_AP_MODE   true
#define BBR_WIFI_SSID      "BBRemote-UDP"
#define BBR_WIFI_PASSWORD  "bbremotes"   // at least 8 characters

#endif // SECRETS_H
