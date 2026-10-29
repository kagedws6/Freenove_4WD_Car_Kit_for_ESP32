#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "Freenove_4WD_Car_WiFi.h"
#include "Freenove_4WD_Car_For_ESP32.h"
#include "RoverNetwork.h"

namespace {
WebServer portal(80);
Preferences preferences;
char savedSsid[33] = {}, savedPassword[65] = {};
bool provisionMode = false, restarting = false;
uint32_t restartRequested = 0;
const char page[] = R"HTML(<!doctype html><html lang="en"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Rover Wi-Fi</title><body><h1>Connect your rover</h1><form action="/configure" method="post"><p><label>Wi-Fi name <input name="ssid" maxlength="32" required></label></p><p><label>Wi-Fi password <input name="password" type="password" maxlength="63" autocomplete="current-password"></label></p><button>Save and reconnect</button></form></body></html>)HTML";
}

void RoverNetworkBegin() {
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
      Serial.printf("WiFi station disconnect reason: %u\n", info.wifi_sta_disconnected.reason);
  });
  const bool prefsReady = preferences.begin("rover-net", false);
  if (prefsReady) {
    preferences.getString("ssid", "").toCharArray(savedSsid, sizeof(savedSsid));
    preferences.getString("password", "").toCharArray(savedPassword, sizeof(savedPassword));
  }
  if (savedSsid[0]) {
    Serial.printf("Saved WiFi field lengths: name=%u key=%u\n",
                  static_cast<unsigned>(strlen(savedSsid)),
                  static_cast<unsigned>(strlen(savedPassword)));
    ssid_Router = savedSsid; password_Router = savedPassword;
    WiFi_Setup(0);
  } else WiFi_Setup(1);
  provisionMode = WiFi.getMode() == WIFI_AP;
  portal.on("/", HTTP_GET, [] {
    if (provisionMode) portal.send(200, "text/html", page);
    else portal.send(200, "text/plain", "Rover online. Command port: 4000. Camera port: 7000.");
  });
  portal.on("/configure", HTTP_POST, [prefsReady] {
    if (!provisionMode || !prefsReady || roverMotion.armed) {
      portal.send(403, "text/plain", "Configuration unavailable."); return;
    }
    String ssid = portal.arg("ssid"), password = portal.arg("password");
    if (ssid.length() < 1 || ssid.length() > 32 ||
        !(password.length() == 0 || (password.length() >= 8 && password.length() <= 63))) {
      portal.send(400, "text/plain", "Check the Wi-Fi name and password length."); return;
    }
    Motor_Stop();
    if (preferences.putString("ssid", ssid) != ssid.length() ||
        preferences.putString("password", password) != password.length()) {
      portal.send(500, "text/plain", "Settings could not be saved."); return;
    }
    portal.send(200, "text/plain", "Saved. The rover is restarting. Reconnect your phone to your home Wi-Fi.");
    restarting = true; restartRequested = millis();
  });
  portal.begin();
  Serial.printf("Rover address: %s\n", (provisionMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
}

bool RoverNetworkService() {
  if (restarting) {
    Motor_Stop();
    if (static_cast<uint32_t>(millis() - restartRequested) >= 750) ESP.restart();
    return true;
  }
  // WebServer can wait for HTTP input. Never service it with an active motor lease.
  if (!roverMotion.armed) {
    portal.handleClient();
  }
  return restarting;
}
