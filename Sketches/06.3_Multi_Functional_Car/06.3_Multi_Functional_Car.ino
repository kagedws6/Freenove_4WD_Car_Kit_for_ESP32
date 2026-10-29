/**********************************************************************
  Based on Freenove Camera Car, 2024/08/12.
  Modular rover extension: bounded input, motion leases and PWM slew limits.
**********************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <sys/socket.h>
#include "esp_camera.h"
#include "Freenove_4WD_Car_WiFi.h"
#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_4WD_Car_WS2812.h"
#include "Freenove_4WD_Car_For_ESP32.h"
#include "RoverNetwork.h"

std::atomic<bool> videoFlag{false};
WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);
WiFiClient commandClient;
rover::LineReceiver receiver;
bool hadCommandClient = false;
bool cameraReady = false;

bool reply(const char* text) {
  const size_t length = strlen(text);
  // Never wait for a controller that stops reading replies while motors run.
  if (send(commandClient.fd(), text, length, MSG_DONTWAIT) != static_cast<int>(length)) {
    Motor_Stop(); commandClient.stop(); return false;
  }
  return true;
}

void WiFi_Init() {
  // Keep network credentials local; the default remains the original AP mode.
  ssid_Router = "********";
  password_Router = "********";
  ssid_AP = "Sunshine";
  password_AP = "Sunshine";
}

void setup() {
  Serial.begin(115200);
  PCA9685_Setup(); // Disable motor outputs before network/camera initialization.
  Buzzer_Setup();
  WiFi_Init();
  RoverNetworkBegin();
  server_Cmd.begin();
  server_Camera.begin();
  cameraReady = cameraSetup();
  Emotion_Setup();
  WS2812_Setup();
  Light_Setup();
  Track_Setup();
  Setup_Battery_Monitor();
  if (cameraReady)
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
}

bool inRange(int v, int lo, int hi) { return v >= lo && v <= hi; }

bool handleCommand(const rover::Command& c) {
  const int* v = c.values;
  const size_t n = c.count;
  const uint32_t now = millis();
  if (!strcmp(c.name, "CMD_STOP") && n == 0) {
    Motor_Stop(); return true;
  }
  if (!strcmp(c.name, "CMD_HEARTBEAT") && n == 0) {
    // A late heartbeat cannot revive an expired motion command.
    return roverMotion.heartbeat(now);
  }
  if (!strcmp(c.name, "CMD_DRIVE") && n == 3) {
    if (!inRange(v[0], -4095, 4095) || !inRange(v[1], -4095, 4095) ||
        !inRange(v[2], 1, rover::kMaxDriveMs)) return false;
    Car_SetMode(0);
    roverMotion.arm(now, v[2], true);
    Motor_Move(v[0], v[0], v[1], v[1]);
    if (v[0] == 0 && v[1] == 0) Motor_Stop();
    return true;
  }
  if (!strcmp(c.name, CMD_MOTOR) && n == 4) {
    for (int i = 0; i < 4; ++i) if (!inRange(v[i], -4095, 4095)) return false;
    Car_SetMode(0);
    roverMotion.arm(now);
    Motor_Move(v[0], v[0], v[2], v[2]); // Preserve the stock side-pair mapping.
    if (v[0] == 0 && v[2] == 0) Motor_Stop();
    return true;
  }
  if ((!strcmp(c.name, CMD_CAR_MODE) || !strcmp(c.name, CMD_LIGHT) ||
       !strcmp(c.name, CMD_TRACK)) && n == 1) {
    int mode = v[0];
    if (!strcmp(c.name, CMD_CAR_MODE)) {
      if (!inRange(mode, 0, 2)) return false;
    } else {
      if (!inRange(mode, 0, 1)) return false;
      if (mode) mode = !strcmp(c.name, CMD_LIGHT) ? 1 : 2;
    }
    Motor_Stop();
    if (mode) { roverMotion.arm(now); Car_SetMode(mode); }
    return true;
  }
  if (!strcmp(c.name, CMD_SERVO) && n == 2) {
    if (!inRange(v[0], 0, 1) || !inRange(v[1], v[0] ? 80 : 0, 180)) return false;
    if (v[0] == 0) Servo_1_Angle(v[1]); else Servo_2_Angle(v[1]);
    return true;
  }
  if (!strcmp(c.name, CMD_CAMERA) && n == 2) {
    if (!inRange(v[0], 0, 180) || !inRange(v[1], 80, 180)) return false;
    Servo_1_Angle(v[0]); Servo_2_Angle(v[1]); return true;
  }
  if (!strcmp(c.name, CMD_LED_MOD) && n == 1 && inRange(v[0], 0, 5)) {
    WS2812_SetMode(v[0]); return true;
  }
  if (!strcmp(c.name, CMD_MATRIX_MOD) && n == 1 && inRange(v[0], 0, 7)) {
    Emotion_SetMode(v[0]); return true;
  }
  if (!strcmp(c.name, CMD_LED) && n == 4 && inRange(v[0], 0, 4095) &&
      inRange(v[1], 0, 255) && inRange(v[2], 0, 255) && inRange(v[3], 0, 255)) {
    WS2812_Set_Color_1(v[0], v[1], v[2], v[3]); return true;
  }
  if (!strcmp(c.name, CMD_VIDEO) && n == 1 && inRange(v[0], 0, 1)) {
    if (v[0] && !cameraReady) return false;
    videoFlag = v[0] != 0; return true;
  }
  if (!strcmp(c.name, CMD_BUZZER) && n == 2 && inRange(v[0], 0, 1) && inRange(v[1], 0, 10000)) {
    Buzzer_Variable(v[0], v[1]); return true;
  }
  if (!strcmp(c.name, CMD_POWER) && n == 0) {
    char response[64];
    snprintf(response, sizeof(response), "CMD_POWER#%.2f\n", Get_Battery_Voltage());
    return reply(response);
  }
  if (!strcmp(c.name, "CMD_STATUS") && n == 0) {
    char response[128];
    snprintf(response, sizeof(response), "CMD_STATUS#1#%d#%d#%d#%d#%d#%.2f#%d\n",
      roverMotion.armed, carFlag, rover::kSpeedLimit,
      roverMotion.output[0], roverMotion.output[2], Get_Battery_Voltage(), cameraReady);
    return reply(response);
  }
  return false;
}

void badCommand() {
  Motor_Stop();
  reply("ERR#BAD_COMMAND\n");
}

void loop() {
  Motor_Service();
  if (RoverNetworkService()) { delay(1); return; }
  if (hadCommandClient && !commandClient.connected()) {
    Motor_Stop();
    videoFlag = false;
    commandClient.stop(); receiver.reset(); hadCommandClient = false;
  }
  if (!hadCommandClient) {
    commandClient = server_Cmd.accept();
    if (commandClient) {
      receiver.reset(); hadCommandClient = true; Motor_Stop();
    }
  }
  if (hadCommandClient) {
    if (receiver.expire(millis())) badCommand();
    // A sender cannot monopolize the motor service by streaming bytes forever.
    for (int budget = 0; budget < 96 && commandClient.available(); ++budget) {
      auto result = receiver.push(static_cast<char>(commandClient.read()), millis());
      if (result == rover::LineReceiver::Invalid) badCommand();
      if (result == rover::LineReceiver::Complete) {
        rover::Command c;
        if (!rover::parseCommand(receiver.line, c) || !handleCommand(c)) badCommand();
      }
      Motor_Service();
    }
  }
  if (roverMotion.armed) Car_Select(carFlag);
  Motor_Service();
  Emotion_Show(emotion_task_mode);
  WS2812_Show(ws2812_task_mode);
  delay(1);
}

void loopTask_Camera(void *pvParameters) {
  while (1) {
    delay(5);
    WiFiClient client = server_Camera.accept();//listen for incoming clients
    if (client) {//if you get a client
      Serial.println("Camera_Server connected to a client.");
      if (client.connected()) {
        while (client.connected()) {//loop while the client's connected
          delay(1);
          if (videoFlag) { 
            camera_fb_t* fb = esp_camera_fb_get();
            size_t jpg_buf_len = 0;
            uint8_t *jpg_buf = NULL;
            if (fb != NULL) {
              if (fb->format != PIXFORMAT_JPEG){
                  bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
                  esp_camera_fb_return(fb);
                  fb = NULL;
                  if (!jpeg_converted) {
                      Serial.println("JPEG compression failed");
                      continue; 
                  }
              }
              else {
                  jpg_buf_len = fb->len;
                  jpg_buf = fb->buf;
              }
              uint8_t slen[4];
              slen[0] = jpg_buf_len >> 0;
              slen[1] = jpg_buf_len >> 8;
              slen[2] = jpg_buf_len >> 16;
              slen[3] = jpg_buf_len >> 24;
              client.write(slen, 4);
              client.write(jpg_buf, jpg_buf_len);
            } else {
              Serial.println("Failed to get frame");
            }
            if (fb){
                esp_camera_fb_return(fb);
                fb = NULL;
                jpg_buf = NULL;
            }
            else if (jpg_buf){
                free(jpg_buf);
                jpg_buf = NULL;
            }
          } else {
              delay(100); 
          }
        }
        //close the connection:
        client.stop();
        Serial.println("Camera Client Disconnected.");
        // Camera disconnects do not restart or drive the rover.
      }
    }
  }
}
