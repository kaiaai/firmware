// Copyright 2023-2025 KAIA.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <SPIFFS.h>
#include "robot_config.h"

extern CONFIG cfg;

class AP {
public:
  typedef String (*set_param_t)(const char*, const char*);
  void obtainConfig(const char * SSID_AP, set_param_t set_param_callback) {
    if (set_param_callback == NULL) {
      Serial.println("AP::obtainConfig() set_param_callback == NULL");
      return;
    }
  
    static set_param_t param_callback = set_param_callback; // hack
    AsyncWebServer server(80);  // Create AsyncWebServer object on port 80
  
    // Connect to Wi-Fi network with SSID and password
    Serial.print("Setting up WiFi ");
    Serial.print(SSID_AP);
    // NULL sets an open Access Point
    WiFi.softAP(SSID_AP);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("; browse to http://");
    Serial.println(IP);
  
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, cfg.INDEX_HTML_PATH, CHAR_ENCODING);
    });
    
    //server.serveStatic("/", SPIFFS, "/www/");
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      String resp = "<HTML><BODY>"
        "<center><h1><br>Connecting to WiFi...</h1><p><table>";
  
      int params = request->params();
      for (int i=0; i < params; i++) {
        AsyncWebParameter* p = request->getParam(i);
        if (p->isPost()) {
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
          String ret = param_callback(p->name().c_str(), p->value().c_str());
          resp += "<tr><td>";
          resp += p->name();
          resp += "</td><td>";
          resp += ret;
          resp += "</td></tr>";
        }
      }
  
      resp += "</table></p></center></BODY></HTML>";
      request->send(200, CHAR_ENCODING, resp);
  
      unsigned long ms = millis();
      while(millis() - ms < 500)
        yield();
      param_callback(NULL, NULL);
    });
  
    server.begin();
    while(true) {
      //callback();
      yield();
    }
  }    

protected:
  static constexpr char * CHAR_ENCODING = (char *)"text/html; charset=utf-8";
};
