// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
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
//
// Based on:
//   ESPAsyncWebSrv Arduino library examples

#include "ap.h"

void AP::obtainConfig(void (*callback)(), const char * SSID_AP,
  set_param_t set_param_callback) {
  if (set_param_callback == NULL) {
    Serial.println("AP::obtainConfig() set_param_callback == NULL");
    return;
  }

  static set_param_t param_callback = set_param_callback; // hack
  AsyncWebServer server(80);  // Create AsyncWebServer object on port 80

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting up Access Point ");
  Serial.println(SSID_AP);
  // NULL sets an open Access Point
  WiFi.softAP(SSID_AP);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP); 

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", CHAR_ENCODING);
    // request->send(SPIFFS, "/index.html", "text/html");
    // request->send(SPIFFS, "/index.html", "utf-8");
  });
  
  server.serveStatic("/", SPIFFS, "/");
  
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    String resp = "<HTML><BODY>"
      "<center><h1><br>Connecting to your router...</h1>";

    int params = request->params();
    for (int i=0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost()) {
        //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        param_callback(p->name().c_str(), p->value().c_str());
        resp += "<p>";
        resp += p->name();
        resp += ": ";
        resp += p->value();
        resp += "</p>";
      }
    }

    resp += "</center></BODY></HTML>";
    Serial.println(resp);
    request->send(200, CHAR_ENCODING, resp);

    unsigned long ms = millis();
    while(millis() - ms < 3000)
      yield();
    param_callback(NULL, NULL);
  });

  server.begin();
  while(true) {
    callback();
    yield();
  }
}
