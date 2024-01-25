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
    String resp = CHAR_ENCODING; // "text/html; charset=utf-8"
    resp += "<HTML><BODY>";
    resp += "<center><h1><br>Connecting to your router...</h1>";

    int params = request->params();
    for (int i=0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost() && param_callback != NULL) {
        param_callback(p->name().c_str(), p->value().c_str());
        //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        resp += "<p>";
        resp += p->name().c_str();
        resp += ": ";
        resp += p->value().c_str();
        resp += "</p>";
      }
    }

    resp += "</center></BODY></HTML>";
    request->send(200, resp);

    unsigned long ms = millis();
    while(millis() - ms < 3000)
      yield();
  });

  server.begin();
  while(true) {
    callback();
    yield();
  }
}
