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

#pragma once
#include <SPIFFS.h>

class PARAM_FILE {
protected:
  char * const* param_name;
  String * param_value;
  uint16_t len;

  static constexpr char * FILE_PATH = (char *)"/config.txt";
  static constexpr char * FILE_PATH_OLD = (char *)"/config.old";

public:
  PARAM_FILE(char* const* param_names, String * param_values, uint16_t len) {
    param_name = param_names;
    param_value = param_values;
    this->len = len;
  }

  bool load(bool old_config = false) {
    const char * CONFIG_PATH = old_config ? FILE_PATH_OLD : FILE_PATH;
    Serial.print("Reading file ");
    Serial.print(CONFIG_PATH);
    
    File file = SPIFFS.open(CONFIG_PATH);
    if (!file || file.isDirectory()) {
      Serial.println(" - file not found");
      return false;
    } else
      Serial.println();
    
    String name_str;
    uint8_t i = 0;
    while (file.available()) {
      switch (i & 0x1) {
        case 0:
          name_str = file.readStringUntil('\n');
          break;
        case 1:
          String value_str = file.readStringUntil('\n');
          setByName(name_str.c_str(), value_str.c_str());
          break;
      }
      i++;
    }
    return true;
  }

  bool save(bool old_config=false) {
    const char * CONFIG_PATH = old_config ? FILE_PATH_OLD : FILE_PATH;
    Serial.print("Writing file ");
    Serial.print(CONFIG_PATH);
  
    File file = SPIFFS.open(CONFIG_PATH, FILE_WRITE);
    if (!file) {
      Serial.print(" - file open failed");
      return false;
    }
    Serial.println();

    bool success = true;
    for (uint16_t i = 0; i < len; i++) {
      //Serial.print(param_name[i]);
      //Serial.print("=");
      //Serial.println(param_value[i]);
      success = success && file.print(param_name[i]);
      success = success && file.print('\n');
      success = success && file.print(param_value[i]);
      success = success && file.print('\n');
      if (!success) {
        Serial.println("Write failed ");
        break;
      }
    }
    
    file.close();
    return success;
  }
  
  bool init() {
    if (!SPIFFS.begin(true)) {
      Serial.println("Error mounting SPIFFS");
      return false;
    }
    Serial.println("SPIFFS mounted successfully");
  
    //if (!SPIFFS.exists("/www/index.html")) {
    if (!SPIFFS.exists("/index.html")) {
      Serial.println("Sketch data not found. Have you uploaded the sketch data?");
      return false;
    }
    return true;
  }

  bool setByName(const char * pname, const char * pvalue) {
    if (pname == NULL || pvalue == NULL) {
      Serial.print("NULL pointer in setByName()");
      return false;
    }

    int16_t idx = nameToIndex(pname);
    if (idx < 0) {
      //Serial.print("Parameter ");
      //Serial.print(pname);
      //Serial.println(" not found in setByName()");
      return false;
    }
    return set(idx, pvalue);
  }
  
  bool set(const uint16_t param_idx, const char * pvalue) {
    if (param_idx >= len || pvalue == NULL) {
      Serial.print("NULL pointer or invalid index in set()");
      return false;
    }
    //Serial.print(param_idx);
    //Serial.print(" ");
    //Serial.print(param_name[param_idx]);
    //Serial.print("=");
    //Serial.println(pvalue);

    param_value[param_idx] = String(pvalue);
      
    return true;
  }

  const char * get(const uint16_t idx) {
    if (idx >= len)
      return "";
  
    return param_value[idx].c_str();
  }

  float getAsFloat(const uint16_t idx) {
    return String(get(idx)).toFloat();
  }

  float getAsInt(const uint16_t idx) {
    return String(get(idx)).toInt();
  }

  const char * getName(const uint16_t idx) {
    if (idx >= len)
      return "";
  
    return param_name[idx];
  }

  const char * getByName(const char * pname) {
    int16_t idx = nameToIndex(pname);
    if (idx < 0 || idx >= len)
      return "";
  
    return param_value[idx].c_str();
  }
  
  int16_t nameToIndex(const char * pname) {
    if (pname == NULL)
      return -1;
  
    // Hack
    for (int16_t i = 0; i < len; i++) {
      if (strcmp(pname, param_name[i]) == 0)
        return i;
    }
  
    return -1;
  }
  
  bool purge() {
    SPIFFS.remove(FILE_PATH_OLD);
    return SPIFFS.rename(FILE_PATH, FILE_PATH_OLD);
  }
};
