/*  Copyright (C) 2016 Buxtronix and Alexander Pruss

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <EEPROM.h>

#ifndef SETTINGS_H
#define SETTINGS_H

#define MAGIC_LENGTH 4
#define SSID_LENGTH 32
#define PSK_LENGTH 64
#define NAME_LENGTH 32
#define SERVER_LENGTH 256
#define ID_LENGTH 16
#define KEY_LENGTH 16
#define FINGERPRINT_LENGTH 128

#define MAGIC "AQE\0"

char my_default_ssid[SSID_LENGTH] = "";
char my_default_psk[PSK_LENGTH] = "";

#define CLOCK_NAME "ESP-AIRQUAL"

class Settings {
  public:
  struct {
    char    magic[MAGIC_LENGTH];               // magic = "AQE"
    char    ssid[SSID_LENGTH];                 // SSID
    char    psk[PSK_LENGTH];                   // PASSWORD
    char    name[NAME_LENGTH];                 // NOMBRE
    int     poolinterval;                      // Intervalo de envío a Jeedom y Wunderground (sec)
    char    stationID[ID_LENGTH];              // Wunderground Station ID
    char    stationKey[KEY_LENGTH];            // Wunderground Station Key
    char    wunderfinger[FINGERPRINT_LENGTH];  // Wunderground certificate Fingerprint
    char    mqttbroker[SERVER_LENGTH];         // Address of MQTT broker server
    char    mqtttopic[NAME_LENGTH];            // MQTT topic for Jeedom
    char    mqttuser[NAME_LENGTH];             // MQTT account user
    char    mqttpassword[NAME_LENGTH];         // MQTT account password
    int     mqttport;                          // port of MQTT broker
    unsigned long ThingSpeakChannel;           // ThingSpeak channel number
    char    ThingSpeakKey[NAME_LENGTH];        // ThingSpeak API Write Key
    float   tempadjust;                        // Calibracion de temperatura
    int     rainthreshold;                     // Threshold of rain sensor
    float   uvadjust;                          // ajuste UV
    float   altitude;                          // altitude over sea level 
    int     windpoolinterval;                      // interval to send wind measures to MQTT
    bool    inecomode;                         // indica si estaba en ECO cuando se apago
    bool    groveactive;                       // Is Grove sensor installed/active ?
    } data;
    
    Settings() {};
 
    void Load() {           // Carga toda la EEPROM en el struct
      EEPROM.begin(sizeof(data));
      EEPROM.get(0,data);  
      EEPROM.end();
     // Verify magic; // para saber si es la primera vez y carga los valores por defecto
     if (String(data.magic)!=MAGIC){
      data.magic[0]=0;
      data.ssid[0]=0;
      data.psk[0]=0;
      data.name[0]=0;
      data.poolinterval = 300;
      data.stationID[0]=0;
      data.stationKey[0]=0;
      data.wunderfinger[0]=0;
      data.mqttbroker[0]=0;
      strcpy(data.mqtttopic, "jeedom");
      data.mqttuser[0]=0;
      data.mqttpassword[0]=0;
      data.mqttport=1883;
      data.ThingSpeakChannel=0;
      data.ThingSpeakKey[0]=0;
      data.tempadjust=0.0;
      data.rainthreshold=700;
      data.uvadjust=1.0;
      data.altitude=720.0;
      data.windpoolinterval = 1;
      data.inecomode=false;
      data.groveactive=false;
      Save();
     }
    };
      
    void Save() {
      EEPROM.begin(sizeof(data));
      EEPROM.put(0,data);
      EEPROM.commit();
      EEPROM.end();
    };
};
#endif
