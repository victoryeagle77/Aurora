#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;

#include "../include/ihm_config.h"
#include "../include/RoArmM2_config.h"
#include "../include/motors_config.h"
#include "../include/ina219_config.h"
#include "../include/sdcard_config.h"
#include "../include/pump_config.h"

#include "esp32/clk.h"

#define SSID "aurora2"

// Server WEB Asynchrone
AsyncWebServer server(PORT);

// Temperature interne de l'ESP32
extern "C" { uint8_t temprature_sens_read(); }

/**
* @brief rootfs
* Permet de monter un systeme de fichier leger dans la memoire Flash
*/
void rootfs(void){
  if(!(SPIFFS.begin()))
    return;

  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  while(file){
    file.close();
    file = root.openNextFile();
  }
}

/**
* @brief webserver
* Mise en place des fichiers web pour l'interface utilisateur
*/
void webserver(void){
  server.begin();

  // Fichier HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", HTML);
  });
  // Fichier CSS
  server.on("/assets/main.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/assets/main.css", CSS);
  });
  // Fichier JavaScript
  server.on("/assets/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/assets/script.js", JS);
  });
  // Fichier de police d'ecriture
  server.on("/assets/sansation_light_V3.woff", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/assets/sansation_light_V3.woff", FONT);
  });
}

/**
 * @brief dynInfo
 * Permet d'envoyer sous forme d'une chaine JSON des informations
 * Concernant la carte et l'ESP32, ces informations dynamiques sont a rafraichir
 * @return Chaine JSON avec les parametres dynamiques
*/
String dynInfo(void){
    // Temperature CPU
    float temp_cpu = (temprature_sens_read() - 32) / 1.8;

    // Memoire RAM
    uint32_t total_ram = ESP.getHeapSize();
    uint32_t free_ram = ESP.getFreeHeap();
    uint32_t used_ram = total_ram - free_ram;
    float ram_usage = (used_ram * 100.0) / total_ram;
    // Niveau de courant
    float bat_current = readCurrent();

    // Niveau de tension
    float bat_voltage = readVoltage();

    // Niveau de puissance
    float bat_power = readPower();

    // Niveau de batterie
    float bat_level = batteryLevel();

    // Informations de la carte SD
    String* dataSD = infoSD();
  
    String str = "{";
    str += "\"Température CPU\":\"" + String(temp_cpu, 2) + " °C" + "\",";
    str += "\"Usage RAM\":\"" + String(ram_usage, 2) + " %" + "\",";
    str += "\"Carte SD\":\"" + dataSD[0] + " " + dataSD[1] + " %" + "\",";
    str += "\"Courrant consommé\":\"" + String(bat_current, 2) + " mA" + "\",";
    str += "\"Tension consommée\":\"" + String(bat_voltage, 2) + " V" + "\",";
    str += "\"Puissance consommée\":\"" + String(bat_power, 2) + " mW" + "\",";
    str += "\"Niveau batterie\":\"" + String(bat_level, 2) + " %" + "\"";
    str += "}";

    return str;
}

/**
 * @brief staInfo
 * Permet d'envoyer sous forme d'une chaine JSON des informations
 * Concernant la carte et l'ESP32, ces informations statiques a envoyer une unique fois
 * @return Chaine JSON avec les parametres statiques
*/
String staInfo(void){
    // Memoire Flash
    size_t total_flash = SPIFFS.totalBytes();
    size_t used_flash = SPIFFS.usedBytes();
    float flash_usage = (used_flash * 100.0) / total_flash;

    // Frequence de fonctionnement
    uint32_t cpu_freq = esp_clk_cpu_freq();
    float cpu_freq_mhz = cpu_freq / 1000000;

    // Adresse MAC
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    String str_mac_addr = String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + 
                          String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + 
                          String(mac[4], HEX) + ":" + String(mac[5], HEX);

    // Adresse IPv4
    IPAddress ipv4_addr = WiFi.softAPIP();
    String str_ipv4_addr = ipv4_addr.toString();

    String str = "{";
    str += "\"Usage Flash\":\"" + String(flash_usage, 2) + " %" + "\",";
    str += "\"Fréquence CPU\":\"" + String(cpu_freq_mhz, 2) + " MHz" + "\",";
    str += "\"Adresse MAC\":\"" + str_mac_addr + "\",";
    str += "\"Adresse IPv4\":\"" + str_ipv4_addr + "\",";
    str += "\"SSID Wifi\":\"" + String(SSID) + "\"";
    str += "}";

    return str;
}

void jsonCmdReceiveHandler(void){
    int cmdType = jsonCmdReceive["T"].as<int>();
    switch(cmdType){
        // Controle tous les elements du bras par rapport a l'angle de 180° (3.1415926 en radian)
        case CMD_JOINTS_RAD_CTRL: 
            RoArmM2_allJointAbsCtrl(jsonCmdReceive["base"], jsonCmdReceive["shoulder"], jsonCmdReceive["elbow"],
                                    jsonCmdReceive["hand"], jsonCmdReceive["spd"], jsonCmdReceive["acc"]);
        break;

        case CMD_SERVO_RAD_FEEDBACK:
            RoArmM2_getPosByServoFeedback();
            RoArmM2_infoFeedback();
        break;
        case CMD_SET_JOINT_PID:
            RoArmM2_setJointPID(jsonCmdReceive["joint"], jsonCmdReceive["p"], jsonCmdReceive["i"]);
        break;

        case CMD_RESET_PID:	
            RoArmM2_resetPID();
        break;

        case CMD_SET_NEW_X:
            setNewAxisX(jsonCmdReceive["xAxisAngle"]);
        break;

        case CMD_CONSTANT_CTRL:
            constantCtrl(jsonCmdReceive["m"], jsonCmdReceive["axis"],
                            jsonCmdReceive["cmd"], jsonCmdReceive["spd"]);
        break;

    	  case CMD_SET_MIDDLE: 
            setMiddlePos(jsonCmdReceive["id"]);
        break;

        case CMD_SET_SERVO_PID: 
            setServosPID(jsonCmdReceive["id"], jsonCmdReceive["p"]);
        break;

	}
}

/**
* @brief sending
* Communication par envoie de requete JSON en HTTP sur l'interface WEB
*/
void sending(void){
  // Donnees dynamiques avec envoie et rafraichissement
  server.on(DYNDATA, HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(GET_SUCCESS, JSON, dynInfo());
  });

  // Donnees statiques en un envoie
  server.on(STADATA, HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(GET_SUCCESS, JSON, staInfo());
  });

  // Niveau de la cuve
  server.on(VATDATA, HTTP_GET, [](AsyncWebServerRequest *request){
    float vat_level = infoPump();
    String str_vat_level = String(vat_level, 2) + " %";
    request->send(GET_SUCCESS, TXT, str_vat_level);
  });

  server.on(ARMCTRL, HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      deserializeJson(jsonCmdReceive, action);
      jsonCmdReceiveHandler();
      request->send(GET_SUCCESS, JSON, RoArmM2_infoFeedback());
    } else {
      request->send(GET_FAILURE, TXT, ERR_LOG);
    }
  });
}

/**
* @brief recieving
* Communication par reception de requete HTTP de l'interface WEB
*/
void recieving(void){
  // Pompe pour epandage
  server.on(VATCTRL, HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      if (action == "on") {
        activePump(0);
        request->send(GET_SUCCESS);
      } else if (action == "off") {
        activePump(1);
        request->send(GET_SUCCESS);
      } else if (action == "tare") {
        activePump(2);
        request->send(GET_SUCCESS);
      } else {
        request->send(GET_FAILURE, TXT, ERR_LOG);
      }

    } else {
      request->send(GET_FAILURE, TXT, ERR_LOG);
    }
  });

  // Directions du vehicule
  server.on(ROBCTRL, HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      // Tourner a droite
      if (action == "right") {
        directionAB(0);
        request->send(GET_SUCCESS);
      // Tourner a gauche
      } else if (action == "left") {
        directionAB(1);
        request->send(GET_SUCCESS);
      // Aller en avant
      } else if (action == "forward") {
        directionAB(2);
        request->send(GET_SUCCESS);
      // Aller en arriere
      } else if (action == "backward") {
        directionAB(3);
        request->send(GET_SUCCESS);
      // Aller en avant a droite
      } else if (action == "right_forward") {
        directionAB(4);
        request->send(GET_SUCCESS);
      // Aller en avant a gauche
      } else if (action == "left_forward") {
        directionAB(5);
        request->send(GET_SUCCESS);
      // Aller en arriere a droite
      } else if (action == "right_backward") {
        directionAB(6);
        request->send(GET_SUCCESS);
      // Aller en arriere a gauche
      } else if (action == "left_backward") {
        directionAB(7);
        request->send(GET_SUCCESS);
      // Arreter les moteurs
      } else if (action == "stop") {
        directionAB(8);
        request->send(GET_SUCCESS);
      } else {
        request->send(GET_FAILURE, TXT, ERR_LOG);
      }

    } else {
      request->send(GET_FAILURE, TXT, ERR_LOG);
    }
  });
}

/**
 * @brief accesPoint
 * Monter le point d'acces Wifi
*/
void accesPoint(void){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PSW);
}

/**
* @brief initIHM
* Mise en place d'un point d'acces Wifi pour atterir sur le serveur WEB
*/
void initIHM(void){
  accesPoint();
  rootfs();
  webserver();
  sending();
  recieving();
}
