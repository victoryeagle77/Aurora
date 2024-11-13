#ifndef IHM_CONFIG_H_
#define IHM_CONFIG_H_

#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#define PSW "12345678"
#define PORT 80

#define HTML "text/html"
#define JSON "application/json"
#define CSS "text/css"
#define JS "text/javascript"
#define FONT "application/font-woff"

#define JSON "application/json"
#define TXT "text/plain"

#define GET_SUCCESS 200
#define GET_FAILURE 400
#define ERR_LOG "Parametre invalide"

#define DYNDATA "/rd_dyn"
#define STADATA "/rd_sta"
#define ARMDATA "/rd_arm"
#define VATDATA "/rd_vat"
#define ROBCTRL "/rob_ctrl"
#define ARMCTRL "/arm_ctrl"
#define VATCTRL "/vat_ctrl"

#define CMD_JOINTS_RAD_CTRL 102
#define CMD_SERVO_RAD_FEEDBACK 105
#define CMD_SET_JOINT_PID 108
#define CMD_RESET_PID 109
#define CMD_SET_NEW_X 110
#define CMD_CONSTANT_CTRL 123
#define CMD_SET_MIDDLE 502
#define CMD_SET_SERVO_PID 503

void initIHM(void);

#endif