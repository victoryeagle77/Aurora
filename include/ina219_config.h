#ifndef INA219_CONFIG_H_
#define INA219_CONFIG_H_

#include <Arduino.h>
#include <Wire.h>
#include <INA219_WE.h>

// Module d'evaluation de batterie INA219
#define SDA_INA219 32
#define SCL_INA219 33
#define ADDR_INA219 0x42 // Adresse I2C du module INA219

#define SHUNT 0.01 // Composant de shuntage sur la tension de 0.01 Ohm

// Le connecteur d'alimentation DC de la carte est fait pour 3 batteries 18650 de 3.7V chacune
#define MAX_VOLT 12.6 // 4.2V lors d'une charge totale donc 3 * 4.2 = 12.6
#define MIN_VOLT 9 // Tension minimale autorisee par la carte

void initINA219(void);
float readVoltage(void);
float readCurrent(void);
float readPower(void);
float batteryLevel(void);

#endif