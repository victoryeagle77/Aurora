#include "../include/ina219_config.h"

INA219_WE ina219 = INA219_WE(ADDR_INA219);

float loadVoltage_V = 0.0;

/**
* @brief initINA219 : Initialisation du module INA219
* Configuration du registre d'adresse
* Configuration des plages de tension de shuntage et de bus au maximum
* Configuration de l'amplificateur de gain programmable au maximum sur résolution ADC 12 bits
*/
extern void initINA219(void){
  Wire.begin(SDA_INA219, SCL_INA219);

  if(ina219.init()){
    ina219.setADCMode(BIT_MODE_9);
    ina219.setPGain(PG_320);
    ina219.setBusRange(BRNG_16);
    ina219.setShuntSizeInOhms(SHUNT);
  }
}

/**
* @brief readVoltage : Lecture de la tension
* @return current : Le courant en V
*/
extern float readVoltage(void) {
  float shuntVoltage_mV = 0.0;
  float busVoltage_V = 0.0;
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  return loadVoltage_V;
}

/**
* @brief readCurrent : Lecture du courrant
* @return current : Le courant en mA
*/
extern float readCurrent(void) {
  float current_mA = 0.0;
  current_mA = ina219.getCurrent_mA();
  return current_mA;
}

/**
* @brief readPower : Lecture de la puissance consommée
* @return power : La puissance en mW
*/
extern float readPower(void) {
  float power_mW = 0.0; 
  power_mW = ina219.getBusPower();
  return power_mW;
}

/**
* @brief batteryLevel : Calculer le niveau de batterie
* @param voltage : Niveau de tension courrant par rapport au maximum et minimum autorise
* @return level : Niveau de batterie en pourcentage
*/
extern float batteryLevel(void) {
  float level = (loadVoltage_V * 100.0) / (MAX_VOLT + MIN_VOLT);
  level = constrain(level, 0, 100);
  return level;
}