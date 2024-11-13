#include "../include/RoArmM2_config.h"
#include "../include/motors_config.h"
#include "../include/screen_config.h"
#include "../include/ina219_config.h"
#include "../include/sdcard_config.h"
#include "../include/pump_config.h"
#include "../include/ihm_config.h"
#include "../include/axl_config.h"

void setup(void){
  RoArmM2_servoInit();
  RoArmM2_initCheck(false);
  RoArmM2_resetPID();

  initMotors(); // Broches des moteurs JETANK 20770
  initINA219(); // Broches pour le module INA219
  initIMU(); // Broches pour les modules AK09918 et QMI8658
  initScreen(); // Broche pour l'ecran OLED SSD1306
  initIHM(); // IHM serveur WEB
  initPump(); // Broche de la pompe peristaltique
  initSD(); // Initialisation de la communication SD
}

void loop(void) {
  screenUpdate();
  constantHandle();
  RoArmM2_getPosByServoFeedback();
}
