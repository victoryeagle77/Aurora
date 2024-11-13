#include "../include/motors_config.h"

/**
* @brief initMotors : Initialisation des broches pour les moteurs JETANK 20770
* Configuration en entree/sortie PWM par la frequence et la voie des moteurs
*/
extern void initMotors(void){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcSetup(CHA, FREQ_M, AWB);
  ledcAttachPin(PWMA, CHA);

  ledcSetup(CHB, FREQ_M, AWB);
  ledcAttachPin(PWMB, CHB);
}

/**
* @brief bcka : Marche arriere sur les moteurs droit
* @param pwm : Modulation par largeur d'impulsion a effectuer par les moterus
*/
void fwda(uint16_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(CHA, pwm);
}

/**
* @brief bckb : Marche arriere sur les moteurs gauche
* @param pwm : Modulation par largeur d'impulsion a effectuer par les moterus
*/
void fwdb(uint16_t pwm){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(CHB, pwm);
}

/**
* @brief fwda : Marche avant sur les moteurs droit
* @param pwm : Modulation par largeur d'impulsion a effectuer par les moterus
*/
void bcka(uint16_t pwm){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(CHA, pwm);
}

/**
* @brief fwdb : Marche avant sur les moteurs gauche
* @param pwm : Modulation par largeur d'impulsion a effectuer par les moterus
*/
void bckb(uint16_t pwm){
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(CHB, pwm);
}

/**
* @brief directionAB : Cas d'usage directionnel en fonction de la rotation des moteurs JETANK 20770
* @param dir : Definition de la direction a adopter
*/
extern void directionAB(const char dir){
  switch (dir) {
      // Droite
      case 0:
        fwda(MAX_SPD);
        bckb(MAX_SPD);
      break;
      // Gauche
      case 1:
        bcka(MAX_SPD);
        fwdb(MAX_SPD);
      break;
      // Marche avant
      case 2:
        fwda(MAX_SPD);
        fwdb(MAX_SPD);
      break;
      // Marche arriere
      case 3:
        bcka(MAX_SPD);
        bckb(MAX_SPD);
      break;
      // Marche avant droite
      case 4:
        fwda(MAX_SPD);
        fwdb(MIN_SPD);
      break;
      // Marche avant gauche
      case 5:
        fwda(MIN_SPD);
        fwdb(MAX_SPD);
      break;
      // Marche arriere droite
      case 6:
        bcka(MAX_SPD);
        bckb(MIN_SPD);
      break;
      // Marche arriere gauche
      case 7:
        bcka(MIN_SPD);
        bckb(MAX_SPD);
      break;
      // Stopper les moteurs
      case 8: 
        fwda(0);
        fwdb(0);
      break;
  }
}
