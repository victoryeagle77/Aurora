#ifndef MOTORS_CONFIG_H_
#define MOTORS_CONFIG_H_

#include <Arduino.h>

// Configuration PWM des moteurs droits et gauches
#define PWMA 25
#define PWMB 26
#define AIN2 17
#define AIN1 21
#define BIN1 22
#define BIN2 23

#define AWB 8 // Ecriture analogique sur 8 bits
#define FREQ_M 100000 // Frequence fixe de fonctionnement des moteurs JETANK 20770

#define CHA 0 // Voie des moteurs droits
#define CHB 1 // Voie des moteurs gauches

#define MAX_SPD 512 // Vitesse de rotation maximale 2^9
#define MIN_SPD 128 // Vitesse de rotation medianne 2^7

void initMotors(void);
void directionAB(const char);

#endif