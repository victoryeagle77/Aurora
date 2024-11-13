#include "../include/sdcard_config.h"
#include "../include/pump_config.h"

/**
* @brief initPump
* Parametrage d'utilsation de la pompe par un transistor MOSFET
*/
extern void initPump(void) {
    pinMode(PUMP, OUTPUT);
    digitalWrite(PUMP, LOW);
}

/**
* @brief activePump
* Active ou desactive la pompe peristaltique et ecrit le temps
* @param flag : Cas d'usage de la pompe
*/
extern void activePump(const char flag) {
    unsigned long start_time=0, end_time=0;
    float save_time=0;

    switch (flag){
        // Initialisation du temps et activation de la broche
        case 0:
            start_time = millis();
            digitalWrite(PUMP, HIGH);
        break;
        // Arret du compteur de temps et desactivation de la broche
        case 1:
            digitalWrite(PUMP, LOW);
            end_time += (millis() - start_time);
            save_time = (readFile(LOG).toFloat() + ((float)end_time)) / 10000;
            writeFile(LOG, String(save_time));
        break;
        // Reinitialisation de l'estimation de la cuve
        case 2:
            writeFile(LOG, INIT);
        break;
    }
}

/**
* @brief infoPump
* Selon la tension de la pompe, mesure le temps d'ecoulement de la cuve
* @return level_time : Niveau de la cuve restant
*/
extern float infoPump(void) {
    float min=0, max=0;
    float flow=0, ref_time=0, empty_time=0;
    float save_time=0, level_time=0;

    if (VOLTAGE == 3.3) {
        max = MAX3V;
        min = MIN3V;
    } else if (VOLTAGE == 6) {
        max = MAX6V;
        min = MIN6V;
    }

    flow = ((max + min) / 2) / 60; // Debit de reference en L/sec
    ref_time = 1/flow; // Temps de reference en secondes
    empty_time = CAPACITY * ref_time; // Temps de vidage de la cuve
    save_time = readFile(LOG).toFloat();
    level_time = 100 - ((save_time * 100) / empty_time); // Niveau de liquide restant

    if (level_time <= 0)
        level_time = 0;

    return level_time;
}
