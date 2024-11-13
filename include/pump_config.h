#ifndef PUMP_CONFIG_H_
#define PUMP_CONFIG_H_

/**
* Moteur RC Boat 370 : Pompe peristaltique pour bateau de modelisme
* [[ DATASHEET ]]
*  - Debit de pompage pour une tension de 3V : Entre 1.1L/min et 300ml/min
*  - Debit de pompage pour une tension de 6V : Entre 1.8L/min et 600ml/min
*  - Fonctionnement de 15h sans interruptions
*/

#define PUMP 4 // Broche d'activation
#define VOLTAGE 3.3 // Tension de fonctionnement
#define CAPACITY 5 // Nombre de litres dans la cuve

#define MIN3V 0.300
#define MAX3V 1.1
#define MIN6V 0.600
#define MAX6V 1.8

#define LOG "/vat_data.txt"
#define INIT "0"

void initPump(void);
void activePump(const char);
float infoPump(void);

#endif