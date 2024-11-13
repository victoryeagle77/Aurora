#ifndef AXL_CONFIG_H
#define AXL_CONFIG_H

#include <Wire.h>

#define SCL_AXL 33
#define SDA_AXL 32

void initIMU(void);
int* getIMU(void);

#endif
