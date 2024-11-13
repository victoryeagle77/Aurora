#ifndef SDCARD_CONFIG_H_
#define SDCARD_CONFIG_H_

#include <SD.h>
#include <SPI.h>

#define SCK 14
#define MISO 12
#define MOSI 13
#define CS 15
#define FREQ_SPI 80000000

void initSD(void);
String readFile(const char*);
String* infoSD(void);
void writeFile(const char*, String);
void deleteFile(const char*);

#endif