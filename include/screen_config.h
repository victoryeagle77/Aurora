#ifndef SCREEN_CONFIG_H_
#define SCREEN_CONFIG_H_

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCL_OLED 33
#define SDA_OLED 32

#define IP "192.168.4.1"

#define WIDTH 128 // OLED display width, in pixels
#define HEIGHT 32 // OLED display height, in pixels
#define RESET -1 // Broche de reinitialisation
#define ADDR_OLED 0x3C // Adresse I2C de l'ecran OLED SSD1306 en resolution 128x32

void initScreen(void);
void screenUpdate(void);

#endif