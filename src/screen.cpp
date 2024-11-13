#include "../include/screen_config.h"
#include "../include/ina219_config.h"

#define SSID "aurora2"

Adafruit_SSD1306 display(WIDTH, HEIGHT, &Wire, RESET);

/**
* @brief initScreen : Initialisation de l'ecran OLED SSD1306
* Rafraichissement et preparation a l'affichage
*/
extern void initScreen(void){
  Wire.begin(SDA_OLED, SCL_OLED);
  if(!(display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)))
    return;
  display.clearDisplay();
  display.display();
}

/**
* @brief screenUpdate : Gestion de l'affichage sur l'ecran OLED SSD1306
* Affichage de plusieurs parametres utiles sous formes de chaines de caracteres
*/
extern void screenUpdate(void){
  char buffer1[20], buffer2[40], buffer3[30];
  float voltage = readVoltage();
  float level = batteryLevel();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  sprintf(buffer1, "Battery:%.f%%", level);
  sprintf(buffer2, "SSID:%s", SSID);
  sprintf(buffer3, "IP:%s", IP);

  display.println(F(buffer1));
  display.println(F(buffer2));
  display.print(F(buffer3));
  display.display();
}
