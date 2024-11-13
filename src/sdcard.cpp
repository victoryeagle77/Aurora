#include "../include/sdcard_config.h"

// Initialisation de la connexion SPI
SPIClass spi(HSPI);

/**
* @brief readFile
* Lecture dans un fichier venant de la carte SD
* @param path : Chemin du fichier
*/
extern String readFile(const char* path) {
  if (!(SD.begin(CS,spi,FREQ_SPI)))
    return "NULL";

  String data;
  File file = SD.open(path, FILE_READ);

  if (file) {
    while (file.available())
      data += (char)file.read();
    file.close();
  } else { return "NULL"; }

  return data;
}

/**
* @brief writeFile
* Ecriture dans un fichier par remplacement de donnees venant de la carte SD
* @param path : Chemin du fichier
* @param data : Chaine de caractere a ecrire
*/
extern void writeFile(const char* path, String data) {
  if (!(SD.begin(CS,spi,FREQ_SPI)))
    return;

  File file = SD.open(path, FILE_WRITE);
  if (file) {
    file.println(data);
    file.close();
  }
}

/**
* @brief appendFile
* Ecriture dans un fichier venant de la carte SD a la suite des donnees
* @param path : Chemin du fichier
* @param data : Chaine de caractere a ecrire
*/
extern void appendFile(const char* path, String data) {
  if (!(SD.begin(CS,spi,FREQ_SPI)))
    return;

  File file = SD.open(path, FILE_APPEND);
  if (file) {
    file.println(data);
    file.close();
  }
}

/**
* @brief deleteFile
* Suppresion d'un fichier venant de la carte SD
* @param path : Chemin du fichier
*/
extern void deleteFile(const char* path) {
  if (!(SD.begin(CS,spi,FREQ_SPI)))
    return;

  if (SD.exists(path))
    SD.remove(path);
}

/**
* @brief infoSD
* Releve des informations concernant la carte SD
* @return dataSD : Tableau de parametres de la carte SD
*/
extern String* infoSD(void){
  static String dataSD[2];

  if (!(SD.begin(CS,spi,FREQ_SPI))) {
    dataSD[0]="NULL";
    dataSD[1]="NULL";
    return dataSD;
  }

  uint8_t type_sd = SD.cardType();
  uint64_t size_sd = SD.cardSize();
  uint64_t used_sd = SD.usedBytes();
  float level_sd = (((float)used_sd) * 100) / ((float)size_sd);

  switch (type_sd) {
    case CARD_NONE:
      dataSD[0] = "NONE";
    break;
    case CARD_MMC:
      dataSD[0] = "MMC";
    break;
    case CARD_SD:
      dataSD[0] = "SDSC";
    break;
    case CARD_SDHC:
      dataSD[0] = "SDHC";
    break;
    default:
      dataSD[0] = "UNKNOWN";
    break;
  }

  dataSD[1] = String(level_sd, 2);
  return dataSD;
}

/**
* @brief initSD
* Initialisation de la communication SPI pour la carte SD
*/
extern void initSD(void){
  spi.begin(SCK, MISO, MOSI, CS);

  if (!(SD.begin(CS,spi,FREQ_SPI)))
    return;
}