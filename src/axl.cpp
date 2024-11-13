#include "../include/axl_config.h"
#include "IMU/IMU.h"

/**
* @brief getIMU : Permet d'obtenir les donnees des modules AK09918 et QMI8658
* @return dataIMU[0] : Rotation en X
* @return dataIMU[1] : Tangage en Y
* @return dataIMU[2] : Inclinaison en Z
* @return dataIMU[3] : Magnetisme
*/
extern int* getIMU(void){
  static IMU_ST_ANGLES_DATA stAngles;
  static IMU_ST_SENSOR_DATA stGyroRawData;
  static IMU_ST_SENSOR_DATA stAccelRawData;
  static IMU_ST_SENSOR_DATA stMagnRawData;
  static int dataIMU[4];

  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  dataIMU[0] = stAngles.fYaw;
  dataIMU[1] = stAngles.fRoll;
  dataIMU[2] = stAngles.fPitch;
  dataIMU[3] = QMI8658_readTemp();
  return dataIMU;
}

/**
* @brief initIMU : Initialisation des modules AK09918 et QMI8658
*/
extern void initIMU(void) {
  Wire.begin(SDA_AXL, SCL_AXL);
  imuInit();
}
