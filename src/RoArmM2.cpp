#include "../include/RoArmM2_config.h"

#include "SCServo/SCSCL.h"
#include "SCServo/SMS_STS.h"

// Articulation du coude : Distance entre les servomteurs
float l2 = sqrt(ARM_L2_LENGTH_MM_A * ARM_L2_LENGTH_MM_A + ARM_L2_LENGTH_MM_B * ARM_L2_LENGTH_MM_B);
// Articulation du coude : Angle d'articulation
float t2rad = atan2(ARM_L2_LENGTH_MM_B, ARM_L2_LENGTH_MM_A);

// Articulation de la main : Distance entre les servomteurs
float l3 = sqrt(ARM_L3_LENGTH_MM_A_0 * ARM_L3_LENGTH_MM_A_0 + ARM_L3_LENGTH_MM_B_0 * ARM_L3_LENGTH_MM_B_0);
// Articulation du coude : Angle d'articulation
float t3rad = atan2(ARM_L3_LENGTH_MM_B_0, ARM_L3_LENGTH_MM_A_0);

float goalX = ARM_L3_LENGTH_MM_A_0 + ARM_L2_LENGTH_MM_B; // Initialisation de l'axe X
float goalY = 0; // Initialisation de l'axe Y
float goalZ = ARM_L2_LENGTH_MM_A - ARM_L3_LENGTH_MM_B_0; // Initialisation de l'axe Z
float goalT = M_PI;

float lastX = goalX;
float lastY = goalY;
float lastZ = goalZ;
float lastT = goalT;

double base_r;
float radB, radS, radE, radG;

double BASE_JOINT_RAD = 0;
double SHOULDER_JOINT_RAD = 0;
double ELBOW_JOINT_RAD = M_PI/2;
double HAND_JOINT_RAD = M_PI;
double HAND_JOINT_RAD_BUFFER;

bool nanIK;
bool RoArmM2_initCheckSucceed = false;

// Mouvements constants
float const_spd;
float const_goal_base = BASE_JOINT_ANG;
float const_goal_shoulder = SHOULDER_JOINT_ANG;
float const_goal_elbow = ELBOW_JOINT_ANG;
float const_goal_hand = HAND_JOINT_ANG;

byte const_mode;
byte const_cmd_base_x;
byte const_cmd_shoulder_y;
byte const_cmd_elbow_z;
byte const_cmd_hand_t;

// Arguments d'ecriture synchrone de positionnement
uint8_t servoID[5] = { BASE_SERVO_ID, SHOULDER_A_SERVO_ID, SHOULDER_B_SERVO_ID, 
                        ELBOW_SERVO_ID, HAND_SERVO_ID };
int16_t goalPos[5] = { ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS, 
                        ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS, ARM_SERVO_MIDDLE_POS };
uint16_t moveSpd[5] = { 0, 0, 0, 0, 0 };
uint8_t moveAcc[5] = { ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC, 
                        ARM_SERVO_INIT_ACC, ARM_SERVO_INIT_ACC };

SMS_STS st;

// Structure contenant des informations sur les servomoteurs
struct ServoFeedback {
  bool status;
  int pos, speed, load;
  float voltage, current, temper;
  byte mode;
};

// [0] BASE_SERVO_ID
// [1] SHOULDER_A_SERVO_ID
// [2] SHOULDER_B_SERVO_ID
// [3] ELBOW_SERVO_ID
// [4] HAND_SERVO_ID
ServoFeedback servoFeedback[5];

/**
 * @brief calculatePosByRad
 * Calcul de la position incrementale d'un servo-moteur via un angle en radian
 * Pos = (Angle / 2*pi) * 4096
 * @param radInput Angle en radian
 * @return Position incrementale du servo-moteur
*/
double calculatePosByRad(double radInput) {
  return round((radInput / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

/**
 * @brief calculateRadByFeedback
 * Calcul l'angle de deplacement d'un servo-moteur via son incrementation
 * Angle = (Pos*2*pi / 4096) +- (k*pi)
 * @param inputSteps Nombre d'etapes incrementales faites par le servo-moteur 
 * @param jointName Servo-moteur de jonction du bras robotique
 * @return L'Angle de la jonction en radians
*/
double calculateRadByFeedback(int inputSteps, int jointName) {
  double getRad;
  switch(jointName){
    case BASE_JOINT:
      getRad = -(inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) + M_PI;
    break;
    case SHOULDER_JOINT:
      getRad = (inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) - M_PI;
    break;
    case ELBOW_JOINT:
      getRad = (inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) - (M_PI / 2);
    break;
    case HAND_JOINT:
      getRad = inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE;
    break;
  }
  return getRad;
}

/**
 * @brief getFeedback
 * Retour des informations concernant un servo-moteur via son ID
 * @param servoID Identifiant du servo-moteur
 * @param returnType Type de retour a realiser
 * @return false pour renvoyer tous les parametres
 * @return true pour renvoyer uniquement les cas d'echec
*/
bool getFeedback(byte servoID, bool returnType) {
  if(st.FeedBack(servoID)!=-1) {
    servoFeedback[servoID - 11].status = true;
    servoFeedback[servoID - 11].pos = st.ReadPos(-1);
    servoFeedback[servoID - 11].speed = st.ReadSpeed(-1);
    servoFeedback[servoID - 11].load = st.ReadLoad(-1);
    servoFeedback[servoID - 11].voltage = st.ReadVoltage(-1);
    servoFeedback[servoID - 11].current = st.ReadCurrent(-1);
    servoFeedback[servoID - 11].temper = st.ReadTemper(-1);
    servoFeedback[servoID - 11].mode = st.ReadMode(servoID);
    if(!returnType)
        return false;

    return true;

  } else {
    servoFeedback[servoID - 11].status = false;
  	return false;
  }
}

/**
 * @param setMiddlePos
 * Effectue la calibration de la position courante d'un servo-moteur, comme etant sa position mediane
 * @param InputID Identifiant du servo-moteur a calibrer
*/
extern void setMiddlePos(byte InputID){
  st.CalibrationOfs(InputID);
}

/**
 * @brief waitMove2Goal
 * Met en place un delai de latence pour qu'un servo-moteur atteigne une position
 * @param InputID Identifiant du servo-moteur a controler
 * @param goalPosition Position finale a atteindre
 * @param offSet Indice de decalage pour atteindre la bonne position
*/
void waitMove2Goal(byte InputID, int16_t goalPosition, int16_t offSet){
  while((servoFeedback[InputID - 11].pos < goalPosition - offSet) || 
        (servoFeedback[InputID - 11].pos > goalPosition + offSet)){

    getFeedback(InputID, true);
    delay(10);
  }
}

/**
 * @brief RoArmM2_servoInit
 * Initialisation des servomoteurs sur le bus UART via niveaux de tensions de type TTL
 * Transistor-Transistor Logic : 0V pour le niveau logique bas et 5V pour le niveau logique haut
*/
extern void RoArmM2_servoInit(void){
  Serial1.begin(FREQ_UART, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while(!Serial1);
}

/**
 * @brief RoArmM2_initCheck
 * Verificiation du status de tous les servomoteurs
 * @param returnType Si tous les status sont OK, l'initialisation vaut 1 et 0 sinon
*/
extern void RoArmM2_initCheck(bool returnType) {
  RoArmM2_initCheckSucceed = false;
  RoArmM2_initCheckSucceed = getFeedback(BASE_SERVO_ID, true) &&
                             getFeedback(SHOULDER_A_SERVO_ID, true) &&
                             getFeedback(SHOULDER_B_SERVO_ID, true) &&
                             getFeedback(ELBOW_SERVO_ID, true);
}

/**
 * @brief setServosPID
 * Configure tous les PID des servomoteurs selon les parametres du RoArm-M2
*/
extern bool setServosPID(byte InputID, byte InputP) {
  if(!getFeedback(InputID, true))
    return false;

  st.unLockEprom(InputID);
  st.writeByte(InputID, ST_PID_P_ADDR, InputP); 
  st.LockEprom(InputID);
  return true;
}

/**
 * @brief RoArmM2_baseJointCtrlRad
 * Calcule de position et controle la position d'une jonction entre deux servos-moteurs : Base du bras robotique
 * @param returnType Retourne la position de la jonction et la conserve
 *                   0 : Aucun deplacement des servomoteurs
 *                   1 : Deplacement des servomoteurs
 * @param radInput Angle en radian
 * @param speedInput Vitesse des servos-moteurs en incrementations par secondes
 * @param accInput Acceleration des mouvements des servos-moteurs
 * @return Position finale calculee
*/
int RoArmM2_baseJointCtrlRad(byte returnType, double radInput, uint16_t speedInput, uint8_t accInput) {
  radInput = -constrain(radInput, -M_PI, M_PI);
  // Rad + 4096
  int16_t computePos = calculatePosByRad(radInput) + ARM_SERVO_MIDDLE_POS;
  goalPos[0] = computePos;

  if(returnType)
    st.WritePosEx(BASE_SERVO_ID, goalPos[0], speedInput, accInput);

  return goalPos[0];
}

/**
 * @brief RoArmM2_shoulderJointCtrlRad
 * Calcule de position et controle d'une jonction entre deux servos-moteurs : Epaule du bras robotique
 * La jonction de l'epaule etant constituee de 2 servomoteurs (droit et gauche)
 * @param returnType Retourne la position de la jonction et la conserve
 *                   0 : Aucun deplacement des servomoteurs
 *                   1 : Deplacement des servomoteurs
 * @param radInput Angle en radian
 * @param speedInput Vitesse des servos-moteurs en incrementations par secondes
 * @param accInput Acceleration des mouvements des servos-moteurs
 * @return Position finale calculee
*/
int RoArmM2_shoulderJointCtrlRad(byte returnType, double radInput, uint16_t speedInput, uint8_t accInput) {
  radInput = constrain(radInput, -M_PI/2, M_PI/2);
  int16_t computePos = calculatePosByRad(radInput);
  int pos;
  goalPos[1] = ARM_SERVO_MIDDLE_POS + computePos;
  goalPos[2] = ARM_SERVO_MIDDLE_POS - computePos;

  if(returnType == 1){
    st.WritePosEx(SHOULDER_A_SERVO_ID, goalPos[1], speedInput, accInput);
    st.WritePosEx(SHOULDER_B_SERVO_ID, goalPos[2], speedInput, accInput);
  }else if(returnType == SHOULDER_A_SERVO_ID){
    pos = goalPos[1];
  }else if(returnType == SHOULDER_B_SERVO_ID){
    pos = goalPos[2];
  }

  return pos;
}

/**
 * @brief RoArmM2_elbowJointCtrlRad
 * Calcule de position et controle d'une jonction entre deux servos-moteurs : Coude du bras robotique
 * @param returnType Retourne la position de la jonction et la conserve
 *                   0 : Aucun deplacement des servomoteurs
 *                   1 : Deplacement des servomoteurs
 * @param radInput Angle en radian
 * @param speedInput Vitesse des servos-moteurs en incrementations par secondes
 * @param accInput Acceleration des mouvements des servos-moteurs
 * @return Position finale calculee
*/
int RoArmM2_elbowJointCtrlRad(byte returnType, double radInput, uint16_t speedInput, uint8_t accInput) {
  int16_t computePos = calculatePosByRad(radInput) + 1024;
  goalPos[3] = constrain(computePos, 512, 3071);

  if(returnType)
    st.WritePosEx(ELBOW_SERVO_ID, goalPos[3], speedInput, accInput);

  return goalPos[3];
}

/**
 * @brief RoArmM2_handJointCtrlRad
 * Calcule de position et controle d'une jonction entre deux servos-moteurs : Main du bras robotique
 * @param returnType Retourne la position de la jonction et la conserve
 *                   0 : Aucun deplacement des servomoteurs
 *                   1 : Deplacement des servomoteurs
 * @param radInput Angle en radian
 * @param speedInput Vitesse des servos-moteurs en incrementations par secondes
 * @param accInput Acceleration des mouvements des servos-moteurs
 * @return Position finale calculee
*/
int RoArmM2_handJointCtrlRad(byte returnType, double radInput, uint16_t speedInput, uint8_t accInput) {
  int16_t computePos = calculatePosByRad(radInput);
  goalPos[4] = constrain(computePos, 700, 3396);

  if (returnType)
    st.WritePosEx(HAND_SERVO_ID, goalPos[4], speedInput, accInput);

  return goalPos[4];
}

/** 
 * @brief setNewAxisX
 * Initialise dans le plan Cartesien une nouvelle abscisse
 * @param angleInput Angle relatif a l'entree de commande
*/
extern void setNewAxisX(double angleInput) {
  // Conversion de radian a degre : rad = (deg/180)*pi
  double radInput = (angleInput / 180) * M_PI;

  RoArmM2_shoulderJointCtrlRad(1, 0, 500, ARM_SERVO_INIT_ACC);
  waitMove2Goal(SHOULDER_A_SERVO_ID, goalPos[1], ARM_SERVO_INIT_ACC);

  RoArmM2_elbowJointCtrlRad(1, 0, 500, ARM_SERVO_INIT_ACC);
  waitMove2Goal(ELBOW_SERVO_ID, goalPos[3], ARM_SERVO_INIT_ACC);

  RoArmM2_baseJointCtrlRad(1, 0, 500, ARM_SERVO_INIT_ACC);
  waitMove2Goal(BASE_SERVO_ID, goalPos[0], ARM_SERVO_INIT_ACC);

  delay(1000);

  RoArmM2_baseJointCtrlRad(1, -radInput, 500, ARM_SERVO_INIT_ACC);
  waitMove2Goal(BASE_SERVO_ID, goalPos[0], ARM_SERVO_INIT_ACC);

  delay(1000);

  setMiddlePos(BASE_SERVO_ID);

  delay(5);
}

/**
 * @brief simpleLinkageIkRad
 * Calcule avec les parametres de la cinematique inverse, les positions et les rotations du modele articulaire,
 * qui est en l'occurrence le bras robotique, afin d'obtenir une pose desiree
 * @param LA : Longueur de la 1ere articulation
 * @param LB : Longueur de la 2nde articulation
 * @param aIn : Coordoonnees x de l'effecteur
 * @param bIn : Coordoonnees y de l'effecteur
*/
void simpleLinkageIkRad(double LA, double LB, double aIn, double bIn) {
  double L2C, LC; 
  double alpha; // Angle d’articulation de l’epaule par rapport a l’horizontal
  double beta; // Angle d’articulation du coude par rapport à l'articulation de l’epaule
  double lambda; // Angle entre l’axe du bras, et la droite passant par l’articulation de l’epaule et de l’extremite de l’effecteur
  double delta; // Angle d’articulation de l’effecteur par rapport a l’horizontal
  double psi; // Angle d’articulation du coude par rapport aux liens inferieur et superieur
  double omega; // Angle d’articulation du coude par rapport a l’horizontal

  if (fabs(bIn) < 1e-6) {
    psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
    alpha = M_PI / 2.0 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
    beta = psi + omega - t3rad;
  } else {
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan2(bIn, aIn);
    psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad; // Elevation de l’articulation de l’epaule et l’angle de flexion du coude
    alpha = M_PI / 2.0 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
    beta = psi + omega - t3rad;
  }

  delta = M_PI / 2.0 - alpha - beta;

  SHOULDER_JOINT_RAD = alpha;
  ELBOW_JOINT_RAD = beta;
  HAND_JOINT_RAD_BUFFER = delta;

  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
}

/**
 * @brief cartesianToPolar
 * Convertir les donnees d'un systeme de coordonnees Cartesiennes (x,y) en coordonnees polaires (r,theta)
 * @param x Abscisse du plan Cartesien
 * @param y Ordonnees du plan Cartesien
 * @param r Distance par rapport a l'origine du repere
 * @param theta Angle polaire en radians definit sur (-pi, pi)
*/
void cartesianToPolar(double x, double y, double* r, double* theta) {
    *r = sqrt(x * x + y * y);
    *theta = atan2(y, x); // tan x = sin(x)/cos(x)
}

/**
 * @brief polarToCartesian
 * Convertir les donnees d'un systeme de coordonnees polaire (r,theta) en coordonnees Cartesiennes (x,y)
 * @param r Distance par rapport a l'origine du repere
 * @param theta Angle en radians definit sur (-pi, pi)
 * @param x Abscisse du plan Cartesien
 * @param y Ordonnees du plan Cartesien
*/
void polarToCartesian(double r, double theta, double &x, double &y) {
  x = r * cos(theta);
  y = r * sin(theta);
}

/**
 * @brief RoArmM2_computePosbyJointRad
 * Permet de calculer le point de butee de chaque jonction (point limite qu'atteint un servomoteur)
 * Sauvegarde les derniers position calculees sur chaque axe du plan P(x,y,z)
 * @param base_joint_rad Angle en radian de la base
 * @param shoulder_joint_rad Angle en radian de l'epaule
 * @param elbow_joint_rad Angle en radian du coude
 * @param hand_joint_rad Angle en radian de la main
*/
void RoArmM2_computePosbyJointRad(double base_joint_rad, double shoulder_joint_rad, double elbow_joint_rad, double hand_joint_rad) {
    double r_ee, x_ee, y_ee, z_ee;
    double aOut, bOut, cOut, dOut, eOut, fOut;

    // Butee du lien entre la base et l'epaule
    polarToCartesian(l2, ((M_PI / 2) - (shoulder_joint_rad + t2rad)), aOut, bOut);
    // Butee du lien entre l'epaule et le coude
    polarToCartesian(l3, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad)), cOut, dOut);

    r_ee = aOut + cOut;
    z_ee = bOut + dOut;
    
    polarToCartesian(r_ee, base_joint_rad, eOut, fOut);
    x_ee = eOut;
    y_ee = fOut;

    // Sauvegarde des position dans le plan P(x,y,z)
    lastX = x_ee;
    lastY = y_ee;
    lastZ = z_ee;

}

/**
 * @brief RoArmM2_getPosByServoFeedback
 * Renvoie des informations de positionnement angulaire en radian de chaque servo-moteur
*/
extern void RoArmM2_getPosByServoFeedback(void) {
  getFeedback(BASE_SERVO_ID, true);
  getFeedback(SHOULDER_A_SERVO_ID, true);
  getFeedback(ELBOW_SERVO_ID, true);
  getFeedback(HAND_SERVO_ID, true);

  radB = calculateRadByFeedback(servoFeedback[BASE_SERVO_ID - 11].pos, BASE_JOINT);
  radS = calculateRadByFeedback(servoFeedback[SHOULDER_A_SERVO_ID - 11].pos, SHOULDER_JOINT);
  radE = calculateRadByFeedback(servoFeedback[ELBOW_SERVO_ID - 11].pos, ELBOW_JOINT);
  radG = calculateRadByFeedback(servoFeedback[HAND_SERVO_ID - 11].pos, HAND_JOINT);

  RoArmM2_computePosbyJointRad(radB, radS, radE, radG);
  lastT = radG;
}

/**
 * @brief RoArmM2_infoFeedback
 * Retourne une chaine JSON de parametre des servos-moteurs
 * @return Chaine formattee en JSON
*/
extern String RoArmM2_infoFeedback(void) {
    String info = "{";
    info += "\"Axe x\":\"" + String(lastX) + "\",";
    info += "\"Axe y\":\"" + String(lastY) + "\",";
    info += "\"Axe z\":\"" + String(lastZ) + "\",";
    info += "\"Base\":\"" + String(radB) + "\",";
    info += "\"Epaule\":\"" + String(radS) + "\",";
    info += "\"Coude\":\"" + String(radE) + "\",";
    info += "\"Main\":\"" + String(lastT) + "\"";
    info += "}";

    return info;
}

/**
 * @brief RoArmM2_baseCoordinateCtrl
 * Point de coordonnees de la position a atteindre pour calculer la position "objectif" de toutes les jonctions 
*/
void RoArmM2_baseCoordinateCtrl(double inputX, double inputY, double inputZ, double inputT){
    cartesianToPolar(inputX, inputY, &base_r, &BASE_JOINT_RAD);
    simpleLinkageIkRad(l2, l3, base_r, inputZ);
    RoArmM2_handJointCtrlRad(0, inputT, 0, 0);
}

/**
 * @brief RoArmM2_lastPosUpdate
 * Mise a jour de la dernier position pour utilisation ulterieure
*/
void RoArmM2_lastPosUpdate(void){
  lastX = goalX;
  lastY = goalY;
  lastZ = goalZ;
  lastT = goalT;
}

/**
 * @brief RoArmM2_goalPosMove
 * Utilise l'anlge de controle directionnel de chaque jonction pour calculer la position a atteindre pour chaque servomoteurs, et les deplacer en consequence
*/
void RoArmM2_goalPosMove(void){
  RoArmM2_baseJointCtrlRad(0, BASE_JOINT_RAD, 0, 0);
  RoArmM2_shoulderJointCtrlRad(0, SHOULDER_JOINT_RAD, 0, 0);
  RoArmM2_elbowJointCtrlRad(0, ELBOW_JOINT_RAD, 0, 0);
  RoArmM2_handJointCtrlRad(0, HAND_JOINT_RAD, 0, 0);
 
  st.SyncWritePosEx(servoID, 5, goalPos, moveSpd, moveAcc);
}

/**
 * @brief RoArmM2_allJointAbsCtrl
 * Permet de controler toutes les articulations simultanement dans le plan P(x,y,z)
 * @param inputBase Angle en radian de deplacement de la base
 * @param inputShoulder Angle en radian de deplacement de l'eapule
 * @param inputElbow Angle en radian de deplacement du coude
 * @param inputHand Angle en radian de deplacement de la main
 * @param inputSpd Vitesse de deplacement du servomoteur
 * @param inputAcc Acceleration de mouvement du servomoteur
*/
extern void RoArmM2_allJointAbsCtrl(double inputBase, double inputShoulder, double inputElbow, 
                              double inputHand, uint16_t inputSpd, uint8_t inputAcc){
  RoArmM2_baseJointCtrlRad(0, inputBase, inputSpd, inputAcc);
  RoArmM2_shoulderJointCtrlRad(0, inputShoulder, inputSpd, inputAcc);
  RoArmM2_elbowJointCtrlRad(0, inputElbow, inputSpd, inputAcc);
  RoArmM2_handJointCtrlRad(0, inputHand, inputSpd, inputAcc);

  for (int i = 0; i < 5; i++) {
    moveSpd[i] = inputSpd;
    moveAcc[i] = inputAcc;
  }

  st.SyncWritePosEx(servoID, 5, goalPos, moveSpd, moveAcc);

  for (int i = 0; i < 5; i++) {
    moveSpd[i] = 0;
    moveAcc[i] = 0;
  }
}

/**
 * @brief RoArmM2_setJointPID
 * Configure les du parametre du correcteur PID (Proportionnel-Integral-Derive) 
 * Calibration PID des jonctions du bras robotique
 * @param jointInput servomoteurs constituant une jonction du bras robotique
 *                   1 = BASE
 *                   2 = EPAULE
 *                   3 = COUDE
 *                   4 = MAIN
 * @param p Calibration du parametre Proportionnel du PID 
 *          32 = Configuration servomoteur unique
 *          16 = Configuration RoArmM2
 * @param i Calibration du parametre Integral du PID 
 *          0 = Configuration servomoteur unique
 *          8 = Configuration RoArm-M2 PID MODE ON
 * Calibration du parametre Dervie non utilise par defaut
*/
extern void RoArmM2_setJointPID(byte jointInput, float inputP, float inputI) {
  switch (jointInput) {
    case BASE_JOINT:
      st.writeByte(BASE_SERVO_ID, ST_PID_P_ADDR, inputP);
      st.writeByte(BASE_SERVO_ID, ST_PID_I_ADDR, inputI);
    break;

    case SHOULDER_JOINT:
      st.writeByte(SHOULDER_A_SERVO_ID, ST_PID_P_ADDR, inputP);
      st.writeByte(SHOULDER_A_SERVO_ID, ST_PID_I_ADDR, inputI);
      st.writeByte(SHOULDER_B_SERVO_ID, ST_PID_P_ADDR, inputP);
      st.writeByte(SHOULDER_B_SERVO_ID, ST_PID_I_ADDR, inputI);
    break;

    case ELBOW_JOINT:
      st.writeByte(ELBOW_SERVO_ID, ST_PID_P_ADDR, inputP);
      st.writeByte(ELBOW_SERVO_ID, ST_PID_I_ADDR, inputI);
    break;

    case HAND_JOINT:
      st.writeByte(HAND_SERVO_ID, ST_PID_P_ADDR, inputP);
      st.writeByte(HAND_SERVO_ID, ST_PID_I_ADDR, inputI);
    break;
  }
}

/**
 * @brief RoArmM2_resetPID
 * Reinitialise les process de calibration de chaque servomoteurs du bras robotique
*/
extern void RoArmM2_resetPID(void) {
  RoArmM2_setJointPID(BASE_JOINT, 16, 0);
  RoArmM2_setJointPID(SHOULDER_JOINT, 16, 0);
  RoArmM2_setJointPID(ELBOW_JOINT, 16, 0);
  RoArmM2_setJointPID(HAND_JOINT, 16, 0);
}

/** 
 * @brief constantCtrl
 * Controle de servomoteur par parametrage de constante
 * @param inputMode 0 = Controle par parametrage angulaire
 *                  1 = Controle par parametrage planaire P(x,y,z) et temporel "t"
 * @param inputAxis Axe de deplacement dans le plan P(x,y,z)
 * @param inputCmd 0 = Arret du servomoteur
 *                 1 = Augmentation de la vitesse du servomoteur
 *                 2 = Diminution de la vitesse du servomoteur
 * @param inputSpd Vitesse de rotation du servomoteur
*/
extern void constantCtrl(byte inputMode, byte inputAxis, byte inputCmd, byte inputSpd) {
  const_mode = inputMode;
  if (const_mode == CONST_ANGLE)
    const_spd = abs(inputSpd) * 0.0005;
  else if (const_mode == CONST_XYZT)
    const_spd = abs(inputSpd) * 0.1;

  switch (inputAxis) {
    case BASE_JOINT:
      const_cmd_base_x = inputCmd;
    break;
    case SHOULDER_JOINT:
      const_cmd_shoulder_y = inputCmd;
    break;
    case ELBOW_JOINT:
      const_cmd_elbow_z = inputCmd;
    break;
    case HAND_JOINT:
      const_cmd_hand_t = inputCmd;
    break;
  }
}

/**
 * @brief constantHandle
 * Permet de determiner le type de controle d'un ou plusieurs servomoteurs 
 * Determine la nature permanante ou temporaire de leur deplacement
 * Limite le deplacement des servomoteurs avant d'atteindre la butee
*/
extern void constantHandle(void) {
  if (!const_cmd_base_x && !const_cmd_shoulder_y && !const_cmd_elbow_z && !const_cmd_hand_t) {
    const_goal_base = radB;
    const_goal_shoulder = radS;
    const_goal_elbow = radE;
    const_goal_hand = radG;

    goalX = lastX;
    goalY = lastY;
    goalZ = lastZ;
    goalT = lastT;
    return;
  }

  // Limiter le deplacement angulaire de la base entre -180° et 180°
  if (const_cmd_base_x == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_base += const_spd;
      if (const_goal_base > M_PI) {
        const_goal_base = M_PI;
        const_cmd_base_x = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalX += const_spd;
    }
  } else if (const_cmd_base_x == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_base -= const_spd;
      if (const_goal_base < -M_PI) {
        const_goal_base = -M_PI;
        const_cmd_base_x = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalX -= const_spd;
    }
  }

  // Limiter le deplacement angulaire de l'epaule entre -90° et 90°
  if (const_cmd_shoulder_y == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_shoulder += const_spd;
      if (const_goal_shoulder > M_PI/2) {
        const_goal_shoulder = M_PI/2;
        const_cmd_shoulder_y = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalY += const_spd;
    }
  } else if (const_cmd_shoulder_y == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_shoulder -= const_spd;
      if (const_goal_shoulder < -M_PI/2) {
        const_goal_shoulder = -M_PI/2;
        const_cmd_shoulder_y = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalY -= const_spd;
    }
  }

  // Limiter le deplacement angulaire du coude entre -45° et 180°
  if (const_cmd_elbow_z == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_elbow += const_spd;
      if (const_goal_elbow > M_PI) {
        const_goal_elbow = M_PI;
        const_cmd_elbow_z = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalZ += const_spd;
    }
  } else if (const_cmd_elbow_z == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_elbow -= const_spd;
      if (const_goal_elbow < -M_PI/4) {
        const_goal_elbow = -M_PI/4;
        const_cmd_elbow_z = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalZ -= const_spd;
    }
  }

  // Limiter le deplacement angulaire de la main entre -45° et 315°
  if (const_cmd_hand_t == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_hand += const_spd;
      if (const_goal_hand > M_PI*7/4) {
        const_goal_hand = M_PI*7/4;
        const_cmd_hand_t = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalT += const_spd/200;
    }
  } else if (const_cmd_hand_t == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_hand -= const_spd;
      if (const_goal_hand < -M_PI/4) {
        const_goal_hand = -M_PI/4;
        const_cmd_hand_t = MOVE_STOP;
      }
    } else if (const_mode == CONST_XYZT) {
      goalT -= const_spd/200;
    }
  }

  if (const_mode == CONST_ANGLE) {
    RoArmM2_allJointAbsCtrl(const_goal_base, const_goal_shoulder, 
                            const_goal_elbow, const_goal_hand, 0, 0);
  } else if (const_mode == CONST_XYZT) {

    static float bufferLastX;
    static float bufferLastY;
    static float bufferLastZ;
    static float bufferLastT;

    RoArmM2_baseCoordinateCtrl(goalX, goalY, goalZ, goalT);
    if (nanIK) {
      goalX = bufferLastX;
      goalY = bufferLastY;
      goalZ = bufferLastZ;
      goalT = bufferLastT;
  
      RoArmM2_baseCoordinateCtrl(bufferLastX, bufferLastY, bufferLastZ, bufferLastT);
      RoArmM2_goalPosMove();
      RoArmM2_lastPosUpdate();
      return;

    } else {
      bufferLastX = goalX;
      bufferLastY = goalY;
      bufferLastZ = goalZ;
      bufferLastT = goalT;
    }

    RoArmM2_goalPosMove();
    RoArmM2_lastPosUpdate();
  }
}
