#ifndef ROARMM2_CONFIG_H_
#define ROARMM2_CONFIG_H_

#include <math.h>
#include <stdint.h>
#include <Arduino.h>

// Broches UART contorlant par BUS les servos-moteurs
#define S_RXD 18
#define S_TXD 19
#define FREQ_UART 1000000

// Configuration du BUS de servomoteurs
#define ST_PID_P_ADDR 21
#define ST_PID_I_ADDR 23

#define BASE_JOINT 1 // Base du bras robotique
#define SHOULDER_JOINT 2 // "Epaule" du bras robotique
#define ELBOW_JOINT 3 // "Coude" du bras robotique
#define HAND_JOINT 4 // "Main" du bras robotique

#define BASE_SERVO_ID 11 // Servo-moteur 1 : Base
#define SHOULDER_A_SERVO_ID 12 // Servo-moteur 2 : "Epaule" A
#define SHOULDER_B_SERVO_ID 13 // Servo-moteur 3 : "Epaule" B
#define ELBOW_SERVO_ID 14 // Servo-moteur 4 : "Coude"
#define HAND_SERVO_ID 15 // Servo-moteur 5 : "Main"

#define ARM_SERVO_MIDDLE_POS 2047 // Position mediane d'un servomoteur
#define ARM_SERVO_POS_RANGE 4096 // Position maximale d'un servomoteur
#define ARM_SERVO_INIT_ACC 20

#define ARM_L2_LENGTH_MM_A 236.82 // Articulation de "l'epaule" A au "coude"
#define ARM_L2_LENGTH_MM_B 30.00 // Articulation de "l'epaule" B au "coude"
#define ARM_L3_LENGTH_MM_A_0 280.15 // Articulation du "coude" a la main
#define ARM_L3_LENGTH_MM_B_0 1.73 // Articulation du "coude" a la main

#define BASE_JOINT_ANG 0
#define SHOULDER_JOINT_ANG 0
#define ELBOW_JOINT_ANG 90.0
#define HAND_JOINT_ANG 180.0

#define MOVE_STOP 0
#define MOVE_INCREASE 1
#define MOVE_DECREASE 2

#define CONST_ANGLE 0
#define CONST_XYZT 1

void RoArmM2_servoInit();
void RoArmM2_initCheck(bool);
void RoArmM2_resetPID();

void RoArmM2_getPosByServoFeedback(void);
void RoArmM2_allJointAbsCtrl(double, double, double, double, uint16_t, uint8_t);
void RoArmM2_setJointPID(byte, float, float);
void setNewAxisX(double);
void constantCtrl(byte, byte, byte, byte);
void setMiddlePos(byte);
void constantHandle(void);
bool setServosPID(byte, byte);
String RoArmM2_infoFeedback(void);

#endif