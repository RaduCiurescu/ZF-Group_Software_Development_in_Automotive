#ifndef __DIROECTION_H__
#define __DIRECTION_H__

/**************************************************************************************************
*
*   INCLUDES
*
**************************************************************************************************/
#include <Arduino.h>
/**************************************************************************************************
*
*   DEFINES
*
**************************************************************************************************/

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define AEB_THRESHOLD 15
#define S_Stanga 7
#define S_Centru 8
#define S_Dreapta A1

/**************************************************************************************************
*
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

static int DC_PWM_Value;
static float distance;
static int previousLeft = 0;
static int previousRight = 0;
static int base_position = 90;
static int claw_position = 0;
static int arm_position = 0;
//static float last_distance;
//static float last_speed;
//bool keep_speed=false;
/**************************************************************************************************
*
*   FUNCTION DECLARATIONS
*
**************************************************************************************************/

void    setupMotors();
void    setupLineTracking();
void    Move_Forward(int speed);
void    Move_Backward(int speed);
void    Stop();
void    Rotate_Left(int speed);
void    Rotate_Right(int speed);
void    autonomousEmergencyBrake();
void    setPwm(uint8_t, uint8_t);
int     getPwm();
void    adaptive_cruise_control(uint8_t, uint8_t, uint8_t, uint16_t);
int     checkPWM(uint8_t, int, int);
void    changeSpeed(uint8_t, uint8_t);
void    LineTrackingFunction();
void    setupRobotArm();
void    moveObject();
void    resetRobotArm();
#endif