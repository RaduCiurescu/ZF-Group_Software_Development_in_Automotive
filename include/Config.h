#ifndef __CONFIG_H__
#define __CONFIG_H__

/**************************************************************************************************
*
*   INCLUDES
*
**************************************************************************************************/

#include "direction.h"
#include "sonar.h"
#include <Arduino.h>
/**************************************************************************************************
*
*   VARIABLE DECLARATIONS
*
**************************************************************************************************/

extern bool showMenu = 0;
static int option;
extern bool bluetooth_flag = 0;
String BLE_val;

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Print_Menu

DESCRIPTION:
    Print the menu in terminal for being able to select the functionality for the car.

**************************************************************************************************/
void Print_Menu()
{
  Serial.println("Enter command:");
  Serial.println("0 - Show Menu");
  Serial.println("1 - Move Forward");
  Serial.println("2 - Move Backward");
  Serial.println("3 - Stop");
  Serial.println("4 - Rotate Left");
  Serial.println("5 - Rotate Right");
  Serial.println("6 - cycle");
  Serial.println("7 - changeSpeed");
  Serial.println("8 - masoara");
  Serial.println("9 - ACC");
  Serial.println("10 - AEB");


  showMenu = 0;
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Print_Menu

DESCRIPTION:
    Select the menu in terminal for being able to select the functionality for the car.

**************************************************************************************************/
void Select_Menu()
{ 
  if(Serial.available() > 0)
  {
    char input = Serial.read();  // Read one character
    switch (input) {
      case '0':
        option = 0;
        showMenu = 1;
      break;

      case '1':
       option =1;
       showMenu=0;
       Move_Forward(255);
       break;
      case '2':
       option =2;
       showMenu=0;
       Move_Backward(255);
      break;
      case '3':
       option =3;
       showMenu=0;
      break;
      case '4':
       option =4;
       showMenu=0;
      break;
      case '5':
       option =5;
       showMenu=0;
      break;
      case '6':
      option =6;
      showMenu=0;
      break;
      case '7':
      option =7;
      showMenu=0;
      break;
      case '8':
      option =8;
      showMenu=0;
      break;
      case '9':
      option =9;
      showMenu=0;
      break;
      case '10':
      option =10;
      showMenu=0;
      break;

      default:
        Serial.println("Unknown command!");
      break;
    }
  }

  if(option == 0)
  {
    if(showMenu == 1)
    {
      Stop();
      Print_Menu();
    }
  }
  if(option == 1)
  {
    if(showMenu == 0)
    {
      //Move_Forward(255);
      changeSpeed(250,FORWARD);
    }
  } if(option ==2)
  {
    if(showMenu == 0)
    {
      Move_Backward(255);
    }
  } 
  if(option == 3)
  {
    if(showMenu == 0)
    {
      Stop();
    }
  }
  if(option==4)
  {
    if(showMenu == 0)
    {
      Rotate_Left(255);
    }
  }
  if(option==5)
  {
    if(showMenu == 0)
    {
      Rotate_Right(255);
    }
  }
  if(option ==6)
  {
  if(showMenu == 0)
    {
      setPwm(220,FORWARD);
      delay(1000);
      setPwm(400,BACKWARD);
      delay(1000);
      setPwm(300,LEFT);
      delay(1000);
      setPwm(220,RIGHT);
      delay(1000);
      
    }
  }
  if(option ==7)
  { 
   if(showMenu==0)
   {
    
      changeSpeed(250,FORWARD);

   }
  }
  if(option ==8)
  { 
   if(showMenu==0)
   {
    
     Serial.println(getFrontObstacleDistance_cm());

   }
  }
  if(option ==10)
  { 
   if(showMenu==0)
   {
    //changeSpeed(250,FORWARD);
    Serial.println(getFrontObstacleDistance_cm());
    autonomousEmergencyBrake();

   }
  }if(option == 9)
  { 
   if(showMenu==0)
   {
    Serial.println("intrat");
    adaptive_cruise_control(100,255,10,40);

   }
  }

}
  



#endif