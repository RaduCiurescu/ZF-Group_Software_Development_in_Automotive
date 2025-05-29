#include <Config.h>
void setup()
{
  Serial.begin(9600);
  BLE_val = "";

  setupSonar();
  setupMotors();
  setupRobotArm();
  Print_Menu();
}

void loop()
{
   //Select_Menu();
  // changeSpeed(80,FORWARD);
  // autonomousEmergencyBrake();
   //adaptive_cruise_control(40,150,20,50);
   //LineTrackingFunction();
  // moveObject();
  moveObject();
}