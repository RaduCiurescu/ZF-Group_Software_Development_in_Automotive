
#include <Arduino.h>
#include <Servo.h>
#include "direction.h"
#include "sonar.h"

Servo base_servo, arm_servo,claw_servo;

int before=0;
/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setupMotors

DESCRIPTION:
    Setup the motors and an default PWM.

**************************************************************************************************/
void setupMotors()
{
    
  pinMode(2, OUTPUT); // left front/rear motor (direction)(port A)
  pinMode(5, OUTPUT); // PWM for port A
  pinMode(4, OUTPUT); // right front/rear motor (direction)(port B)
  pinMode(6, OUTPUT); // PWM for port B

  //viteza default
  DC_PWM_Value = 0;
}
/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setups the line sensors

DESCRIPTION:
    sets the pins

**************************************************************************************************/


void setupLineTracking()
{
  pinMode(7, INPUT); // left line tracking sensor
  pinMode(8, INPUT); // center line tracking sensor
  pinMode(A1, INPUT); //right line tracking sensor
}
/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Move_Forward

DESCRIPTION:
    Move the car forward with a desired speed.

**************************************************************************************************/
void Move_Forward(int speed) 
{
    
speed= checkPWM(speed, 0, 255); // Check if the speed is within the limits
digitalWrite(2, HIGH); // Set the direction for port A (forward)
digitalWrite(4,LOW);
analogWrite(5,speed);
analogWrite(6,speed);

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Move_Backward

DESCRIPTION:
    Move the car backward with a desired speed.

**************************************************************************************************/
void Move_Backward(int speed) 
{
    speed= checkPWM(speed, 0, 255); // Check if the speed is within the limits
    digitalWrite(2, LOW); // Set the direction for port A (forward)
    digitalWrite(4,HIGH);
    analogWrite(5,speed);
    analogWrite(6,speed);

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Stop

DESCRIPTION:
    Stop the motors from running.

**************************************************************************************************/
void Stop() 
{

    analogWrite(5,0);
    analogWrite(6,0);
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Rotate_Left

DESCRIPTION:
    Rotate the car to the left with a desired speed.

**************************************************************************************************/
void Rotate_Left(int speed) 
{
    
    speed= checkPWM(speed, 0, 255);
    digitalWrite(2,LOW);
    digitalWrite(4,LOW);
    analogWrite(5,speed);
    analogWrite(6,speed);

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    Rotate_Right

DESCRIPTION:
    Rotate the car to the right with a desired speed.

**************************************************************************************************/
void Rotate_Right(int speed) 
{
    speed= checkPWM(speed, 0, 255);
    digitalWrite(2,HIGH);
    digitalWrite(4,HIGH);
    analogWrite(5,speed);
    analogWrite(6,speed);
}


/**************************************************************************************************
                              FUNCTION INFO
NAME:
    autonomousEmergencyBrake

DESCRIPTION:
    Handle the AEB functionality(stop at obstacle).

**************************************************************************************************/
void autonomousEmergencyBrake()
{
    if(getFrontObstacleDistance_cm()< AEB_THRESHOLD)
    {
        Stop();
        DC_PWM_Value = 0;
    }

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setPwm

DESCRIPTION:
    Set the PWM(speed) and the direction of the car(FORWARD/BACKWARD/LEFT/RIGHT)

**************************************************************************************************/
void setPwm(uint8_t dcspeed, uint8_t mode)
{
    
 DC_PWM_Value=checkPWM(dcspeed,0,255);
if(mode == FORWARD)
{
    Move_Forward(DC_PWM_Value);
}
else if(mode == BACKWARD)
{
    Move_Backward(DC_PWM_Value);
}
else if(mode == LEFT)
{
    Rotate_Left(DC_PWM_Value);
}
else if(mode == RIGHT)
{
    Rotate_Right(DC_PWM_Value);
}
else
{
    Serial.println("Invalid mode selected!");
}

}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    getPwm

DESCRIPTION:
    Get the current PWM.

**************************************************************************************************/
int getPwm()
{
    return DC_PWM_Value;
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    changeSpeed

DESCRIPTION:
    Change the speed.
    Parameters:
    dcspeed - desired speed 
    mode    - direction of the car (FORWARD/BACKWARD/LEFT/RIGHT)

**************************************************************************************************/
void changeSpeed(uint8_t targetSpeed, uint8_t mode)
{
    
    int now= millis();
    //Serial.println(now);
    //Serial.println(before);

    if(now-before>= 10)
    {
        if(targetSpeed>DC_PWM_Value)
        {
        setPwm(++DC_PWM_Value,mode);
        }
        else if(targetSpeed< DC_PWM_Value)
        {
            setPwm(--DC_PWM_Value,mode);
        }
        before= millis();
    }
}
/**************************************************************************************************
                              FUNCTION INFO
NAME:
    adaptive_cruise_control

DESCRIPTION:
    Controls the car's speed to maintain a safe distance from the obstacle in front (ACC - Adaptive Cruise Control).
    - If the obstacle is too close (obstacle <'min_distance'), AEB is activated.
    - If the obstacle is very far away (obstacle >'max_distance'), the maximum speed is used.
    - If the obstacle is somewhere between 'min_distance' and 'max_distance', the speed is calculated proportionally.


    adaptive_cruise_control(150,255,10,30);
        105
    




**************************************************************************************************/
void adaptive_cruise_control(uint8_t min_speed, uint8_t max_speed, uint8_t min_distance, uint16_t max_distance) 
{
    float changed_speed;
    float change;
    float distance =getFrontObstacleDistance_cm();
    if(distance <min_distance)
       { 
        autonomousEmergencyBrake();
        Serial.println("brake");
         }
    else if(distance > max_distance)
        {
            changeSpeed(max_speed,FORWARD);

            Serial.println(max_speed);
        }
   else
   {
        /*if(last_distance<distance){ 
        keep=true;                           
         }
        if(keep==true)
        {
            changeSpeed(last,FORWARD);
            return;
        }
        */
        change= (distance-min_distance)/(max_distance-min_distance);
        changed_speed=min_speed + change* (max_speed-min_speed);
        if(changed_speed<min_speed)
            changeSpeed(min_speed,FORWARD);
            else if (changed_speed >max_speed)
            changeSpeed(max_speed,FORWARD);
            else
            changeSpeed((uint8_t)changed_speed,FORWARD);
        /*if(keep==false)
            last_distance=distance;
        */
   }
}   

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    checkPWM

DESCRIPTION:
    Verify if the PWM it's within threshold.

**************************************************************************************************/
int checkPWM(uint8_t number, int lower_limit, int upper_limit)
{
    if(number<lower_limit )
    {
        number = lower_limit;
        return lower_limit;
    }
    else if(number>upper_limit)
    {
       number=upper_limit;
       return upper_limit;
    }
    return number;

}
/**************************************************************************************************
                              FUNCTION INFO
NAME:
    LineTrackingFunction

DESCRIPTION:
    Follows the line with the car.
**************************************************************************************************/

void LineTrackingFunction()
{
    int leftSensor = digitalRead(7);
    int centerSensor = digitalRead(8);
    int rightSensor = digitalRead(A1);

    // Serial.print("Left:");
    // Serial.println(leftSensor);
    // Serial.print("Center:");
    // Serial.println(centerSensor);
    // Serial.print("Right:");
    // Serial.println(rightSensor);

    if(centerSensor == LOW && leftSensor == LOW && rightSensor == LOW)
    {
        if(previousLeft)
            Rotate_Left(60);
        else if(previousRight)
            Rotate_Right(60);
        else
            Rotate_Right(60);
    }
    else
    {
    if(centerSensor == LOW && leftSensor == HIGH && rightSensor == LOW) //left 
        Rotate_Left(60);

    if(centerSensor == HIGH && leftSensor == HIGH && rightSensor == LOW) //left center
        Rotate_Left(60);

    if(centerSensor == HIGH && leftSensor == LOW && rightSensor == HIGH) //right center
        Rotate_Right(60);

    if(centerSensor == LOW && leftSensor == LOW && rightSensor == HIGH) //right
        Rotate_Right(60);

    if(centerSensor == HIGH) //intersectie/ default
    {
        Move_Forward(70);
        previousLeft = 0;
        previousRight = 0;
    }

    if(leftSensor == HIGH)
        previousLeft = leftSensor;

    if(rightSensor == HIGH)
        previousRight = rightSensor;
    }
    
}
    

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    setupRobotArm

DESCRIPTION:
   setup robot smotors
**************************************************************************************************/


void setupRobotArm()
{
  #if CAR1_PIXY == 1
    base_servo.attach(9);
  #endif

  #if CAR2_ARM == 1
    base_servo.attach(10);
    arm_servo.attach(9);
    claw_servo.attach(11);

    base_servo.write(90);
    claw_servo.write(0);
    arm_servo.write(0);
  #endif



  
}


/**************************************************************************************************
                              FUNCTION INFO
NAME:
    moveObject

DESCRIPTION:
    Follows the line with the car.
**************************************************************************************************/

void moveObject()
{

    for(int i=0; i<=90;i++) //apleaca bratu
    {
        arm_servo.write(i);
        claw_position=i;
        delay(20);
    }
    for(int i=0; i<=100;i++) //inchide claw
    {
        claw_servo.write(i);
        claw_position=i;
        delay(20);
    }
    for(int i=90; i>0;i--)  //ridica bratu
    {
        arm_position=i;
        arm_servo.write(i);
        delay(20);
    }
    for(int i=90; i<=180;i++) //roteste baseu
    {
        base_position=1;
        base_servo.write(i);
        delay(20);
    }
    for(int i=0; i<=90;i++) //apleaca bratu
    {
        arm_position=i;
        arm_servo.write(i);
        delay(20);
    }
    for(int i=100; i>0;i--) //inchide claw
    {
        claw_position=i;
        claw_servo.write(i);
        delay(20);
    }
    for(int i=90; i>0;i--)  //ridica bratu
    {
        arm_position=i;
        arm_servo.write(i);
        delay(20);
    }
    for(int i=180; i>=90;i--) //roteste baseu
    {
        base_position=1;
        base_servo.write(i);
        delay(20);
    }
}

/**************************************************************************************************
                              FUNCTION INFO
NAME:
    resetRobotArm

DESCRIPTION:
    Resets the arm
**************************************************************************************************/
void resetRobotArm()
{
    for(int i=180; i>=90;i--) //roteste baseu
    {
        base_servo.write(i);
        delay(20);
    }
    claw_servo.write(0); // 0 deschis 100 inchis
    delay(1000);
    arm_servo.write(0);  /// 0 sus// 90 in fata
    delay(1000);
}