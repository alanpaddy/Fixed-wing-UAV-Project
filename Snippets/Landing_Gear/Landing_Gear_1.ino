#include "Arduino.h"
#include <Servo.h>
Servo Servo_Wheel_1; //Landing gear 1
Servo Servo_Wheel_2; //Landing gear 2

const int Pin_Servo_Wheel_1 = 3 ; //LDG 1 pin
const int Pin_Servo_Wheel_2 = 23; //LDG 2 pin

void setup()
{


  //Serial.begin(9600);Servo_wheel_1.attach(Pin_Servo_Wheel_1);  // attaches the servo on pin 9 to the servo object
Servo_Wheel_1.attach(Pin_Servo_Wheel_1);
Servo_Wheel_2.attach(Pin_Servo_Wheel_2);
Servo_Wheel_1.writeMicroseconds(1500);
Servo_Wheel_2.writeMicroseconds(1500);
}

void loop()
{
  Servo_Wheel_1.writeMicroseconds(1000);
  Servo_Wheel_2.writeMicroseconds(1000);
  delay(10000);

  //Servo_wheel_1.writeMicroseconds(2000);
//  delay(10000);
}
