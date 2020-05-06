// This gist uses this sensor https://learn.sparkfun.com/tutorials/lsm6ds3-breakout-hookup-guide
// with this filter https://github.com/tcleg/Six_Axis_Complementary_Filter
// to give you angles according to the X and Y axes

/******************************************************************************
Filter Arduino hookup guide by Vlady Veselinov,
******************************************************************************/
#include "Arduino.h"
#include <MPU6050.h>
#include "Wire.h"
#include "SPI.h"
#include "six_axis_comp_filter.h"

MPU6050 mpu; // Default constructor for the motion sensor


CompSixAxis CompFilter(0.1, 2);

float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle,       // variables for angles calculated by filter
      accelAngleX, accelAngleY, //accelerometer angle
      gyroAngleX, gyroAngleY, //gyroscope angle
      pitch, roll;
void setup() {
  Serial.begin(9600);

  mpu.begin(); //to configure the IMU (Inertial Measurement Unit)
 // mpu.calibrateGyro(); //Carlibrate Gyro, must be at rest
 // mpu.setThreshold(1); //Threshold for Gyro

}


void loop() {


  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  accelX = normAccel.XAxis;
  accelY = normAccel.YAxis;
  accelZ = normAccel.ZAxis;

  gyroX = normGyro.XAxis;
  gyroY = normGyro.YAxis;
  gyroZ = normGyro.ZAxis;

  // Convert these values into angles using the Complementary Filter
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  // Get angle relative to X and Y axes and write them to the variables in the arguments
  CompFilter.CompAnglesGet(&xAngle, &yAngle);

  roll = xAngle*57.2957795f;
  pitch = yAngle*57.2957795f;

  if (roll >180) { //-Ve left banking, +Ve right banking
    roll = roll-360;
  }

  if (pitch>180){ //+ve upwards, -ve downwards
    pitch = pitch-360;
  }
  if (pitch<180){
    pitch= -pitch;
  }







  Serial.println(pitch);
  Serial.print(" ");
  //Serial.println(yAngle*57.2957795f);
  //Serial.println("No pomegranates");

}
