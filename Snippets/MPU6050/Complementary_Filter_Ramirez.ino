#include <Wire.h>
#include <MPU6050.h>

float ax, ay, az; //Accelerometer values
float wx, wy, wz; //Gyroscope values
float time_1, time_2, dt; //Time values
float pitch_gyro=0, roll_gyro=0, yaw_gyro=0;
float pitch_accel, roll_accel;
float pitch, roll; //Complementary filter

MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Co<uld not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro(); //Carlibrate Gyro, must be at rest
  mpu.setThreshold(1); //Threshold for Gyro

  dt = 0.01;
  time_1 = 0;

}


void loop()
{
  time_1 = millis(); //start counting time

  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  //Accelerometer values
  ax = normAccel.XAxis;
  ay = normAccel.YAxis;
  az = normAccel.ZAxis;
  //Gyro values
  wx = normGyro.XAxis;
  wy = normGyro.YAxis;
  wz = normGyro.ZAxis;

  //Gyroscope Attitude
  pitch_gyro += -wy*dt; //+Ve is upwards
  roll_gyro += wx*dt; //+Ve is to the right
  yaw_gyro += -wz*dt; //+Ve is to the right


  //Accelerometer Attitude
  pitch_accel = atan2(ax, sqrt(ay*ay + az*az))*180/M_PI;
  roll_accel = atan2(ay, az)*180/M_PI;


  //Complementary Filter
  pitch = 0.98*(pitch + pitch_gyro*dt) + 0.02*pitch_accel; //90deg = 2.14
  roll = 0.98*(roll + roll_gyro*dt) + 0.02*roll_accel;

  time_2 = millis(); //stop counting time



/*
  Serial.print(" ax = ");
  Serial.print(ax);
  Serial.print(" ay= ");
  Serial.print(ay);
  Serial.print(" az = ");
  Serial.println(az);

  Serial.print(" wx = ");
  Serial.print(wx);
  Serial.print(" wy= ");
  Serial.print(wy);
  Serial.print(" wz = ");
  Serial.println(wz);
*/
//  Serial.print(" pitch = ");
  //Serial.print(pitch_gyro);
//  Serial.print(" roll= ");
  Serial.print(roll_accel);
  Serial.print(",");
  Serial.print(roll_gyro);
  Serial.print(",");
  Serial.println(roll);
  delay((dt*1000 - (time_2-time_1)));
}
