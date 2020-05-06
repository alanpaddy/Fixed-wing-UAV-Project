#include <Wire.h>
#include <MPU6050.h>

float ax, ay, az;
MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
mpu.setThreshold(30);
}

void loop()
{
  Vector normAccel = mpu.readNormalizeAccel();

  ax = normAccel.XAxis-8.9;
  ay = normAccel.YAxis-0.4;
  az = normAccel.ZAxis+5.9;

  Serial.print(" Xnorm = ");
  Serial.print(ax);
  Serial.print(" Ynorm = ");
  Serial.print(ay);
  Serial.print(" Znorm = ");
  Serial.println(az);

  delay(10);
}
