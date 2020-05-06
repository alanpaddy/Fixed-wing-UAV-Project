
#include <Wire.h>
#include <MPU6050.h>
float wx, wy, wz;
MPU6050 mpu;

void setup()
{
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);


}

void loop()
{
  Vector normGyro = mpu.readNormalizeGyro();

  wx = normGyro.XAxis;
  wy = normGyro.YAxis;
  wz = normGyro.ZAxis;

  Serial.print(" Xnorm = ");
  Serial.print(wx);
  Serial.print(" Ynorm = ");
  Serial.print(wy);
  Serial.print(" Znorm = ");
  Serial.println(wz);

  delay(100);
}
