#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <math.h>
/*
 This sample code demonstrates how the SimpleKalmanFilter object can be used with a
 pressure sensor to smooth a set of altitude measurements.
 This example needs a BMP180 barometric sensor as a data source.
 https://www.sparkfun.com/products/11824

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty
 e_est: Estimation Uncertainty
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1.5, 1, 0.4);

Adafruit_BMP085 bmp;

// Serial output refresh time






void setup() {

  Serial.begin(115200);

  // BMP180 Pressure sensor start
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}


}}

void loop() {

  float altitude = bmp.readAltitude(101500); //insert local seal level preasure
  float temperature = bmp.readTemperature();
  double kalman_altitude = pressureKalmanFilter.updateEstimate(altitude);

}
