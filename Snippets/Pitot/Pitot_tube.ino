
#include "Arduino.h"

const int pin_pitot = A0;
float diff_pressure;
double raw_pitot;
void setup()
{
Serial.begin(9600);
}

void loop()
{

raw_pitot = analogRead (pin_pitot);
if (raw_pitot < 102) {
  diff_pressure = -2.0;
} else {
  if (raw_pitot > 921) {
    diff_pressure = 2.0;
  } else {
    diff_pressure = map(raw_pitot, 102, 921, -2000, 2000)-119;
  }
}
// Some magical calcuation for the windspeed (just as an example)
float windspeed = sqrt ( (2.0 * diff_pressure) / 1.2 );

Serial.println(windspeed);
delay(100);
}
