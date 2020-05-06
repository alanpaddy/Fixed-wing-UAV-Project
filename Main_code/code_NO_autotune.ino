#include <PID_v1.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include "six_axis_comp_filter.h"

#include <math.h>
#include <Wire.h>
#include <Arduino.h>
#include "SPI.h"
#include <Servo.h>

#define pin_elevator 4
#define pin_ailerons 3
#define pin_relay 2
#define pin_AP 5
#define pin_bat_1 A7
#define pin_bat_2 A6
#define pin_pitot A11

//PID vars
boolean AP = false; //set to true to activate all PID stuff

double input_elevator, output_elevator, setpoint_elevator=0;
double kp_elevator=6,ki_elevator=2,kd_elevator=0.5;
double output_elevatorStart=5;
float output_elevator_maped;

double input_ailerons, output_ailerons, setpoint_ailerons=0;
double kp_ailerons=6, ki_ailerons=2, kd_ailerons = 0.5;
double output_aileronsStart =5;
float output_ailerons_maped;

//MPU vars
float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle,       // variables for angles calculated by filter
      accelAngleX, accelAngleY, //accelerometer angle
      gyroAngleX, gyroAngleY, //gyroscope angle
      pitch, roll,
      temperature_MPU;  //MPU also has T sensor

//BMP085 vars
float altitude, temperature_BMP, rho, vertical_speed, altitude_filtered_2, pressure;
float altitude_filtered;

//Pitot vars
float diff_pressure;
float raw_pitot, speed_pitot;

//derivative values
float time_1, time_2, dt;

//battery vars
int raw_bat_1, raw_bat_2;

//PID stuff
PID PIDelevator(&input_elevator, &output_elevator, &setpoint_elevator,kp_elevator,ki_elevator,kd_elevator, DIRECT);
PID PIDailerons(&input_ailerons, &output_ailerons, &setpoint_ailerons,kp_ailerons,ki_ailerons,kd_ailerons, DIRECT);

MPU6050 mpu;
Adafruit_BMP085 bmp;

CompSixAxis CompFilter(0.1, 2); //For MPU (gyro+accel)
SimpleKalmanFilter pressureKalmanFilter(.7, .1, 0.1);// for BMP085 pressure
SimpleKalmanFilter dtKalmanFilter(0.5, 0.1, 0.1);// for dt BMP085 vertical velocity

//control stuff
Servo elevator;
Servo ailerons;

void setup()
{
  pinMode(pin_relay, OUTPUT); //switch relay on/off
  pinMode(pin_AP, INPUT); //signal into arduino

  elevator.attach(pin_elevator); //attach elevators
  ailerons.attach(pin_ailerons); //attach ailerons

  mpu.begin(); //Start MPU6050
  bmp.begin(); //Start BMP085

  //BMP085 Pressure sensor start
  if (!bmp.begin()) {
   Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1); }
  // MPU605' sensor start
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);}

  //PID stuff
  PIDelevator.SetMode(AUTOMATIC); //Allows the controller to be set to manual (0) or Automatic (non-zero)
  PIDailerons.SetMode(AUTOMATIC);

Serial.begin(57600);
}



void loop(){

 //MPU stuff
 Vector normAccel = mpu.readNormalizeAccel();
 Vector normGyro = mpu.readNormalizeGyro();

//skidipi papa
 accelX = normAccel.XAxis;
 accelY = normAccel.YAxis;
 accelZ = normAccel.ZAxis;
 gyroX = normGyro.XAxis;
 gyroY = normGyro.YAxis;
 gyroZ = normGyro.ZAxis;
 temperature_MPU = mpu.readTemperature();

 //Complementary Filter
 CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
 CompFilter.CompUpdate();
 CompFilter.CompStart();

 // Get angle relative to X and Y axes and write them to the variables in the arguments
 CompFilter.CompAnglesGet(&xAngle, &yAngle);

 roll = xAngle*57.2957795f;
 pitch = yAngle*57.2957795f;

 if (roll >180) { //+Ve left banking, -Ve right banking
   roll = roll-360;}
 if (roll <180){
  roll =- roll;}
 if (pitch>180){ //+ve upwards, -ve downwards
   pitch = -(360-pitch);}
 if (pitch<180){
   pitch= pitch;}

  float time_1=millis(); //start timing

  /* BMP */
  altitude = bmp.readAltitude(98510); //insert local seal level preasure
  temperature_BMP = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude_filtered = pressureKalmanFilter.updateEstimate(altitude);
  rho = pressure/(286.9*temperature_BMP);

  float time_2 = millis(); //stop timing

  float dt = (time_2-time_1)/1000; //count time difference

  altitude_filtered_2=pressureKalmanFilter.updateEstimate(altitude);
  vertical_speed = dtKalmanFilter.updateEstimate((altitude_filtered_2-altitude_filtered)/dt);



  /* Pitot tube */
  raw_pitot = analogRead(pin_pitot);
  if (raw_pitot < 120){
    diff_pressure = -2.0;}
  else{
    if (raw_pitot >921){
      diff_pressure = 2;}
      else {
      diff_pressure=map(raw_pitot, 102, 921, -2000, 2000)-119;}
  }
  speed_pitot = sqrt((2*diff_pressure)/rho);

  /* Battery monitoring */
  raw_bat_1 = analogRead(pin_bat_1);
  raw_bat_2 = analogRead(pin_bat_2);


   //AP on-off thing
  float AP_pulse = pulseIn(pin_AP, HIGH);
  float AP_pulse_1_0 = map(AP_pulse, 900, 1900, 0, 1); //Scale input, (original signal 983 , 20000)
  if ( AP_pulse_1_0 ==1){
    digitalWrite(pin_relay, HIGH); AP=1;}
  else {
    digitalWrite(pin_relay, LOW); AP=0;}

//Autopilot stuff
/*-----------------------------------------------------------*/
if(AP == 1){
    input_elevator = pitch;
    input_ailerons = roll;

    PIDelevator.Compute();
    PIDailerons.Compute();

    output_elevator_maped = map(output_elevator, -255, 255, 45, 135 );
    output_ailerons_maped = map(output_ailerons, -255, 255, 45, 135 );
    elevator.write(output_elevator_maped);
    ailerons.write(output_ailerons_maped);
    }

/*
//User friendly

  Serial.print(speed_pitot);
  Serial.print(" [m/s]");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" [deg]");
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" [deg]");
  Serial.print("Altitude");
  Serial.println(altitude_filtered);
  Serial.print("[m]");
  Serial.print(vertical_speed);
  Serial.print("[m/s]");
*/
//raw

Serial.println("----------------")
  Serial.print("Pitot: ")
  Serial.print(speed_pitot);
  Serial.print("   Vertsp: ");
  Serial.print(vertical_speed);
  Serial.print("   Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print("   Alt: ");
  Serial.println(altitude_filtered);
  Serial.print("    Bat1: ");
  Serial.print(raw_bat_1);
  Serial.print(" Bat2: ");
  Serial.print(raw_bat_2);
  Serial.print("   AP: ");
  Serial.print(AP);
  Serial.print("   Tb: ");
  Serial.print(temperature_BMP);
  Serial.print(" Tm: ");
  Serial.print(temperature_MPU);
  Serial.print("   ax: ");
  Serial.print(accelX );
  Serial.print(" ay: ");
  Serial.print(accelY );
  Serial.print(" az: ");
  Serial.print(accelZ);
  Serial.print("   gx: ");
  Serial.print(gyroX);
  Serial.print(" gy: ");
  Serial.print(gyroY);
  Serial.print(" gz: ");
  Serial.print(gyroZ);
  Serial.print("   pamb: ");
  Serial.print(pressure);
  Serial.print("   Rho: ");
  Serial.print(rho);

  Serial.print("  ");


  delay(10);
}
