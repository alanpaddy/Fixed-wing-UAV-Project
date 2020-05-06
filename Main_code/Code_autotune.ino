#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
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
#define pin_elevator_monitor A0
#define pin_ailerons_monitor A1
#define pin_relay 2
#define pin_AP 5
#define pin_bat_1 A7
#define pin_bat_2 A6
#define pin_pitot A11

//PID vars
boolean AP = false; //set to true to activate all PID stuff
boolean tuning_elevator = false;
boolean tuning_ailerons = false; //set to true to autotune PID

//Autopilot stuff
byte ATuneModeRemember=1;

double input_elevator, output_elevator, setpoint_elevator=0;
double kp_elevator=6,ki_elevator=2,kd_elevator=0.5;
double output_elevatorStart=5;
double aTuneStep_elevator=1, aTuneNoise_elevator=0.3, aTuneStartValue_elevator=10;
float output_elevator_maped;
unsigned int aTuneLookBack_elevator=200;

double input_ailerons, output_ailerons, setpoint_ailerons=0;
double kp_ailerons=6, ki_ailerons=2, kd_ailerons = 0.5;
double output_aileronsStart =5;
double aTuneStep_ailerons=1, aTuneNoise_ailerons=0.3, aTuneStartValue_ailerons=10;
float output_ailerons_maped;
unsigned int aTuneLookBack_ailerons=200;

//MPU variables
float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle,       // variables for angles calculated by filter
      accelAngleX, accelAngleY, //accelerometer angle
      gyroAngleX, gyroAngleY, //gyroscope angle
      pitch, roll,
      temperature_MPU;  //MPU also has T sensor

//BMP085 variables
float altitude, temperature_BMP, rho, vertical_speed, altitude_filtered_2, pressure;
double altitude_filtered;

//Pitot variables
float diff_pressure;
double raw_pitot, speed_pitot;

//derivative values
float time_1, time_2, dt;

//PID stuff
PID PIDelevator(&input_elevator, &output_elevator, &setpoint_elevator,kp_elevator,ki_elevator,kd_elevator, DIRECT);
PID PIDailerons(&input_ailerons, &output_ailerons, &setpoint_ailerons,kp_ailerons,ki_ailerons,kd_ailerons, DIRECT);
PID_ATune aTune_elevator(&input_elevator, &output_elevator);
PID_ATune aTune_ailerons(&input_ailerons, &output_ailerons);

MPU6050 mpu;
Adafruit_BMP085 bmp;

CompSixAxis CompFilter(0.1, 2); //For MPU (gyro+accel)
SimpleKalmanFilter pressureKalmanFilter(.7, .1, 0.1);// for BMP085 pressure
SimpleKalmanFilter dtKalmanFilter(0.5, 0.1, 0.1);// for dt BMP085 vertical velocity

//control stuff
Servo elevator;
Servo ailerons;

Servo elevator_monitor, ailerons_monitor;

//PID autotune functions
/*---------------------------------------------------------------------*/
void AutoTuneHelper_elevator(boolean start)
{
  if(start)
    ATuneModeRemember = PIDelevator.GetMode();
  else
    PIDelevator.SetMode(ATuneModeRemember);
}

void AutoTuneHelper_ailerons(boolean start)
{
  if(start)
    ATuneModeRemember = PIDailerons.GetMode();
  else
    PIDailerons.SetMode(ATuneModeRemember);
}

void changeAutoTune_elevator()
{
 if(!tuning_elevator)
  {
    //Set the output_elevator to the desired starting frequency.
    output_elevator=aTuneStartValue_elevator;
    aTune_elevator.SetNoiseBand(aTuneNoise_elevator);
    aTune_elevator.SetOutputStep(aTuneStep_elevator);
    aTune_elevator.SetLookbackSec((int)aTuneLookBack_elevator);
    AutoTuneHelper_elevator(true);
    tuning_elevator = true;
  }
  else
  { //cancel autotune
    aTune_elevator.Cancel();
    tuning_elevator = false;
    AutoTuneHelper_elevator(false);
  }
}

void changeAutoTune_ailerons()
{
 if(!tuning_ailerons)
  {
    //Set the output_elevator to the desired starting frequency.
    output_ailerons=aTuneStartValue_ailerons;
    aTune_ailerons.SetNoiseBand(aTuneNoise_ailerons);
    aTune_ailerons.SetOutputStep(aTuneStep_ailerons);
    aTune_ailerons.SetLookbackSec((int)aTuneLookBack_ailerons);
    AutoTuneHelper_ailerons(true);
    tuning_ailerons = true;
  }
  else
  { //cancel autotune
    aTune_ailerons.Cancel();
    tuning_ailerons = false;
    AutoTuneHelper_ailerons(false);
  }
}
void SerialSend_elevator()
{
  Serial.print("setpoint: ");Serial.print(setpoint_elevator); Serial.print(" ");
  Serial.print("input_elevator: ");Serial.print(input_elevator); Serial.print(" ");
  Serial.print("output_elevator: ");Serial.print(output_elevator); Serial.print(" ");
  if(tuning_elevator){
    Serial.println("tuning elevator");
  } else {
    Serial.print("kp elevator: ");Serial.print(PIDelevator.GetKp());Serial.print(" ");
    Serial.print("ki elevator: ");Serial.print(PIDelevator.GetKi());Serial.print(" ");
    Serial.print("kd elevator: ");Serial.print(PIDelevator.GetKd());Serial.println();
  }
}
void SerialSend_ailerons()
{
  Serial.print("setpoint: ");Serial.print(setpoint_ailerons); Serial.print(" ");
  Serial.print("input_ailerons: ");Serial.print(input_ailerons); Serial.print(" ");
  Serial.print("output_ailerons: ");Serial.print(output_ailerons); Serial.print(" ");
  if(tuning_ailerons){
    Serial.println("tuning ailerons");
  } else {
    Serial.print("kp ailerons: ");Serial.print(PIDailerons.GetKp());Serial.print(" ");
    Serial.print("ki ailerons: ");Serial.print(PIDailerons.GetKi());Serial.print(" ");
    Serial.print("kd ailerons: ");Serial.print(PIDailerons.GetKd());Serial.println();
  }
}
void SerialReceive_elevator()
{
  if(Serial.available())
  {
   char b = Serial.read();
   Serial.flush();
   if((b=='1' && !tuning_elevator) || (b!='1' && tuning_elevator))changeAutoTune_elevator();
  }
}

void SerialReceive_ailerons()
{
  if(Serial.available())
  {
   char b = Serial.read();
   Serial.flush();
   if((b=='1' && !tuning_ailerons) || (b!='1' && tuning_ailerons))changeAutoTune_ailerons();
  }
}
/*---------------------------------------------------------------------*/







void setup()
{
  pinMode(pin_relay, OUTPUT); //switch relay on/off
  pinMode(pin_AP, INPUT); //signal into arduino
  pinMode(pin_ailerons_monitor, INPUT);
  pinMode(pin_elevator_monitor, INPUT);

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
  if(tuning_elevator){
     tuning_elevator=false;
     changeAutoTune_elevator();
     tuning_elevator=true;}
  if(tuning_ailerons){
      tuning_ailerons=false;
      changeAutoTune_ailerons();
      tuning_ailerons=true;}

Serial.begin(57600);
}








void loop(){
  //AP on-off thing
  float AP_pulse = pulseIn(pin_AP, HIGH);
  float AP_pulse_1_0 = map(AP_pulse, 900, 1900, 0, 1); //Scale input, (original signal 983 , 20000)
  if ( AP_pulse_1_0 ==1){
    digitalWrite(pin_relay, HIGH); AP=true;}
  else {
    digitalWrite(pin_relay, LOW); AP=false;}

 //MPU stuff
 Vector normAccel = mpu.readNormalizeAccel();
 Vector normGyro = mpu.readNormalizeGyro();

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

 if (roll >180) { //-Ve left banking, +Ve right banking
   roll = roll-360;}
 if (pitch>180){ //+ve upwards, -ve downwards
   pitch = pitch-360;}
 if (pitch<180){
   pitch= -pitch;}


  float time_1=millis(); //start timing
  //BMP
  altitude = bmp.readAltitude(102100); //insert local seal level preasure
  temperature_BMP = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude_filtered = pressureKalmanFilter.updateEstimate(altitude);
  rho = pressure/(286.9*temperature_BMP);

  float time_2 = millis(); //stop timing

  float dt = (time_2-time_1)/1000;
  altitude_filtered_2=pressureKalmanFilter.updateEstimate(altitude);
  vertical_speed = dtKalmanFilter.updateEstimate((altitude_filtered_2-altitude_filtered)/dt);

  //Pitot tube
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



  //Battery monitoring
  float raw_bat_1 = analogRead(pin_bat_1);
  float raw_bat_2 = analogRead(pin_bat_2);
  float volt_bat_1 = raw_bat_1*5.5/1023*3.0545;
  float volt_bat_2 = raw_bat_2*5.5/1023*3.0545;
  float soc_bat_1 = (volt_bat_1-12.4)*100/4.4;
  float soc_bat_2 = (volt_bat_2-12.4)*100/4.4;

//Autopilot stuff
/*-----------------------------------------------------------*/
if(AP == 1){
    input_elevator = pitch;
    input_ailerons = roll;
    if(tuning_elevator)
    {
      byte val = (aTune_elevator.Runtime());
      if (val!=0)
      {
        tuning_elevator = false;
      }
      if(!tuning_elevator){ //we're done, set the tuning parameters
        kp_elevator = aTune_elevator.GetKp();
        ki_elevator = aTune_elevator.GetKi();
        kd_elevator = aTune_elevator.GetKd();
        PIDelevator.SetTunings(kp_elevator,ki_elevator,kd_elevator);
        AutoTuneHelper_elevator(false);}
    }
    else PIDelevator.Compute();

    if(tuning_ailerons){
      byte val = (aTune_ailerons.Runtime());
      if (val!=0){
        tuning_ailerons = false;}
      if(!tuning_ailerons){ //we're done, set the tuning parameters
        kp_ailerons = aTune_ailerons.GetKp();
        ki_ailerons = aTune_ailerons.GetKi();
        kd_ailerons = aTune_ailerons.GetKd();
        PIDailerons.SetTunings(kp_ailerons,ki_ailerons,kd_ailerons);
        AutoTuneHelper_ailerons(false);}
    }
    else PIDailerons.Compute();

    output_elevator_maped = map(output_elevator, -255, 255, 0, 180 );
    output_ailerons_maped = map(output_ailerons, -255, 255, 0, 180 );
    elevator.write(output_elevator_maped);
    ailerons.write(output_ailerons_maped);
    }




    //SerialReceive_elevator();
  //  SerialSend_elevator();
  //  SerialReceive_ailerons();
  //  SerialSend_ailerons();
/*
//User friendly
  Serial.print("Airspeed pitot: ");
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


  /*
  Serial.print(speed_pitot);
  Serial.print("  ");
  Serial.print(vertical_speed);
  Serial.print("  ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print("  ");
  Serial.println(altitude_filtered);
  Serial.print("  ");
  Serial.print(accelX );
  Serial.print(" ");
  Serial.print(accelY );
  Serial.print(" ");
  Serial.print(accelZ);
  Serial.print("  ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);
  Serial.print("  ");
  Serial.print(pressure);
  Serial.print("  ");
  Serial.print(temperature_BMP);
  Serial.print(" ");
  Serial.print(temperature_MPU);
    Serial.print("  ");
  Serial.print(soc_bat_1);
  Serial.print(" ");
  Serial.print(soc_bat_1);
  */
  Serial.println(analogRead(pin_elevator_monitor));

}
