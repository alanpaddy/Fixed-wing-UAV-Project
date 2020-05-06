// Program to:
// - read servo input
// - output binary servo output on other pins to manage gear doors and wheels on model aircraft
// - fully adjustable end points on two independent gear doors outpus
// - fully adjustable timing for wheels up, doors close & doors open, wheels down
// Code by RGN01 on RCMF
// 2013-03-24

#include <Servo.h>

Servo ServoWheels;                       // create servo object to control a servo for the wheels
Servo ServoDoor_1;                       // create servo object to control a servo for gear door 1
Servo ServoDoor_2;                       // create servo object to control a servo for gear door 2
int ServoIn;                             // integer to capture servo input value from RX
int ServoOut_1;                          // integer to use for servo 1 output
int ServoOut_2;                          // integer to use for servo 2 output
int ServoOut_3;                          // integer to use for servo 3 output
int DoorUp_1;                            // limit setting for door 1 up
int DoorUp_2;                            // limit setting for door 2 up
int DoorDown_1;                          // limit setting for door 1 down
int DoorDown_2;                          // limit setting for door 2 down
int Delay_up;                            // delay setting for doors up
int Delay_down;                          // delay setting for doors down
int ServoUp;                             // integer to use to 'debounce' the input
int ServoDown;                           // integer to use to 'debounce' the input
const int PinServoInput = 8;             // set pin for servo input from RX
const int PinServoWheels = 9;           // set pin for servo driver for wheels
const int PinServoDoors_1 = 10;          // set pin for servo driver for doors 1
const int PinServoDoors_2 = 11;          // set pin for servo driver for doors 2
const int PinValDoors_1up = 4;           // set pin for input value for doors 1 up potentiomente input
const int PinValDoors_1down = 5;         // set pin for input value for doors 1 down potentiomente input
const int PinValDoors_2up = 3;           // set pin for input value for doors 2 up potentiomente input
const int PinValDoors_2down = 1;         // set pin for input value for doors 2 down potentiomente input
const int PinValDoorsUpDelay = 2;        // set pin for input value for doors up delay potentiomente input
const int PinValDoorsDownDelay = 0;      // set pin for input value for doors down delay potentiomente input

// The next section runs once on startup
void setup() {
  pinMode(PinServoInput, INPUT);         // Sets the input pin as an input
//  Serial.begin(9600);                  // Set up serial monitor
  ServoWheels.attach(PinServoWheels);    // attaches the servo on the pin dedicated to the wheels to the servo object

  ServoWheels.write(160);                // raises wheels to start
  delay(Delay_up);                       // delay to let wheels raise before doors close

  ServoUp = 1600;                        // initial setting for 'debounce' value
  ServoDown = 1400;                      // initial setting for 'debounce' value
  delay(1000);                           // dealy to stabilise before loop runs
}

// the next section runs continually
void loop() {
  ServoIn = pulseIn(PinServoInput, HIGH, 25000);          // Read the pulse width of servo input from RX
  DoorUp_1 = analogRead(PinValDoors_1up);                 // reads the Doors 1 up limit (0 - 1023) from the potentiometer
  DoorUp_1 = map(DoorUp_1, 0, 1023, 0, 179);              // scale it to use it with the servo (value between 0 and 180)
  DoorUp_2 = analogRead(PinValDoors_2up);                 // reads the Doors 2 up limit (0 - 1023) from the potentiometer
  DoorUp_2 = map(DoorUp_2, 0, 1023, 0, 179);              // scale it to use it with the servo (value between 0 and 180)
  DoorDown_1 = analogRead(PinValDoors_1down);             // reads the Doors 1 down limit (0 - 1023) from the potentiometer
  DoorDown_1 = map(DoorDown_1, 0, 1023, 0, 179);          // scale it to use it with the servo (value between 0 and 180)
  DoorDown_2 = analogRead(PinValDoors_2down);             // reads the Doors 2 down limit (0 - 1023) from the potentiometer
  DoorDown_2 = map(DoorDown_2, 0, 1023, 0, 179);          // scale it to use it with the servo (value between 0 and 180)
  Delay_up = analogRead(PinValDoorsUpDelay);              // reads the Up delay (0 - 1023) from the potentiometer
  Delay_up = Delay_up *2;                                 // adds multiplier to increase range
  Delay_down = analogRead(PinValDoorsDownDelay);          // reads the down delay (0 - 1023) from the potentiometer
  Delay_down = Delay_down *2;                             // adds multiplier to increase range


// Now determine what state the RX signal is and move the wheels and doors accordingly
  if (ServoIn >= 1600) {                    // if the servo input from the RX is at the high end goto the doors up subroutine
    goto GearUp;
  }
  else if (ServoIn <= 1400) {               // if the servo input from the RX is at the low end goto the doors down subroutine
    goto GearDown;
  }
  goto EndLoop;                             // a catch-all to bypass the subroutines if neither condition is met (unlikely)

GearUp:
  {
    ServoWheels.write(160);           // raises wheels
    if (ServoIn >ServoUp){            // checks whether the input value has changed, if not it skips the delay to improve responsiveness
//      Serial.println("Delay Up");
    delay(Delay_up);                  // delay to let wheels raise
    }
    ServoDoor_1.write(DoorUp_1);      // closes wheel door 1
    ServoDoor_2.write(DoorUp_2);      // closes wheel door 2
    goto EndLoop;                     // exite the subroutine to loop back and start again
  }                                   // end loop
GearDown:
  {
    ServoDoor_1.write(DoorDown_1);    // opens wheel door 1
    ServoDoor_2.write(DoorDown_2);    // opens wheel door 2
    if (ServoIn <ServoDown){          // checks whether the input value has changed, if not it skips the delay to improve responsiveness
//      Serial.println("Delay Down");
      delay(Delay_down);              // delay to let doors drop
    }
    ServoWheels.write(20);            // lowers wheels
    goto EndLoop;                     // exite the subroutine to loop back and start again
  }                                   // end loop
EndLoop:
  ServoUp = ServoIn + 100;            // adds 100 to the current servo in value to help detect if it has changed
  ServoDown = ServoIn -100;           // adds 100 to the current servo in value to help detect if it has changed
  //  delay(1500); // I put this here just to make the terminal window happier when debugging
}
