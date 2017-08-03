/*
  Code for the Arduino Mega to be used with the Go-Bot cart. The Mega will be contained in the back of the cart which will contain -

  Arduino Mega mounted to a PCB.
  Pololu G2 High-Power Motor Driver 24v21 for the seat motors.
  Sabertooth 2x32 Motor Controller for the cart motors.
  A joystick jack to allow for analog or digital joysticks to be connected to override the front controller.
  Button to turn on and off the system.
  Power system to control power levels on startup and during runtime.

  This code will collect all data sent by the Arduino Nano and use that data to control the cart unless parental override is active.

  Sabertooth Manual: https://www.dimensionengineering.com/datasheets/Sabertooth2x32.pdf
  Pololu Manual: https://www.pololu.com/product/2995

  By: Andrew Gates, Reagan Stovall and Jesse Wiklanski in conjunction with Robert Gutmann.
  For: Mary Bridge Children's Therapy Unit at Good Samaritan Hospital.
*/

#include <Wire.h>
#include <ArduinoNunchuk.h>
#define BAUDRATE 19200
#define INT_POS = 2047;     // Positive bounds for motor controller.
#define INT_NEG = -2047;    // Negative bounds for motor controller.

ArduinoNunchuk nunchuk = ArduinoNunchuk();

//-------------------- Joystick Data --------------------------//
const int SIZE = 14;
int myFrontArray[SIZE];
int myPreviousFrontArray[SIZE];
int myAtariArray[SIZE];
int myWiiArray[SIZE];
bool wiiInitialized;
bool isWii;
int wiiCounter = 0;
bool frontArraySaved = false;
bool frontArrayError = false;

//-------------------- Power Data --------------------------//
const int buttonTurnOnOffPin = 13;
const int systemStartPin = 8;
const int currentSensePin = A6;
const int PowerStrengthPin = 7;
bool seatInactive = false;
bool driveInactive = false;
int frontShutDownCounter = 0;
int rearShutDownCounter = 0;
int inactiveCounter = 0;
const int maxSpike = 50;
const int overload = 30;     
unsigned long loopTime = 0;

//--------------------- Seat Data -------------------------//
const int motorControlPWMPin = 9;
const int motorControlDIRPin = 10;
const int seatLimitSwitchPin = 11;
int seatCounter = 0;
int seatSignalCounter = 0;
const int seatSpeed = 50;

//-------------------- Cart Data --------------------------//
const int batteryPin = A0;
int batteryLevel = 0;
int driveVal = 0;     // pos = forward, neg = back
int turnVal = 0;      // pos = right, neg = left
bool killSwitch;      // button on the parental controls to stall the motor
bool atariActive = false;

//-------------------- Signal Data --------------------------//
const bool DEBUG = true;
int oldCount = 0;
int sameCount = 0;

//-------------------- Enum Setup --------------------------//
enum ArrayIndices
{
  AnalogForwardBack,
  AnalogLeftRight,
  Potentiometer,
  DigitalFoward,
  DigitalReverse,
  DigitalLeft,
  DigitalRight,
  KillMotor,
  MiscellaneousOutput,
  AtariButton,
  SeatDown,
  SeatUp,
  OnOffButton,
  ConnectionCounter
};

void setup()
{
  pinMode(systemStartPin, OUTPUT);
  digitalWrite(systemStartPin, HIGH);

  //-------------------- Pin Setup --------------------------//
  // Atari hookups, each number is the corresponding pin.
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  // Pwr
  pinMode(buttonTurnOnOffPin, INPUT_PULLUP);
  pinMode(PowerStrengthPin, OUTPUT);

  // Seat Motor
  pinMode(motorControlPWMPin, OUTPUT); // input a
  pinMode(motorControlDIRPin, OUTPUT); // input b
  pinMode(seatLimitSwitchPin, INPUT_PULLUP);

  //-------------------- Timing Setup --------------------------//
  Serial1.begin(9600);             // For Sabertooth
  Serial3.begin(19200);            // The Max that the Nano can transmit
  Serial.begin(115200);

  // Command to send to Sabertooth motor controller for drive strength. Initialize to 0.
  Serial1.print("MD: ");
  Serial1.print(0);
  Serial1.print("\r\n");

  // Command to send to Sabertooth motor controller for turn strength. Initialize to 0.
  Serial1.print("MT: ");
  Serial1.print(0);
  Serial1.print("\r\n");

  //-------------------- Boolean/Declarations --------------------------//
  isWii = false;      // is Wii plugged in?
  killSwitch = false; // stalls motor
  driveVal = 0;       // pos = forward, neg = back
  turnVal = 0;        // pos = right, neg = left

  //-------------------- Nunchuk Initialization --------------------------//
  Wire.begin();
  nunchuk.init(); // initiated in loop as necissary

  delay(3000);    // Allows for the cart to turn on and wait until the system starts up

  //-------------------- Start delay/Fix data --------------------------//
  for (int i = 0; i < 50; i++)
  {
    delay(10);
    readFront();
    printFront();
    Serial.println();
  }
}

void loop()
{
  //------------------- Power/Charging --------------------------//
  batteryLevel = readPower();
  if (DEBUG) Serial.print(readPower());
  // AS OF 6/28 460
  // AS OF 7/31 456
  if (batteryLevel < 450)
  {
    analogWrite(PowerStrengthPin, 255);
  }

  //------------------- Front Controllers --------------------------//
  Serial3.flush();
  readFront();
  Serial3.flush();

  //-------------------- Rear Controllers --------------------------//
  Serial.flush();
  readWii();
  Serial.flush();
  readAtari();
  Serial.flush();
  arrayErrorCheck();

  //-------------------- Print Controllers --------------------------//
  if (DEBUG)
  {
    printFront();
    Serial.flush();
    printWii();
    Serial.flush();
    printAtari();
    Serial.flush();
    printMotorVals();
    Serial.flush();
    Serial.println();
  }

  //-------------------- Controller Signals --------------------------//
  seatControl();
  if (seatInactive == true) cartControl();
  else killMotor();
  
  Serial1.print("MD: ");
  Serial1.print(int(driveVal * float(myFrontArray[Potentiometer])/300));
  Serial1.print("\r\n");

  Serial1.print("MT: ");
  Serial1.print(int(turnVal * float(myFrontArray[Potentiometer])/600));
  Serial1.print("\r\n");

  //-------------------- Inactive Turn Off --------------------------//
  if (driveVal == 0 && turnVal == 0) driveInactive = true;
  else driveInactive = false;

  if (driveInactive == true && seatInactive == true) {
    if (inactiveCounter == 0) {
      loopTime = millis();
      inactiveCounter = 1;
    }
    // Shut down after 5 minutes of inactivity.
    if ((millis() - loopTime) > 300000) {
      if (DEBUG) Serial.print("Inactive");
      shutDown();
    }
  }
  else {
    inactiveCounter = 0;
    loopTime = millis();
  }

  //-------------------- System OnOff Checks --------------------------//
  communicationCheck();
}

// Function to read and save the data that the Arduino Nano is sending from the front of the cart
// into myFrontArray.
void readFront()
{
  if (Serial3.available())
  {
    while (Serial3.available())
    {
      // Looks for the beginning of the string
      if (Serial3.read() == ':')
      {
        for (int i = 0; i < (SIZE); i++)
        {
          // Read the data if it is not a comma
          if (Serial3.peek() != ',')
          {
            myFrontArray[i] = Serial3.parseInt();
          }
          Serial3.read();     // Reads ','
        }
      }
      Serial3.read();         // Reads ':'
    }
  }
  else
  {
    if (DEBUG) Serial.println("Front Unavailabe");
    shutDown();
  }
}

// Function to read and save the various values from the rear Nunchuk. This function will reinitialize
// the Nunchuk if necessary. Otherwise it will save the corresponding values into myWiiArray.
void readWii()
{
  if (wiiCounter == 5)
  {
    // If the Wii controller gets unplugged then plugged back in, reinitialize.
    if (Wire.requestFrom(0x52, 6) != 0)
    {
      nunchuk.init();
      //      if (DEBUG) Serial.println("init_-__--___---____----_____________");
      isWii = true;
    }
    else
    {
      isWii = false;
      myWiiArray[AnalogForwardBack] = 0;
      myWiiArray[AnalogLeftRight] = 0;
      myWiiArray[KillMotor] = 0;
      myWiiArray[MiscellaneousOutput] = 0;
    }

    nunchuk.update();

    if (isWii == true)
    {
      int midPoint = 128;
      int range = 15;

      // Take the current X-Axis reading and convert it according to the midPoint and range values.
      signed int tempInt = nunchuk.analogY;
      tempInt = (tempInt > (midPoint + range) || tempInt < (midPoint - range)) ? tempInt : midPoint;
      myWiiArray[AnalogForwardBack] = map(tempInt, 26, 230, -2047, 2047); // Wii X-Axis

      // Take the current Y-Axis reading and convert it according to the midPoint and range values.
      tempInt = nunchuk.analogX;
      tempInt = (tempInt > (midPoint + range) || tempInt < (midPoint - range)) ? tempInt : midPoint;
      myWiiArray[AnalogLeftRight] = map(tempInt, 26, 230, -2047, 2047); // Wii Y-Axis

      myWiiArray[MiscellaneousOutput] = (nunchuk.cButton); // Wii C button
      myWiiArray[KillMotor] = (nunchuk.zButton); // Wii Z button

      if ((myWiiArray[KillMotor] == 0) && (myWiiArray[MiscellaneousOutput] == 0)) killSwitch = false;
      else killSwitch = true;
    }
    Wire.beginTransmission(0x52);
    Wire.write(0x55);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    wiiCounter = 0;
  }

  else
  {
    wiiCounter++;
  }
}

// Function to read and process the various values from the rear Atari. This function will save the
// corresponding values into myWiiArray.
void readAtari()
{
  for (int i = 0; i < 5; i++)
  {
    myAtariArray[i + 3] = digitalRead(i + 2);
  }

  int tempInt;
  tempInt = myAtariArray[DigitalFoward];
  myAtariArray[DigitalFoward] = map(tempInt, 1, 0, 0, 2047);
  tempInt = myAtariArray[DigitalReverse];
  myAtariArray[DigitalReverse] = map(tempInt, 1, 0, 0, -2047);
  tempInt = myAtariArray[DigitalLeft];
  myAtariArray[DigitalLeft] = map(tempInt, 1, 0, 0, -2047);
  tempInt = myAtariArray[DigitalRight];
  myAtariArray[DigitalRight] = map(tempInt, 1, 0, 0, 2047);

  // If the Nunchuk is not plugged in, assign killSwitch to the Atari button
  if (!isWii)
  {
    killSwitch = ((myAtariArray[KillMotor] == 1) ? false : true);
  }
}

//ReadPower
int readPower()
{
  return analogRead(batteryPin);
}

//PrintPower
void printPwr()
{
  Serial.print("Pwr: " + batteryLevel);
}

// Function to print the values sent from the front Arduino Nano.
void printFront()
{
  Serial.print(" printFront(): ");
  for (int i = 0; i < SIZE; i++)
  {
    Serial.print(myFrontArray[i]);
    Serial.print(", ");
  }
}

// Function to print the values received from the back Nunchuk.
void printWii()
{
  // Wii Print
  Serial.print("  printWii(): ");
  Serial.print(myWiiArray[AnalogForwardBack], DEC);
  Serial.print(", ");
  Serial.print(myWiiArray[AnalogLeftRight], DEC);
  Serial.print(", ");
  Serial.print(myWiiArray[KillMotor], DEC);
  Serial.print(", ");
  Serial.print(myWiiArray[MiscellaneousOutput], DEC);
  Serial.print(", ");
}

// Function to print the values received from the back Atari.
void printAtari()
{
  Serial.print("  printAtari(): ");
  for (int i = 3; i < 8; i++)
  {
    Serial.print(myAtariArray[i], DEC);
    Serial.print(", ");
  }
}

// Function to print the motor values that are assigned from cartControl().
void printMotorVals()
{
  Serial.print("Turn: ");
  Serial.print(turnVal);
  Serial.print(",  Drive: ");
  Serial.print(driveVal);
}

// Function to control the raising and lowering of the seat. MyFrontArray[SeatDown] corresponds to lowering the seat while
// MyFrontArray[SeatUp] corresponds to raising the seat. If the seat is being lowered it will check to see the status of
// the limit switch as well, if that is active then it will stop the output of the motor controller.
void seatControl(void)
{
  // Stuck Seat
  if (analogRead(currentSensePin) > overload) 
  {
    seatCounter++;
    if (seatCounter > 3 || analogRead(currentSensePin) > maxSpike) 
    {
      if (DEBUG) Serial.println("Stuck Seat");
      shutDown();
    }
  }
  else seatCounter = 0;
  
  // Raising of the seat
  if (myFrontArray[SeatUp] == 0)
  {
    if (seatSignalCounter > 5) {
      digitalWrite(motorControlPWMPin, seatSpeed);
      digitalWrite(motorControlDIRPin, HIGH);
      delay(1);
      seatInactive = false;
    }
    else seatSignalCounter++;
  }

  // Lowering of the seat
  else if (myFrontArray[SeatDown] == 0)
  {
    if (seatSignalCounter > 5) {
      // If limit switch is pressed then stop lowering the seat
      if (digitalRead(seatLimitSwitchPin) == 1)
      {
        digitalWrite(motorControlPWMPin, LOW);
        digitalWrite(motorControlDIRPin, LOW);
      }
      else
      {
        digitalWrite(motorControlPWMPin, seatSpeed);
        digitalWrite(motorControlDIRPin, LOW);
        delay(1);
      }
      seatInactive = false;
    }
    else seatSignalCounter++;
  }

  // Seat is inactive, stop raising/lowering it and set seatInactive to true
  else
  {
    digitalWrite(motorControlPWMPin, LOW);
    digitalWrite(motorControlDIRPin, LOW);
    delay(1);
    seatInactive = true;
    seatSignalCounter = 0;
  }
}

// Function that will control which joystick is used for controlling the motors.
// Precedence goes - rear Nunchuk > rear Atari > front Atari/Digital > front Analog
void cartControl()
{
  // If Z button on the Nunchuk or the button on Atari are pressed, shut off motors.
  if (killSwitch)
  {
    killMotor();
  }
  else
  {
    // If a Nunchuk is plugged in
    if (isWii)
    {
      // If Nunchuk is active use it's values
      if (myWiiArray[AnalogForwardBack] != 0 || myWiiArray[AnalogLeftRight] != 0)
      {
        driveVal = myWiiArray[AnalogForwardBack];
        turnVal = myWiiArray[AnalogLeftRight];
      }
      // Else use the values from the front controllers.
      else
      {
        proccessFront();
      }
    }

    // If a Nunchuk is not plugged in
    else
    {
      // If rear Atari is not being used all will be 0
      int ForwardAtariVal = myAtariArray[DigitalFoward];
      int ReverseAtariVal = myAtariArray[DigitalReverse];
      int LeftAtariVal = myAtariArray[DigitalLeft];
      int RightAtariVal = myAtariArray[DigitalRight];

      // If one of the values is not 0, find out which one and assign that to the corresponding direction.
      if (ForwardAtariVal != 0 || ReverseAtariVal != 0 || LeftAtariVal != 0 || RightAtariVal != 0)
      {
        driveVal = ForwardAtariVal + ReverseAtariVal;
        turnVal = LeftAtariVal + RightAtariVal;
        atariActive = true;
      }

      // If Atari was active and now there is no input, set the turn and drive values to 0 and set Atari to inactive
      else if (atariActive == true)
      {
        turnVal = 0;
        driveVal = 0;
        atariActive = false;
      }

      // If there are no values from the Atari, use the values from the front controllers
      else
      {
        proccessFront();
      }
    }
  }
}

// Function that will be called if the parental override is not active. This function will then take the corresponding
// values from the Arduino Nano and then assign the driveVal and turnVal based on what controller is being used in front.
void proccessFront()
{
  int FBVal = 0;
  int LRVal = 0;

  killSwitch = ((myFrontArray[AtariButton] == 1) ? false : true);

  // If digital forward is not 0
  if (myFrontArray[DigitalFoward] != 0)
  {
    FBVal = myFrontArray[DigitalFoward];
  }
  // Else if digital reverse is not 0
  else if (myFrontArray[DigitalReverse] != 0)
  {
    FBVal = myFrontArray[DigitalReverse];
  }
  // Else assign FBVal to 0,
  else
  {
    FBVal = 0;
  }
  // If digital left is not 0
  if (myFrontArray[DigitalLeft] != 0)
  {
    LRVal = myFrontArray[DigitalLeft];
  }
  // Else if digital right is not 0
  else if (myFrontArray[DigitalRight] != 0)
  {
    LRVal = myFrontArray[DigitalRight];
  }
  // Else assign LRVal to 0
  else
  {
    LRVal = 0;
  }

  // If either of these values is not 0 then use the digital outputs to control the motors.
  if (FBVal != 0 || LRVal != 0)
  {
    driveVal  = FBVal;
    turnVal = LRVal;
  }

  // Otherwise use the analog outputs to control the motors.
  else
  {
    driveVal = myFrontArray[AnalogForwardBack];
    turnVal  = myFrontArray[AnalogLeftRight];
  }

  if (killSwitch)
  {
    killMotor();
  }

}

// Function to stop the motors if the button on the joystick is pressed.
void killMotor()
{
  turnVal  = 0;
  driveVal = 0;
}

void communicationCheck() {
  if (digitalRead(buttonTurnOnOffPin) == LOW) {
    if (rearShutDownCounter > 3) {
      if (DEBUG) Serial.println("Rear Shutdown Signal");
      shutDown();
    }
    else rearShutDownCounter++;
  }
  else rearShutDownCounter = 0;

  if (myFrontArray[OnOffButton] == 1) {
    if (frontShutDownCounter > 5) {
      if (DEBUG) Serial.println("Front Shutdown Signal");
      shutDown();
    }
    else frontShutDownCounter++;
  }
  else frontShutDownCounter = 0;

  int newCount = myFrontArray[ConnectionCounter];
  if (newCount == oldCount)
  {
    sameCount += 1;
  }
  else sameCount = 0;
  if (sameCount == 35)
  {
    Serial.println("Count Shutdown Signal");
    shutDown();
  }
  oldCount = newCount;
}

void arrayErrorCheck()
{
  for (int i = 3; i < 14; i++)
  {
    if ((myFrontArray[i] > 1) && (i != 13) && (i > 6))
    {
      frontArrayError = true;
    }
    else if ((myFrontArray[i] > 9) && (i == 13))
    {
      frontArrayError = true;
    }
    else if ((myFrontArray[i] == 1) && (i == 6))
    {
      frontArrayError = true;
    }
    else if ((myFrontArray[i] == 0 && myFrontArray[i + 1] == 0) && (i == 10))
    {
      frontArrayError = true;
    }
    else if ((myFrontArray[i] == 0 && myFrontArray[i + 2] == 0) && (i == 11))
    {
      frontArrayError = true;
    }
  }

  if (frontArrayError == false)
  {
    for (int i = 0; i < 14; i++)
    {
      myPreviousFrontArray[i] = myFrontArray[i];
    }
  }

  else
  {
    for (int i = 0; i < 14; i++)
    {
      myFrontArray[i] = myPreviousFrontArray[i];
    }
  }
  frontArrayError = false;

}

// Function that when called turns off systemStartPin which turns off the entire system.
void shutDown()
{
  if (DEBUG) Serial.println("----------------ShutDown-----------------");
  digitalWrite(systemStartPin, LOW);
}
