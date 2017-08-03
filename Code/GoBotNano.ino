/*
   Code for the Arduino Nano to be used with the Go-Bot cart. The Nano will be contained in the front controller box, which will contain -

   Arduino Nano mounted to a PCB.
   An analog joystick.
   9 pin serial connection wired for an attari digital joystick or wireless joystick.
   A switch to control the raising and lowering of the seat
   A potentiometer to control the overall speed of the cart
   Digital jacks for buttons/switches to control // Readings for Quick Stop, Brake Release, Button to shut off system, Seat-up command, and Seat-down command.
   CAT5 cable to send the data to the back of the cart to the Arduino Mega.
   SPDT Momentary action switch for on off/light indication.
   A secondary easy reach on Btn

   This code will collect all data attached to the Arduino Nano and send it via the CAT5 cable back to the Arduino Mega.

   By: Reagan Stovall, Andrew Gates, and Jesse Wiklanski in conjunction with Robert Gutmann.
   For: Mary Bridge Children's Therapy Unit at Good Samaritan Hospital.
*/

// Constanst for Analog Pins
const int FBPin = A2;
const int LRPin = A1;
const int potPin = A0;
const int tglPin = A3;

// Constants for the digital Pins
const int ledPin = 12;       // LED to let hte user know the cart is on
const int onOffPin = 11;     // Button to turn system off
const int forwardPin = 10;   // Digital Forward pin
const int reversePin = 9;    // Digital Reverse pin
const int leftPin = 8;       // Digital Left pin
const int rightPin = 7;      // Digital Right pin
const int QSPin = 6;         // Digital Quick Stop pin
const int MPPin = 5;         // Digital Miscellaneous Power pin
const int buttonPin = 4;     // Digital Atari button pin
const int SUPin = 3;         // Digital Seat Up pin
const int SDPin = 2;         // Digital Seat Down pin

int connectionCounter = 0;

void setup()
{
  pinMode(forwardPin, INPUT_PULLUP);  //2
  pinMode(reversePin, INPUT_PULLUP);  //3
  pinMode(leftPin, INPUT_PULLUP);     //4
  pinMode(rightPin, INPUT_PULLUP);    //5
  pinMode(QSPin, INPUT_PULLUP);       //6
  pinMode(MPPin, OUTPUT);             //7
  pinMode(buttonPin, INPUT_PULLUP);   //8
  pinMode(SUPin, INPUT_PULLUP);       //9
  pinMode(SDPin, INPUT_PULLUP);       //10
  pinMode(onOffPin, INPUT_PULLUP);    //11
  pinMode(ledPin, OUTPUT);            //12
  pinMode(tglPin, INPUT_PULLUP);    

  digitalWrite(ledPin, HIGH);         // Lets the user know the cart is on.
  Serial.begin(19200);
}

void loop()
{
  // This sets the high reference for the analog read to whatever
  // voltage the AREE pin is reading, in this case, it should be
  // wired to the same power supply as the analog sensors... 3.3V
  analogReference(EXTERNAL);

  char start  = ':';   // starts the set
  char next   = ',';   // useded to break up the string
  int tempInt = 0;     // used as a dummy var

  // This toggles the Micelaneious pin when the top button is pressed
  if (digitalRead(tglPin) == 0) digitalWrite(MPPin, HIGH);
  else digitalWrite(MPPin, LOW);

  // This bit filters any noise out of the center range first so
  // when idleing, there is a consistent zero read. Then the
  // temp Int is mapped to the range expected by the motor.
  // While the Sabertooth Motor Controller needs a range from -2047
  // to 2047, the mapping is changed speciffically for this joystick
  // If any problems result, try reading the serial port of the Nano
  // and adjusting the range appropriately. keep in mind that positive
  // and negative shoud increase or deascrease the same amount.
  tempInt = analogRead(FBPin);
  tempInt = (tempInt > 534 || tempInt < 490) ? tempInt : 512;
  tempInt = (map(tempInt, 0, 1024, -2407, 2407));
  if (tempInt <= -2047) tempInt = -2047;
  else if (tempInt >= 2047) tempInt = 2047;
  String FBVal = String(tempInt);

  tempInt = analogRead(LRPin);
  tempInt = (tempInt > 534 || tempInt < 490) ? tempInt : 512;
  tempInt = (map(tempInt, 0, 1024, -2457, 2457));
  if (tempInt <= -2047) tempInt = -2047;
  else if (tempInt >= 2047) tempInt = 2047;
  String LRVal = String(tempInt);

  String pot;
  tempInt = analogRead(potPin);
  tempInt = (map(tempInt, 0, 1024, 0, 100));
  if (tempInt > 100) pot = String(50);
  else pot = String(tempInt);

  // These mappings are for the motor controller, order is important.
  tempInt = digitalRead(rightPin);
  String rightVal = String(map(tempInt, 0, 1, 2047, 0));
  tempInt = digitalRead(leftPin);
  String leftVal = String(map(tempInt, 0, 1, -2047, 0));
  tempInt = digitalRead(reversePin);
  String reverseVal = String(map(tempInt, 0, 1, -2047, 0));
  tempInt = digitalRead(forwardPin);
  String forwardVal = String(map(tempInt, 0, 1, 2047, 0));

  // This is a simple counter indicator to let the Mega know the data is still being sent
  connectionCounter += 1;
  connectionCounter = connectionCounter % 10;

  
  if ((digitalRead(forwardPin) != 1) || (digitalRead(reversePin) != 1) || 
      (digitalRead(rightPin) != 1) || (digitalRead(leftPin) != 1) || 
      (digitalRead(buttonPin) != 1)){
      FBVal = String(0);
      LRVal = String(0);
    }

  
  // Readings for Quick Stop, Miscellaneous Power, Button, Seat-up command, and Seat-down command.
  String QSVal = String(digitalRead(QSPin), DEC);
  String MPVal = String(digitalRead(MPPin), DEC);
  String Btn   = String(digitalRead(buttonPin), DEC);
  String SUVal = String(digitalRead(SUPin), DEC);
  String SDVal = String(digitalRead(SDPin), DEC);
  String onOff = String(digitalRead(onOffPin), DEC);
  String Count = String(connectionCounter, DEC);

  //array[0] = analog forward and back
  //array[1] = analog left and right
  //array[2] = pot
  //array[3] = d-forward
  //array[4] = d-reverse
  //array[5] = d-left
  //array[6] = d-right
  //array[7] = d-killswitch
  //array[8] = d-misc
  //array[9] = d-ataribtn
  //array[10] = d-seatup
  //array[11] = d-seatdown
  //array[12] = d-off
  //array[13] = connectionCounter
  // String to be sent over to the Mega containing all of the data.
  String sendString = String(start + LRVal + next + FBVal + next + pot + next + forwardVal
                             + next + reverseVal + next + leftVal + next + rightVal + next
                             + QSVal + next + MPVal + next + Btn + next + SUVal + next
                             + SDVal + next + onOff + next + Count);
  Serial.println(sendString);

  Serial.flush();   // insures that all data in the buffer is sent before more
}
