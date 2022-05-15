/**
 * @file PitchMotorController.ino
 * @brief code for the Arduino used to control the linear actuators for auger inclination
 * @version 0.2 - refactored function and variable names
 *          0.1 - created Donovan Caruso
 */

// The following define the pins for the lab
#define HBridge_in1 8
#define HBridge_in2 9
#define HBridge_in3 10
#define HBridge_in4 11

#define homePin 4
#define extendPin 5
#define retractPin 6
#define halfExtPin 12

#define modeSelect0 4
#define modeSelect1 5
#define enablePin 6

#define pitchL 2
#define pitchR 3
// #define bPin    A2 // this may be illegal due to interrupt, later...

int encoderR = 0;
int encoderL = 0;
int encoderlast = 1;
int state = 0; // Colapsed = 0, extend half = 1, extend full = 2
int eStop = LOW;

int mode = 0; // determined by mode pins in void loop
bool enable = LOW;

enum modes
{
  modeHome,
  modeExtend,
  modeRetract,
  modeHalfExt
};

void setup()
{
  Serial.begin(9600);
  pinMode(HBridge_in1, OUTPUT);
  pinMode(HBridge_in2, OUTPUT);
  pinMode(HBridge_in3, OUTPUT);
  pinMode(HBridge_in4, OUTPUT);
  // pinMode(bPin, INPUT_PULLUP);

  // pinMode(homePin, INPUT);    // Home
  // pinMode(extendPin, INPUT);  // Extend
  // pinMode(retractPin, INPUT); // Retract
  // pinMode(halfExtPin, INPUT); // 1/2 Extend
  pinMode(modeSelect0, INPUT);
  pinMode(modeSelect1, INPUT); // mode select pins
  pinMode(enablePin, INPUT);   // enable pin

  attachInterrupt(digitalPinToInterrupt(pitchL), EncoderCountR, RISING);
  attachInterrupt(digitalPinToInterrupt(pitchR), EncoderCountL, RISING);
  // attachInterrupt(digitalPinToInterrupt(bPin), button, HIGH); // Cannot assign interrupt to pin A2 on ATMega32U4 series Arduinos...
  // Inturrupt for encoder A used to detect changes in revolutions
}

void loop()
{
  mode = digitalRead(modeSelect0) & (digitalRead(modeSelect1) << 1);
  enable = digitalRead(enablePin);
  if (enable) // only hit the switch statement
    switch (mode)
    {
    case modeHome:
      Home();
      break;
    case modeExtend:
      Extend();
      break;
    case modeRetract:
      Retract();
      break;
    case modeHalfExt:
      HalfExtend();
      break;

    default:
      // do nothing if invalid mode
      break;
    }
}

/* encoder count incrementers (triggered on pin rising interrupt) */
void EncoderCountR() { encoderR++; }
void EncoderCountL() { encoderL++; }

// void button()
// {
//   eStop = !eStop;
// }

void Home()
{
  // if (digitalRead(homePin) == HIGH)
  // { // Home
  Serial.println("Home");
  digitalWrite(HBridge_in1, LOW);
  digitalWrite(HBridge_in2, HIGH);
  digitalWrite(HBridge_in3, LOW);
  digitalWrite(HBridge_in4, HIGH);
  while (encoderlast != encoderR)
  { //&& eStop == LOW){
    Serial.println("Home2");
    Serial.print(encoderR);
    Serial.print(",  ");
    Serial.println(encoderlast);
    encoderlast = encoderR; // this will be the same once actuators reach home (stop moving at limit switch)
    digitalWrite(HBridge_in1, LOW);
    digitalWrite(HBridge_in2, HIGH);
    digitalWrite(HBridge_in3, LOW);
    digitalWrite(HBridge_in4, HIGH);
    Serial.println(eStop);
    delay(500);
  }
  digitalWrite(HBridge_in1, LOW);
  digitalWrite(HBridge_in2, LOW);
  digitalWrite(HBridge_in3, LOW);
  digitalWrite(HBridge_in4, LOW);
  encoderR = 0;
  state = 0;
  // eStop == LOW;
  // }
}

void Extend()
{
  // if (digitalRead(extendPin) == HIGH)
  // { // Extend
  Serial.println("Extend");
  while (encoderR <= 500 && eStop == LOW)
  {
    Serial.print(encoderR);
    Serial.print(",  ");
    Serial.println(encoderlast);
    digitalWrite(HBridge_in1, HIGH);
    digitalWrite(HBridge_in2, 0);
    digitalWrite(HBridge_in3, HIGH);
    digitalWrite(HBridge_in4, LOW);
  }
  digitalWrite(HBridge_in1, LOW);
  digitalWrite(HBridge_in2, LOW);
  digitalWrite(HBridge_in3, LOW);
  digitalWrite(HBridge_in4, LOW);
  encoderR = 0;
  state = 2;
  // eStop == LOW;
  // }
}

void HalfExtend()
{
  // if (digitalRead(halfExtPin) == HIGH)
  // { // Extend
  Serial.println("Half Extend");
  while (encoderR <= 320 && eStop == LOW)
  {
    Serial.print(encoderR);
    Serial.print(",  ");
    Serial.println(encoderlast);
    digitalWrite(HBridge_in1, HIGH);
    digitalWrite(HBridge_in2, LOW);
    digitalWrite(HBridge_in3, HIGH);
    digitalWrite(HBridge_in4, LOW);
  }
  digitalWrite(HBridge_in1, LOW);
  digitalWrite(HBridge_in2, LOW);
  digitalWrite(HBridge_in3, LOW);
  digitalWrite(HBridge_in4, LOW);
  encoderR = 0;
  state = 1;
  // eStop == LOW;
  // }
}

void Retract()
{
  // if (digitalRead(retractPin) == HIGH && state == 2)
  // { // Retract
  if (state == 2)
  { // Retract
    Serial.println("Retract");
    while (encoderR <= 500 && eStop == LOW)
    {
      Serial.print(encoderR);
      Serial.print(",  ");
      Serial.println(encoderlast);
      digitalWrite(HBridge_in1, LOW);
      digitalWrite(HBridge_in2, HIGH);
      digitalWrite(HBridge_in3, LOW);
      digitalWrite(HBridge_in4, HIGH);
    }
    // eStop == LOW;
    // Home();
  }
  // if (digitalRead(retractPin) == HIGH && state == 1)
  // { // Retract
  else if (state == 1)
  { // Retract
    while (encoderR <= 250 && eStop == LOW)
    {
      Serial.print(encoderR);
      Serial.print(",  ");
      Serial.println(encoderlast);
      digitalWrite(HBridge_in1, LOW);
      digitalWrite(HBridge_in2, HIGH);
      digitalWrite(HBridge_in3, LOW);
      digitalWrite(HBridge_in4, HIGH);
    }
    /* float x = 250;
     while (encoderlast != encoder){
         Serial.print(encoder);
         Serial.print(",  ");
         Serial.println(encoderlast);
         encoderlast = encoder;
         digitalWrite(HBridge_in3, LOW);
         digitalWrite(HBridge_in4, x);
         x = x-5;
         delay(200);
         }
       digitalWrite(HBridge_in3,LOW);
       digitalWrite(HBridge_in4, LOW);
       encoder = LOW;
      }*/
    // eStop == LOW;
    // Home();
  }
}