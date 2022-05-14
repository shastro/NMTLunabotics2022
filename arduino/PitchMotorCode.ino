/*Donovan test
 *
 */

// The following define the pins for the lab
#define HBridge_in1 8
#define HBridge_in2 9
#define HBridge_in3 10
#define HBridge_in4 11

#define homePin     4
#define extendPin   5
#define retractPin  6
#define halfExtPin  12

#define pitchL  2
#define pitchR  3
#define bPin    A2

int encoder = 0;
int encoderL = 0;
int encoderlast = 1;
int state = 0; // Colapsed = 0, extend half = 1, extend full = 2
int eStop = LOW;

void setup()
{
  Serial.begin(9600);
  pinMode(HBridge_in1, OUTPUT);
  pinMode(HBridge_in2, OUTPUT);
  pinMode(HBridge_in3, OUTPUT);
  pinMode(HBridge_in4, OUTPUT);
  pinMode(bPin, INPUT_PULLUP);
  pinMode(4, INPUT);  // Home
  pinMode(5, INPUT);  // Extend
  pinMode(6, INPUT);  // Retract
  pinMode(12, INPUT); // 1/2 Extend
  attachInterrupt(digitalPinToInterrupt(pitchL), c, RISING);
  attachInterrupt(digitalPinToInterrupt(pitchR), cL, RISING);
  attachInterrupt(digitalPinToInterrupt(bPin), button, HIGH);
  // Inturrupt for encoder A used to detect changes in revolutions
}

void loop()
{
  Home();
  Extend();
  Retract();
  HalfExtend();
}

void c()
{
  encoder++;
}

void cL()
{
  encoderL++;
}

void button()
{
  eStop = !eStop;
}

void Home()
{
  if (digitalRead(homePin) == HIGH)
  { // Home
    Serial.println("Home");
    digitalWrite(HBridge_in1, LOW);
    digitalWrite(HBridge_in2, HIGH);
    digitalWrite(HBridge_in3, LOW);
    digitalWrite(HBridge_in4, HIGH);
    while (encoderlast != encoder)
    { //&& eStop == LOW){
      Serial.println("Home2");
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      encoderlast = encoder;
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
    encoder = 0;
    state = 0;
    // eStop == LOW;
  }
}

void Extend()
{
  if (digitalRead(extendPin) == HIGH)
  { // Extend
    Serial.println("Extend");
    while (encoder <= 500 && eStop == LOW)
    {
      Serial.print(encoder);
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
    encoder = 0;
    state = 2;
    // eStop == LOW;
  }
}

void HalfExtend()
{
  if (digitalRead(halfExtPin) == HIGH)
  { // Extend
    Serial.println("Half Extend");
    while (encoder <= 320 && eStop == LOW)
    {
      Serial.print(encoder);
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
    encoder = 0;
    state = 1;
    // eStop == LOW;
  }
}

void Retract()
{
  if (digitalRead(retractPin) == HIGH && state == 2)
  { // Retract
    Serial.println("Retract");
    while (encoder <= 500 && eStop == LOW)
    {
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      digitalWrite(HBridge_in1, LOW);
      digitalWrite(HBridge_in2, HIGH);
      digitalWrite(HBridge_in3, LOW);
      digitalWrite(HBridge_in4, HIGH);
    }
    eStop == LOW;
    Home();
  }
  if (digitalRead(retractPin) == HIGH && state == 1)
  { // Retract
    while (encoder <= 250 && eStop == LOW)
    {
      Serial.print(encoder);
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
    Home();
  }
}
