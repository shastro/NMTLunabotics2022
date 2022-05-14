/*Donovan test
 *
 */

//The following define the pins for the lab
#define in1 9
#define in2 10
#define in3 11
#define in4 12
#define pitchL 20
#define pitchR 21
#define bPin 3

int encoder = 0;
int encoderL = 0;
int encoderlast = 1;
int state = 0; //Colapsed = 0, extend half = 1, extend full = 2
int eStop = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(bPin, INPUT_PULLUP);
  pinMode(23, INPUT); //Home
  pinMode(25, INPUT); //Extend
  pinMode(27, INPUT); //Retract
  pinMode(29, INPUT); //1/2 Extend
  attachInterrupt(digitalPinToInterrupt(pitchL), c, RISING);
  attachInterrupt(digitalPinToInterrupt(pitchR), cL, RISING);
  attachInterrupt(digitalPinToInterrupt(bPin), button, HIGH);

  //Inturrupt for encoder A used to detect changes in revolutions
}

void loop() {
  Home();
  Extend();
  Retract();
  HalfExtend();
}

void c() {
  encoder++ ;
}

void cL() {
  encoderL++ ;
}

void button(){
  eStop = !eStop;
}

void Home(){
  if (digitalRead(23)==HIGH){ //Home
    analogWrite(in1, 0);
    analogWrite(in2,255);
    analogWrite(in3, 0);
    analogWrite(in4,255);
    while (encoderlast != encoder && eStop == LOW){
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      encoderlast = encoder;
      analogWrite(in1, 0);
      analogWrite(in2,255);
      analogWrite(in3, 0);
      analogWrite(in4,255);
      Serial.println(eStop);
      delay(500);
    }
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
    encoder = 0;
    state = 0;
    // eStop == LOW;
  }
}

void Extend(){
  if (digitalRead(25)==HIGH){ //Extend
    while (encoder <= 500 && eStop == LOW){
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      analogWrite(in1, 255);
      analogWrite(in2, 0);
      analogWrite(in3, 255);
      analogWrite(in4, 0);
    }
    analogWrite(in1,0);
    analogWrite(in2, 0);
    analogWrite(in3,0);
    analogWrite(in4, 0);
    encoder = 0;
    state = 2;
    // eStop == LOW;
  }
}

void HalfExtend(){
  if (digitalRead(29)==HIGH){ //Extend
    while (encoder <= 320 && eStop == LOW){
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      analogWrite(in1, 255);
      analogWrite(in2, 0);
      analogWrite(in3, 255);
      analogWrite(in4, 0);
    }
    analogWrite(in1,0);
    analogWrite(in2, 0);
    analogWrite(in3,0);
    analogWrite(in4, 0);
    encoder = 0;
    state = 1;
    //eStop == LOW;
  }
}

void Retract(){
  if (digitalRead(27)==HIGH && state == 2){ //Retract
    while (encoder <= 500 && eStop == LOW){
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      analogWrite(in1, 0);
      analogWrite(in2, 255);
      analogWrite(in3, 0);
      analogWrite(in4, 255);
    }
    eStop == LOW;
    Home();
  }
  if (digitalRead(27)==HIGH && state == 1){ //Retract
    while (encoder <= 250 && eStop == LOW){
      Serial.print(encoder);
      Serial.print(",  ");
      Serial.println(encoderlast);
      analogWrite(in1, 0);
      analogWrite(in2, 255);
      analogWrite(in3, 0);
      analogWrite(in4, 255);
    }
    /* float x = 250;
       while (encoderlast != encoder){
       Serial.print(encoder);
       Serial.print(",  ");
       Serial.println(encoderlast);
       encoderlast = encoder;
       analogWrite(in3, 0);
       analogWrite(in4, x);
       x = x-5;
       delay(200);
       }
       analogWrite(in3,0);
       analogWrite(in4, 0);
       encoder = 0;
       }*/
    //eStop == LOW;
    Home();
  }
}
