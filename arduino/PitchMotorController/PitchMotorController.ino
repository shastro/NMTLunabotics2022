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

#define fullExtendEncoderTarget 500
#define halfExtendEncoderTarget 325

// deprecated pin usage:
// #define homePin 4
// #define extendPin 5
// #define retractPin 6
// #define halfExtPin 12

#define statusLED 13 // the pin connected to the blue SMD LED on the board

#define modeSelect0 4
#define modeSelect1 5
#define enablePin 6

#define debounceCount 50 // number of timer ticks to wait before changing debounced button states

#define pitchL 2
#define pitchR 3
// #define bPin    A2 // this may be illegal due to interrupt, later...

int encoderR = 0;
int encoderL = 0;
int encoderlast = 1;
int eStop = LOW;
int statusLEDcounter = 0;

int mode = 0;           // determined by mode pins in void loop
bool enable = LOW;      // pulled high to enable system; lupped low for system standby
bool enableNew = LOW;   // used for devouncing the enable state
int enableDebounce = 0; // used to avoid enable floating/bouncing

/**
 * @brief states used to keep track of actuator position
 *
 */
enum states
{
  atHome,
  halfExtend,
  fullExtend,
  retracting,
  extending,
  unknown
};
states state = unknown; // state of actuators cannot be known at startup

/**
 * @brief operational modes set by modeSelect pins
 *
 */
enum modes
{
  modeHome,
  modeExtend,
  modeRetract,
  modeHalfExt
};

/**
 * @brief directions valid for actuator actuation
 *
 */
enum directions
{
  reverse = -1,
  stop,
  forward
};

void setup()
{
  Serial.begin(9600);
  Serial.print("Controller Starting...\n");
  pinMode(HBridge_in1, OUTPUT);
  pinMode(HBridge_in2, OUTPUT);
  pinMode(HBridge_in3, OUTPUT);
  pinMode(HBridge_in4, OUTPUT);
  pinMode(statusLED, OUTPUT);
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
  // Inturrupt for encoder A used to detect changes in revolutions

  // Set the Timer Interrupts at the hardware level
  setTimerInterrupts();
}

/**
 * @brief sets hardware-level interrupts timer0 and timer1 at 2kHz and 1Hz, respectively
 *
 */
void setTimerInterrupts()
{
  // set timer0 interrupt at 2kHz
  TCCR0A = 0;
  TCCR0B = 0;                          // set timer0 counter registers to 0
  TCNT0 = 0;                           // counter value to 0
  OCR0A = 124;                         // for 2khz increments must be <65536
  TCCR0A |= (1 << WGM01);              // CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00); // bits for 1024 prescaler
  TIMSK0 |= (1 << OCIE0A);             // enable interrupt

  // set timer1 interrupt at 1Hz
  TCCR1A = 0;
  TCCR1B = 0;                          // set timer1 counter registers to 0
  TCNT1 = 0;                           // counter value to 0
  OCR1A = 15624;                       // for 1hz increments must be <65536
  TCCR1B |= (1 << WGM12);              // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);             // enable interrupt
}
/**
 * @brief timer0 interrupt 2kHz
 */
ISR(TIMER0_COMPA_vect)
{
  // flash the status LED once every second
  if (statusLEDcounter < 100 && !enable)
    digitalWrite(statusLED, HIGH);
  else // flash the LED on for the first 100 counts each 2000-count cycle (1 sec) if not enabled
    digitalWrite(statusLED, enable);
  if (statusLEDcounter == 2000)
    statusLEDcounter = 0;
  else // increment the status LED counter every timer tick, resetting at 2000 (after 1 sec)
    statusLEDcounter++;

  // detect enable state
  enableNew = digitalRead(enablePin); // get the new enable pin state
  if (enable != enableNew)
  {                   // if the new state is not the old state
    enableDebounce++; // increment the debounce counter
    if (enableDebounce > debounceCount)
    {                     // if the new state has been stable long enough
      enable = enableNew; // update the  enable state
      if (enable)
      {
        Serial.print("Enabled: ");
      }
      else
      {
        Serial.println("Disabled: standby mode");
        actuate(stop); // stop all actuations when disabled
      }
      enableDebounce = 0; // and reset the debounce counter
    }
  }
  else
  { // if state returns to old state, reset the debounce counter
    enableDebounce = 0;
  }
}

void loop()
{
  if (enable)
  { // only hit the switch statement
    // mode = digitalRead(modeSelect0) + (digitalRead(modeSelect1) * 2);
    mode = digitalRead(modeSelect0) | (digitalRead(modeSelect1) << 1);
    digitalWrite(statusLED, HIGH); // light status LED when enable pulled high
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
  else
  {
    // digitalWrite(statusLED, LOW); // disable status LED if not enabled
  }
}

/* encoder count incrementers (triggered on pin rising interrupt) */
void EncoderCountR() { encoderR += (state == extending) ? 1 : (state == retracting) ? -1
                                                                                    : 0; }
void EncoderCountL() { encoderL += (state == extending) ? 1 : (state == retracting) ? -1
                                                                                    : 0; }

/**
 * @brief calculates the average encoder count
 *
 * @return int the average encoder count
 */
int encoderAvg() { return (encoderL + encoderR) / 2; }

void Home()
{
  Serial.println("Homing Mode");
  if (state != atHome) // only go home if not already homed
    actuate(reverse);  // begin actuation towards home
  while (encoderlast != encoderAvg() && enable)
  {
    Serial.println("Home2");
    printEncoderCounts();
    encoderlast = encoderAvg(); // this will be the same once actuators reach home (stop moving at limit switch)
    actuate(reverse);
    delay(500);
  }
  actuate(stop);
  encoderR = 0;
  encoderL = 0;
  state = atHome;
}

void Extend()
{
  Serial.println("Extend Mode");
  while (encoderAvg() <= fullExtendEncoderTarget && enable)
  {
    printEncoderCounts();
    actuate(forward);
  }
  actuate(stop);
  // encoderR = 0;
  state = fullExtend;
}

void HalfExtend()
{
  Serial.println("Half Extend Mode");
  if (state == atHome)
  { // transition to 1/2-extended from home position
    while (encoderAvg() <= halfExtendEncoderTarget - 5 && enable)
    {
      printEncoderCounts();
      actuate(forward);
    }
    state = halfExtend;
  }
  else if (state == fullExtend)
  { // transition to 1/2-extended from full-extended position
    while (encoderAvg() >= halfExtendEncoderTarget + 5 && enable)
    {
      printEncoderCounts();
      actuate(reverse);
    }
    state = halfExtend;
  }
  actuate(stop);
  // encoderR = 0;
}

// I don't like this function because state information cannot be obtained reliably; it is advisable to use the home() function for retraction, instead
void Retract()
{
  if (state == fullExtend)
  { // Retract
    Serial.println("Retract Mode\n");
    while (encoderAvg() >= 0 && enable)
    {
      printEncoderCounts();
      actuate(reverse);
    }
  }
  else if (state == halfExtend)
  { // Retract
    while (encoderAvg() >= 0 && enable)
    {
      printEncoderCounts();
      actuate(reverse);
    }
  }
  actuate(stop);
}

/**
 * @brief prints the encoder countes over the serial debug connection
 *
 */
void printEncoderCounts()
{
  Serial.print(encoderAvg());
  Serial.print(", ");
  Serial.println(encoderlast);
}

/**
 * @brief sets the appropriate H-bridge pins for the specified movement
 *
 * @param direction  the direction to move the actuators
 */
void actuate(directions direction)
{
  switch (direction)
  {
  case reverse:
    state = retracting;
    digitalWrite(HBridge_in1, LOW);
    digitalWrite(HBridge_in2, HIGH);
    digitalWrite(HBridge_in3, LOW);
    digitalWrite(HBridge_in4, HIGH);
    break;
  case forward:
    state = extending;
    digitalWrite(HBridge_in1, HIGH);
    digitalWrite(HBridge_in2, LOW);
    digitalWrite(HBridge_in3, HIGH);
    digitalWrite(HBridge_in4, LOW);
    break;

  case stop:
  default:
    digitalWrite(HBridge_in1, LOW);
    digitalWrite(HBridge_in2, LOW);
    digitalWrite(HBridge_in3, LOW);
    digitalWrite(HBridge_in4, LOW);
    break;
  }
}