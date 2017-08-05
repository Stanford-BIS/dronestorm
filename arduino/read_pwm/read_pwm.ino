/*
  Read PWM
  Reads in the PWM signal from a pin and
  indicates its value in the brightness of the led onboard
 */
 
// pins
int ledpin = 13; // Pin 13 has an LED connected on most Arduino boards.
int pwmpin = 2;

// values
volatile unsigned long pw = 0;
volatile unsigned long trise = 0;
volatile float pw_scaled = 0.;
int duty_cycle = 0;

unsigned int pw_min_us = 1100;
unsigned int pw_max_us = 800;
float k = 255. / ((float) pw_max_us); // convert between measured pw and led duty cycle

void setup() {                
  pinMode(ledpin, OUTPUT);
  pinMode(pwmpin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(pwmpin), read_pwm, CHANGE);
  Serial.begin(9600);
}

void loop() {
}

void read_pwm() {
  if(digitalRead(pwmpin)) {
    trise = micros();
  } else {
    if(trise > 0) {
      pw = micros()-trise;
      duty_cycle = (int) (((float) (pw - pw_min_us)) * k);      
      analogWrite(ledpin, constrain(duty_cycle, 0, 255));
    }
  }
}
