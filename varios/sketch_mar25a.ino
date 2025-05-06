#include <NewPing.h>
#include <Servo.h>

#define ECHO_PIN 3
#define TRIGGER_PIN 2
#define FRECUENCIA 50.0

float distance;
float dif_temp;
float freq;
unsigned long a;
unsigned long b;
NewPing sonar(TRIGGER_PIN,ECHO_PIN);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {

a = micros();
// put your main code here, to run repeatedly:
distance= sonar.ping() / 29.287; // microseg a cm
b = micros();

freq = 1000000/(b-a);

Serial.println(freq);

delay(100);

/*
  a = micros();
  // put your main code here, to run repeatedly:
  distance= sonar.ping() / (2*29.287); // microseg a cm
  Serial.println(distance);
  b = micros();
  dif_temp = (b-a)/1000;
  delay(1000.0/FRECUENCIA - dif_temp);
  */
}

