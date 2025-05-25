#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

#define DELTA 0.02
#define ALFA 0.05
#define MAX_ABS 45

#define ECHO_PIN 6
#define TRIGGER_PIN 7

Servo myservo;
NewPing sonar(TRIGGER_PIN,ECHO_PIN, 100);

Adafruit_MPU6050 mpu;

void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
float PID_bilineal(float ref, float medicion);

void setup(void) {
  Serial.begin(115200);
  
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens


  //Potenciometro
  pinMode(A0, INPUT);

  //Init servo
  myservo.attach(5, 500, 2500);  //500 -> 0, 2500->180
  
  mover_servo(0);

  delay(2000);

}


float Dk_matlab = 0;
float Ik_matlab = 0;

void loop() {

  static float ref = -0.1;
  static float angulo = 0;
  static float angulo_gyro = 0;
  static float angulo_accel = 0;

  static float u = 0;
  static float u1 = 0;
  static float u2 = 0;
  static float error = 0;
  static float error1 = 0;
  static float error2 = 0;

  float distance = 0;

  unsigned long t1 = micros();

  //Leo la referencia

  /*
  ref = analogRead(A0)*(300.0/1024.0) - 150; 
  ref = -ref;
  */

  //Medicion de distancia
  distance = (sonar.ping() ) * (0.343/2) * (1.0/1000.0); // microseg a mm
  distance -= 157.0* (1.0/1000.0); //ajusto el cero
  
  //Controlador de posicion PID
  u = PID_bilineal(ref, distance);

  mover_servo(u);

  //Datos
  matlab_send(distance, ref,  ref-distance, u, Ik_matlab, Dk_matlab, 0);

  //Frecuencia de muestreo
  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}

void mover_servo(float grados) {

  float g = 0;

  g = grados;

  if(g > 40) g = 0.7;
  if(g < -40) g = -0.7;
  
  float pwm = 0;
  pwm = (g * 2000.0 / 3.14) + 1500;
  myservo.writeMicroseconds(pwm);
}

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7) {
  Serial.write("abcd");
  byte *b = (byte *)&dato1;
  Serial.write(b, 4);
  b = (byte *)&dato2;
  Serial.write(b, 4);
  b = (byte *)&dato3;
  Serial.write(b, 4);
  b = (byte *)&dato4;
  Serial.write(b, 4);
  b = (byte *)&dato5;
  Serial.write(b, 4);
  b = (byte *)&dato6;
  Serial.write(b, 4);
  b = (byte *)&dato7;
  Serial.write(b, 4);

  //etc con mas datos tipo float. Tambien podría pasarse como parámetro a esta funcion un array de floats.
}


float estimar_angulo_gyro(float gyro, float angulo_prev) {
  return (angulo_prev + gyro * DELTA);
}

float estimar_angulo_accel(float accel_z, float accel_y) {
  float anguloRadianes = atan2(accel_y, accel_z);
  return anguloRadianes;
}

float PID_bilineal(float ref, float medicion){

  float kp = 2.5, kd = 0, ki = 0.5, Ts = 0.02; //tp2
  //float kp = 0.15, kd = 0.0000008, ki = 0.07, Ts = 0.02; //actividad en clase
  
  float uk = 0;
  static float ek_1 = 0, ek = 0, Dk = 0, Ik = 0;

  // Cálculo de error
  ek = ref - (medicion);

  // Término integral (usando integración trapezoidal)
  Ik += (Ts/2.0) * ek;
  Ik += (Ts/2.0) * ek_1;

  // Término derivativo (diferenciación discreta)
  Dk = 2 * ((ek - ek_1) / Ts) - Dk;

  // Ley de control PID completa
  uk = kp*ek + ki * Ik + kd * Dk;

  // Actualización del error anterior
  ek_1 = ek;

  //matlab
  Dk_matlab = Dk;
  Ik_matlab = Ik;

  return uk;
}