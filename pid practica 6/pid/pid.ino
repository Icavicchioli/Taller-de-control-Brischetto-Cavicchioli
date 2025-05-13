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

//Adafruit_MPU6050 mpu;

void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
float PID_bilineal(float ref, float medicion);

void setup(void) {
  Serial.begin(115200);
  
  /*
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  */
  /*
  // Try to initialize!
  if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  */

  //Potenciometro
  pinMode(A0, INPUT);

  //Init servo
  myservo.attach(5, 500, 2500);  //500 -> 0, 2500->180
  
  mover_servo(0);

  delay(2000);

}


void loop() {

  static float ref = 0;
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

  ref = analogRead(A0)*(300.0/1024.0) - 150; 
  ref = -ref;

  /*
  //Medicion de angulo
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel; // en radianes
*/
  //Medicion de distancia
  distance = (sonar.ping() * 0.8) * (0.343/2) + 0.2*distance; // microseg a mm
  distance -= 157; //ajusto el cero
  
  /*
  //Controlador de angulo
  //coefs tustin
  #define E0 1.737
  #define E1 -1.433
  #define U1 1

  error = ref + (angulo * 180/3.1415); //asi la referencia puede ser en grados

  u = (E0 * error + E1 * error1 + U1 * u1);

  error2 = error1;
  error1 = error;
  u2 = u1;
  u1 = u;
  */
  //Controlador de posicion PID
  u = PID_bilineal(ref, distance);

  mover_servo(u);


  Serial.println(u);

/*
  Serial.print(ref);
  Serial.println();
*/  
  //Datos
  //matlab_send( error,  error1,  u,  u1, angulo);

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

  if(g > 40) g = 40;
  if(g < -40) g = -40;
  
  float pwm = 0;
  pwm = (g * 2000.0 / 180) + 1500;
  myservo.writeMicroseconds(pwm);
}

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5) {
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

  float kp = 0.18, kd = 0.0002, ki = 0.008, Ts = 0.02;
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

  //Serial.println(Ik);
  //Serial.println(" ");
  //Serial.print(uk);

  //Serial.print(ek);
  return uk;
}
