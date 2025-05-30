#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#define DELTA 0.02
#define ALFA 0.05
#define MAX_ABS 45
#define PI 3.1415

Servo myservo;
Adafruit_MPU6050 mpu;

void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
float observador(float u, float angulo_medido);
float observador_sesgo(float u, float velocidad_medido, float angulo_medido);

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7);

void setup(void) {
  Serial.begin(115200);
  

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Init servo
  myservo.attach(5, 500, 2500);  //500 -> 0, 2500->180
  mover_servo(0);

  //Potenciometro
  pinMode(A0, INPUT);

  delay(1000);
}

float angulo_estimado_print = 0;
float velocidad_estimado_print = 0;
float sesgo_estimado_print = 0;

void loop() {

  static float ref = 0;
  static float angulo = 0;
  static float angulo_gyro = 0;
  static float angulo_accel = 0;
  static float u = 0;

  unsigned long t1 = micros();

  //Medicion de angulos
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel; // en radianes

  //Referencia
  ref = analogRead(A0)*(2*0.52/1024.0) - 0.52; 
  u = ref;
  
  observador(u, angulo);

  mover_servo(u);

  //Datos
  matlab_send(angulo, angulo_estimado_print, gyro_x, velocidad_estimado_print, ref, sesgo_estimado_print, 0);

  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}

void mover_servo(float grados) {

  float g = 0;

  g = grados;

  if(g > 0.7) g = 0.7;
  if(g < -0.7) g = -0.7;

  float pwm = 0;
  pwm = (grados * 2000.0 / PI) + 1500;
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

float observador(float u, float angulo_medido){
  // las variables de las matrices discretizadas
  const float A[4][4] =  {{1, 0.02, 0, 0},
                          {0, 0.9238, 0.1756, 0},
                          {0, 0, 1.0000, 0.0200},
                          {0, 0, -3.0896, 0.5852}};

  const float Bd[4] = {0, 0, 0, 1.2978}; 
         
 
  return 0;

}