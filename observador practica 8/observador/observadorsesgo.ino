// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#define DELTA 0.02
#define ALFA 0.05
#define MAX_ABS 45

//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo myservo;
Adafruit_MPU6050 mpu;

void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
float observador(float u, float angulo_medido);

void setup(void) {
  Serial.begin(115200);
  
  while (!Serial){
    Serial.println("Failed to find MPU6050 chip");
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  }

  
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
  static float u1 = 0;
  static float u2 = 0;
  static float error = 0;
  static float error1 = 0;
  static float error2 = 0;


  unsigned long t1 = micros();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel; // en radianes

  error = ref + (angulo * 180/3.1415); //asi la referencia puede ser en grados

  u = 0;

  observador(u, angulo);

  mover_servo(u);

  Serial.println(" ");
  Serial.print(angulo_estimado_print);
  Serial.print(velocidad_estimado_print);
  Serial.println(" ");


  //Datos
  //matlab_send( error,  error1,  u,  u1, angulo);

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
  pwm = (grados * 2000.0 / 180) + 1500;
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

float observador(float u, float angulo_medido){
  // las variables de las matrices discretizadas
  #define a11 1
  #define a12 0.02
  #define a21 -3.09
  #define a22 0.5028
  #define b11 0
  #define b12 1.2978
  #define L11 0.5308
  #define L12 -3.0759

  static float angulo = 0;
  static float velocidad = 0;
  static float velocidad_estimado = 0;
  static float angulo_estimado = 0;

  // X(k+1)=A X(k) + X(K) U
  angulo_estimado = a11 * angulo_estimado + a12 * velocidad_estimado + L11 * (angulo_medido - angulo_estimado) + b11 * u; // es el mismo error para ambos pero se multuplica por cosas dferentes
  velocidad_estimado = a21 * angulo_estimado + a22 * velocidad_estimado + L12 * (angulo_medido - angulo_estimado) + b12 * u;

  angulo_estimado_print = angulo_estimado;
  velocidad_estimado_print = velocidad_estimado;
 
  return 0;
  //return (-L11 * angulo_estimado);

}

float observador_sesgo(float u, float angulo_medido){
  // las variables de las matrices discretizadas
#define a11 1
#define a12 0.02
#define a13 0
#define a21 -3.09
#define a22 0.5028
#define a23 0
#define a31 0
#define a32 0
#define a33 1

#define b11 0
#define b21 1.2978
#define b31 0

#define L1 0.0933
#define L2 -0.9233
#define L3 1.9461

  static float angulo = 0;
  static float velocidad = 0;
  static float velocidad_estimado = 0;
  static float angulo_estimado = 0;
  static float sesgo_estimado = 0;

  // Salida estimada
  float salida_estimada = velocidad_estimado + sesgo_estimado;

  // Error entre la medida y la estimación
  float error = angulo_medido - salida_estimada;

  // X(k+1)=A X(k) + X(K) U
  //angulo_estimado = a11 * angulo_estimado + a12 * velocidad_estimado + a13 * sesgo_estimado + L1 * error + b1 * u;
  velocidad_estimado = a21 * angulo_estimado + a22 * velocidad_estimado + a23 * sesgo_estimado + L2 * error + b2 * u;
  sesgo_estimado = a31 * angulo_estimado + a32 * velocidad_estimado + a33 * sesgo_estimado + L3 * error + b3 * u;
  
  
  angulo_estimado_print = angulo_estimado;
  velocidad_estimado_print = velocidad_estimado;
  sesgo_estimado_print = sesgo_estimado;
 
  return 0;
  //return (-L11 * angulo_estimado);

}

