#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <NewPing.h>

//Definiciones ctes
#define DELTA 0.02
#define ALFA 0.05
#define MAX_ABS 45
#define PI 3.1415

//Definiciones pines
#define ECHO_PIN 6
#define TRIGGER_PIN 7

NewPing sonar(TRIGGER_PIN,ECHO_PIN, 100);
Servo myservo;
Adafruit_MPU6050 mpu;

//Prototipos
void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
float observador(float u, float x1_med, float x3_med);
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8);

void setup(void) {
  Serial.begin(115200);
  
  // Inicializar IMU
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

//Variables globales
float x1_est_print = 0;
float x2_est_print = 0;
float x3_est_print = 0;
float x4_est_print = 0;

void loop() {

  static float ref = 0;
  static float angulo = 0;
  static float angulo_gyro = 0;
  static float angulo_accel = 0;
  static float u = 0;
  static float posicion = 0;

  unsigned long t1 = micros();

  //Medicion de angulo
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel; // en radianes

  //Medicion de posicion 
  posicion = (sonar.ping() ) * (0.343/2.0) * (1.0/1000.0); // microseg a m
  posicion -= 0.157; //ajusto el cero

  //Controladores y observador
  observador(u, posicion, angulo);
  
  u = state_feedback_sin_feed(x1_est_print, x2_est_print, x3_est_print, x4_est_print);

  mover_servo(u);

  //Datos
  matlab_send(posicion, x1_est_print, 0/*Velocidad*/, x2_est_print, angulo, x3_est_print, gyro_x, x4_est_print);

  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}

float state_feedback_sin_feed(x1_est, x2_est, x3_est, x4_est){
  float u = 0;
  float K[4] = {-3.9340, -0.9499, -0.8737, -0.0781};

  u = x1_est*K[0] + x2_est*K[1] + x3_est*K[2] + x4_est*K[3];

  return u
}


float observador(float u, float x1_med, float x3_med){
  // Matrices discretizadas
  const float Ad[4][4] =  {{1, 0.02, 0, 0},
                          {0, 0.9238, 0.1756, 0},
                          {0, 0, 1.0000, 0.0200},
                          {0, 0, -3.0896, 0.5852}};

  const float Bd[4] = {0, 0, 0, 1.2978}; 

  const float Ld[4][2] = {{1.1349, 0},
                         {13.9806 , 0.1756},
                         {0, 0.7963},
                         {0, -1.3004}};

  static float x1_est = 0, x2_est = 0, x3_est = 0, x4_est = 0;

  //Cambio de notacion
  float y1_med = x1_med;
  float y2_med = x3_med;
  float y1_est = x1_est;
  float y2_est = x3_est;

  x1_est =  Ad[0][0]*x1_est + Ad[0][1]*x2_est + Ad[0][2]*x3_est + Ad[0][3]*x4_est + Ld[0][0] * (y1_med - y1_est) + Ld[0][1] * (y2_med - y2_est) + Bd[0] * u;
  x2_est =  Ad[1][0]*x1_est + Ad[1][1]*x2_est + Ad[1][2]*x3_est + Ad[1][3]*x4_est + Ld[1][0] * (y1_med - y1_est) + Ld[1][1] * (y2_med - y2_est) + Bd[1] * u;
  x3_est =  Ad[2][0]*x1_est + Ad[2][1]*x2_est + Ad[2][2]*x3_est + Ad[2][3]*x4_est + Ld[2][0] * (y1_med - y1_est) + Ld[2][1] * (y2_med - y2_est) + Bd[2] * u;
  x4_est =  Ad[3][0]*x1_est + Ad[3][1]*x2_est + Ad[3][2]*x3_est + Ad[3][3]*x4_est + Ld[3][0] * (y1_med - y1_est) + Ld[3][1] * (y2_med - y2_est) + Bd[3] * u;

  //Variables de ploteo 
  x1_est_print = x1_est;
  x2_est_print = x2_est;
  x3_est_print = x3_est;
  x4_est_print = x4_est;

  return 0;
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

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8) {
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
  b = (byte *)&dato8;
  Serial.write(b, 4);
  }


float estimar_angulo_gyro(float gyro, float angulo_prev) {
  return (angulo_prev + gyro * DELTA);
}

float estimar_angulo_accel(float accel_z, float accel_y) {
  float anguloRadianes = atan2(accel_y, accel_z);
  return anguloRadianes;
}