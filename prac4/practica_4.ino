// Basic demo for accelerometer readings from Adafruit MPU6050

//#include <Adafruit_ADXL345_U.h>
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

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens


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

  //Init servo
  myservo.attach(9, 500, 2500);  //500 -> 0, 2500->180
  mover_servo(0);


  delay(1000);
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


  unsigned long t1 = micros();


  //Sensado
  //sensors_event_t event;
  //accel.getEvent(&event);

  //paso a radianes para procesar
  //angulo = -atan2(event.acceleration.x, event.acceleration.z) * 180.0 / PI;;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel; // en radianes

    //coefs tustin
    #define E0 1.737
    #define E1 -1.433
    #define U1 1

  error = ref +(angulo * 180/3.1415); //asi la referencia puede ser en grados

  //Accion de control
  //u = (E0 * error + E1 * error1 + U1 * u1);


  // control foh
  //u = (1.737 * error + (-1.433) * error1 + 1 * u1);

  // control boh obtenido a manopla
  u = (1.8886 * error + (-1.585) * error1 + 1 * u1);


  // saturadores en las variables internas
  if (u > MAX_ABS) u = MAX_ABS;
  if (u < -MAX_ABS) u = -MAX_ABS;

  if (error > MAX_ABS) error = MAX_ABS;
  if (error < -MAX_ABS) error = -MAX_ABS;


  //Actualzicion de errores
  error2 = error1;
  error1 = error;
  u2 = u1;
  u1 = u;

  Serial.println(u);
  Serial.println(error);

  mover_servo(u);

  //Datos
  //matlab_send( error,  error1,  u,  u1, angulo);

  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}

void mover_servo(float grados) {

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
