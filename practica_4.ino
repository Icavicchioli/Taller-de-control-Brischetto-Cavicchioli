// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h> 
#define DELTA 0.02
#define ALFA 0.05

Adafruit_MPU6050 mpu;
Servo myservo;

void mover_servo(float grados);
float estimar_angulo_gyro(float gyro, float angulo_prev);
float estimar_angulo_accel(float accel_z, float accel_y);
float estimar_angulo(float angulo_prev);

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

  delay(100);


  //Init servo
  myservo.attach(9, 500, 2500); //500 -> 0, 2500->180
  Serial.begin(115200);

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
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1-ALFA) * angulo_gyro + ALFA*angulo_accel;

  Serial.println(angulo*180.0/3.14);

  #define E0 5.47
  #define E1 -9.005
  #define E2 3.706
  #define U1 0.9091
  #define U2 0.09091

  //Calculo el error
  error = ref - (-angulo) * 180.0/3.14; //meto el menos del angulo aca

  //Accion de control
  u = E0 * error + E1 * error1 + E2 * error2 + U1 * u1 + U2 * u2; 

  //Actualizacion de errores
  error2 = error1;
  error1 = error;
  u2 = u1;
  u1 = u;
  
  //limitar angulo
  mover_servo(u);
  
  //Datos a matlab
  //matlab_send(error,  error1,  u,  u1, angulo);

  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;
  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}

void mover_servo(float grados){
  float g = grados;
  if(g > 30) g = 30;
  if(g < -30) g = -30;

  float pwm = 0;
  pwm = (g*2000.0/180) + 1500;
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


float estimar_angulo_gyro(float gyro, float angulo_prev){
  return (angulo_prev+gyro*DELTA);

}

float estimar_angulo_accel(float accel_z, float accel_y){
  float anguloRadianes = atan2(accel_y,accel_z);
  return anguloRadianes;
}

float estimar_angulo(float angulo_prev){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  float angulo_gyro = estimar_angulo_gyro(gyro_x, angulo_prev);
  float angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  return (1-ALFA) * angulo_gyro + ALFA*angulo_accel;
  
}
