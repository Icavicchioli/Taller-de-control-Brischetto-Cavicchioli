// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#define DELTA 0.02
#define ALFA 0.05

Adafruit_MPU6050 mpu;


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Serial.println("");
  delay(100);
}

float estimar_angulo_gyro(float gyro, float angulo_prev);

float estimar_angulo_accel(float accel_z, float accel_y);

void loop() {

  static float angulo = 0;
  static float angulo_gyro = 0;
  static float angulo_accel = 0;
  static float angulo_gyro_diverge = 0;


  unsigned long t1 = micros();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro_diverge = estimar_angulo_gyro(gyro_x,angulo_gyro_diverge);
  angulo_gyro = estimar_angulo_gyro(gyro_x,angulo);

  angulo_accel = estimar_angulo_accel(accel_z,accel_y);

  angulo = (1-ALFA)* angulo_gyro + ALFA*angulo_accel,

  matlab_send(angulo,angulo_gyro,angulo_gyro_diverge,angulo_accel);

  unsigned long t2 = micros();

  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}


void matlab_send(float dato1, float dato2, float dato3, float dato4) {
  Serial.write("abcd");
  byte *b = (byte *)&dato1;
  Serial.write(b, 4);
  b = (byte *)&dato2;
  Serial.write(b, 4);
  b = (byte *)&dato3;
  Serial.write(b, 4);
  b = (byte *)&dato4;
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

