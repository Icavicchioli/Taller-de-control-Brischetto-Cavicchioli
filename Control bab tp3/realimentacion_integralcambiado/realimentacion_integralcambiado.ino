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

//Pruebas
#define CON_FF 1

//Definiciones motor
#define MOTOR_MAX 2500.0
#define MOTOR_MIN 500.0
#define MOTOR_CERO 1500.0

#define MOTOR_MAX 2250.0
#define MOTOR_MIN 800.0
#define MOTOR_CERO 1500.0

#define BTN2 PIN_A1
#define BTN1 4

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 100);
Servo myservo;
Adafruit_MPU6050 mpu;

//Prototipos
void mover_servo(float grados);
float estimar_angulo_accel(float accel_z, float accel_y);
void observador(float u, float x1_med, float x3_med);
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8, float dato9, float dato10);
float state_feedback_int(float ref_x1, float ref_x3, float x1_est, float x2_est, float x3_est, float x4_est);

static float secuencia_sigmoidal[40] = {
    0.002473, 0.003360, 0.004566, 0.006200, 0.008415, 0.011413,
    0.015461, 0.020915, 0.028237, 0.038024, 0.051025, 0.068155,
    0.090488, 0.119203, 0.155473, 0.200269, 0.254089, 0.316646,
    0.386621, 0.461614, 0.538386, 0.613379, 0.683354, 0.745911,
    0.799731, 0.844527, 0.880797, 0.909512, 0.931845, 0.948975,
    0.961976, 0.971763, 0.979085, 0.984539, 0.988587, 0.991585,
    0.993800, 0.995434, 0.996640, 0.997527
};

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
  myservo.attach(5, MOTOR_MIN, MOTOR_MAX);  //500 -> 0, 2500->180
  mover_servo(0);

  //Potenciometro
  pinMode(A0, INPUT);

  //boton de cambio rapido de referencia
  pinMode(BTN2, INPUT);
  pinMode(BTN1, INPUT);

  delay(1000);
}

//Variables globales
float x1_est_print = 0;
float x2_est_print = 0;
float x3_est_print = 0;
float x4_est_print = 0;
float q_print = 0;

void loop() {
  static float ref_x1 = 0;
  static float ref_x3 = 0;
  static float angulo = 0;
  static float angulo_gyro = 0;
  static float angulo_accel = 0;
  static float u = 0;
  float posicion = 0;
  static float cnt_ciclo = 0;

  static unsigned int index = 0;
  static float ref_sigmoidal = 0;
  static float ref_x1_ant = 0; 

  unsigned long t1 = micros();

  //Medición de sensores
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_z = a.acceleration.z;
  float accel_y = a.acceleration.y;
  float gyro_x = g.gyro.x;

  angulo_gyro = estimar_angulo_gyro(gyro_x, angulo);
  angulo_accel = estimar_angulo_accel(accel_z, accel_y);

  angulo = (1 - ALFA) * angulo_gyro + ALFA * angulo_accel;  // en radianes

  //Medición de posición
  posicion = (sonar.ping()) * (0.343 / 2.0) * (1.0 / 1000.0);  // microseg a m
  posicion -= 0.157;  //ajusto el cero

  // --- FILTRO DE MEDICIONES ---
  const float alpha = 1;  // Ajustar según nivel de ruido

  static float x1_filtrado = 0.0;
  static float x3_filtrado = 0.0;

  x1_filtrado = alpha * posicion + (1.0 - alpha) * x1_filtrado;
  x3_filtrado = alpha * angulo   + (1.0 - alpha) * x3_filtrado;
  // -----------------------------

  // Referencia
  #if CON_FF == 0
    if (!digitalRead(BTN2)) ref_x1 = 0.08;
    if (!digitalRead(BTN1)) ref_x1 = -0.08;
  #else
    cnt_ciclo++;

    if(cnt_ciclo == 1){ ref_x1 = 0.0;}
    if(cnt_ciclo == 400){ ref_x1 = 0.08;}
    if(cnt_ciclo == 800){ ref_x1 = -0.08;}
    if(cnt_ciclo == 1200){ cnt_ciclo = 399;}
  #endif

  static float ref_x1_filt = 0;
  const float vel_max_ref = 0.002;  // m por ciclo

  if (ref_x1_filt < ref_x1)
    ref_x1_filt += vel_max_ref;
  else if (ref_x1_filt > ref_x1)
    ref_x1_filt -= vel_max_ref;

  if(abs(ref_x1_filt - ref_x1) <= vel_max_ref)
    ref_x1_filt = ref_x1;

  ref_x3 = 0;

  // Controladores y observador con mediciones filtradas
  observador(u, x1_filtrado, x3_filtrado);
  u = state_feedback_int(ref_x1_filt, ref_x3, x1_est_print, x2_est_print, x3_est_print, x4_est_print);

  mover_servo(u);

  // Datos para MATLAB
  matlab_send(posicion, x1_est_print, 0, x2_est_print, angulo, x3_est_print, gyro_x, x4_est_print, ref_x1_filt, q_print);

  unsigned long t2 = micros();
  unsigned long diff = t2 - t1;
  unsigned long diffUS = diff % 1000;

  delay(20 - (diff - diffUS) / 1000);
  delayMicroseconds(diffUS);
}


float state_feedback_int(float ref_x1, float ref_x3, float x1_est, float x2_est, float x3_est, float x4_est) {
  const float Ts = 0.02;
  
  //const float K[4] = {-15.1705, -2.7130, -2.7658, -0.1514};
  //const float H = 18.7183;
  //const float K[4] = {-8.5638, -1.6764, -1.6533, -0.1083};
  //const float H = 7.7126;

  //const float K[4] = {-7.0207 ,  -1.4373 ,  -1.3023 ,-0.0885 }; //-2-1.5i; -2+1.5i ; -8; -11; -10
  //const float H = 6.9718;

  //const float K[4] = {-10.1792 ,  -2.0043  , -1.9859 ,  -0.1172 }; //-3-0.1i; -3+0.1i ; -8; -11; -10
  //const float H = 9.8540;

  //const float K[4] = {-16.3140  , -2.9647  , -3.0269 ,  -0.1589  }; //[-4+0.1j; -4-0.1j; -9; -11; -10]; este anda bastante bien (capaz un poco oscilatorio) (el que mejor anda)
  //const float H = 19.1261;

  //const float K[4] = {-10.7544,   -2.1502,   -2.2368 ,  -0.1299 }; // [-4; -2; -9; -11; -10]; //suele andar bien (a veces mucho overshoot)
  //const float H =   9.7482;

  //const float K[4] = {-13.6706 ,  -2.5726 ,  -2.6412 ,  -0.1446 }; // [-3.5+0.1j; -3.5-0.1j; -9; -11; -10] //A veces tiene overshoot
  //const float H =   14.7915;

  //const float K[4] = {-8.8458 ,  -1.9586,   -2.2900,   -0.1396}; //[-2+0.1j; -2-0.1j; -11; -11+2j; -11-2j] //mucho overshoot
  //const float H = 6.7236;

  //const float K[4] = {-8.6628  , -1.9161 ,  -2.2455 ,  -0.1388}; //[-2+0.1j; -2-0.1j; -11; -11+1j; -11-1j] //mucho overshoot
  //const float H = 6.5628;

  //const float K[4] = { -7.0827 ,  -1.6822 ,  -2.0273 ,  -0.1313}; //[-2.5; -1; -11; -11+1j; -11-1j]; //A veces anda bien, normalmente se va de overshoot
  //const float H = 4.1120;

  //const float K[4] = {-27.8709 ,  -4.6830  , -4.7623  , -0.2251}; //[-5-0.1j; -5+0.1j; -11; -11+1j; -11-1j]; //Sigue perfecta la referencia pero oscila un monton
  //const float H = 38.5752;

  //const float K[4] = {-22.4845  , -3.8519 ,  -3.8676 ,  -0.1892}; //[-5-0.1j; -5+0.1j; -10; -10.1; -10.05]; //Sigue bien la referencia pero oscila un monton
  //const float H = 29.9917;

  const float K[4] = { -19.4825, -3.4280, -3.4748, -0.1752 };  //[-4.5-0.1j; -4.5+0.1j; -10; -10.1; -10.05] //Este muy muy bien, oscila menos y sigue bien
  const float H = 24.5358;

  //const float K[4] = {-17.7544  , -3.1799  , -3.2402  , -0.1668 }; //[-4.2-0.1j; -4.2+0.1j; -10; -10.1; -10.05]; //Sigue bien, pero oscila y tiene un poco de error
  //const float H = 21.5018;

  //const float K[4] = {-14.0580  , -2.5603  , -2.4541  , -0.1305 }; //[-4.2-0.1j; -4.2+0.1j; -9.2; -9.1; -9] //Este ya no sigue muy bien y se suele pasar, oscila menos
  //const float H = 16.4074;

  //const float K[4] = {-15.4487, -2.7680, -2.6687, -0.1390}; //[-4.5-0.1j; -4.5+0.1j; -9.2; -9.1; -9] //Este ya no sigue muy bien y se suele pasar, oscila menos
  //const float H = 18.7227;

  //const float K[4] = {-15.7966 ,  -2.8056  , -2.6886  , -0.1392}; //[-4.5-1j; -4.5+1j; -9.2; -9.1; -9] // bastante overshoot y un poco lento
  //const float H = 19.6369;

  //const float K[4] = {-12.8955  , -2.2856  , -1.9441   ,-0.1012}; //[-4.5-2j; -4.5+2j; -8; -8.1; -8.2]  //Un poco de overshoot y no sigue bien
  //const float H = 16.2688;

  float u = 0.0;
  float e = 0.0;
  static float q = 0.0;

  //En backwards, ver de hacerlo de otra forma mejor
  e = ref_x1 - x1_est;  //rk - yk
  q = q + Ts * e;

  // Anti-windup
  const float q_max = 0.1*H;  // ajustar según K[4] y u_max -> qmax = umax / ganancia integrador
  if (q > q_max) q = q_max;
  if (q < -q_max) q = -q_max;


  //Nota: modificar la constante de la referencia nos permite modificar los ceros?
  u = x1_est * K[0] + x2_est * K[1] + x3_est * K[2] + x4_est * K[3] + H * q /*+ ref_x1*/;

  //Variables de plot
  q_print = q;

  return u;
}


float state_feedback(float ref_x1, float ref_x3, float x1_est, float x2_est, float x3_est, float x4_est) {
  float u = 0;
  float K[4] = { -3.9340, -0.9499, -0.8737, -0.0781 };
  float F[2] = { 3.9340, 0 };

  u = x1_est * K[0] + x2_est * K[1] + x3_est * K[2] + x4_est * K[3] + ref_x1 * F[0] + ref_x3 * F[1];

  return u;
}


void observador(float u, float x1_med, float x3_med) {
  // Matrices discretizadas
  const float Ad[4][4] = { { 1, 0.02, 0, 0 },
                           { 0, 0.9238, 0.1756, 0 },
                           { 0, 0, 1.0000, 0.0200 },
                           { 0, 0, -3.0896, 0.5852 } };

  const float Bd[4] = { 0, 0, 0, 1.2978 };

  /*
  //mas lento -[2.7 2.4 2.5 2.6] *12.4289
  const float Ld[4][2] = { { 0.8744, -0.0392 },
                           { 7.9568, -0.6022 },
                           { -0.0008, 0.5116 },
                           { -0.0020, -2.9808 } };
  */
  
/*
  // -[3.8 3.2 3 2.6] *12.4289 
  const float Ld[4][2] = {{1.0585, -0.0740},
                         {11.9768 , -1.5409},
                         {-0.0035, 0.6119},
                         {-0.0174, -2.6687}};
*/
  
  /*
  //intermedio  -[3.5 3.5 2.7 2.7] *12.4289 //Funciona
  const float Ld[4][2] = {{0.9938, 0},
                         {10.4178 , 0.1756},
                         {0, 0.6551},
                         {0, -2.4737}};
  */

/*
  // [-10 -10 (-10.37+1i)*2 (-10.37-1i)*2] //No sigue bien posicion
  const float Ld[4][2] = {
                          { 0.3619,  0.0986 },
                          { 0.9492,  0.6937 },
                          {-0.0773,  0.1897 },
                          { 0.9025, -3.1884 }
                        };
*/

/*
  // [-10 -10 (-10.37+1i) (-10.37-1i)] ; //No sigue bien posicion
  const float Ld[4][2] ={
  {  0.0329,  0.0119 },
  { -0.7798,  0.2380 },
  { -5.7065,  0.2136 },
  { 66.6324, -3.4675 }
  };
*/

  //rapido  -[4 4 3.5 3.5] *12.4289
  /*
  const float Ld[4][2] = {{1.1349, 0},
                         {13.9806 , 0.1756},
                         {0, 0.7963},
                         {0, -1.3004}};
  */

  /*
  // [-10 -12  -12+i -12-i] * 2 
  const float Ld[4][2] = {{0.63, 0.0534},
                         {4.075 , 1},
                         {-0.015, 0.374},
                         {-0.0017, -2.48}};
  */

  /*
  //[-3.81 -1 -10.37+1i -10.37-1i] * 3.5
  //Este funciona "bien" pero no tiene mucho sentido teorico y ademas es medio lento
  const float Ld[4][2] = {{0.4957, -0.1176},
                          {-0.1834, 0.2237},
                          {-0.0174, 0.3496},
                          {-0.2290, -3.9651}};
  */
  
  /*
   //Este es un poquito menos ruidoso
  //[-20 -20 (-10.37+1i)*3.5 (-10.37-1i)*3.5] 
  const float Ld[4][2] = {{0.8766, 0.0810},
                          {6.9320, 1.2021},
                          {-0.1524, 0.3264},
                          {0.6487, -3.0754}
                        };
  */

  /*
  //[-13 -12 (-10.37+1i)*2.5 (-10.37-1i)*2.5];
  const float Ld[4][2] = {
    {  0.5374, -0.2251 },
    {  2.2562, -1.4524 },
    { -0.0060,  0.2246 },
    { -0.0318, -2.9686 }
    };*/
  
  //[-13 -12 (-10.37+1i)*3.5 (-10.37-1i)*3.5]; 
  /*
  const float Ld[4][2] = {
    {  0.7591,  0.0840 },
    {  3.7429,  0.7520 },
    { -0.1440,  0.2268 },
    {  1.3403, -3.0692 }
    };
  */

/*
  /// [-25 -26 -27 -28]
  const float Ld[4][2] = {
  {  0.7396, -0.0975 },
  {  5.4932, -1.4320 },
  { -0.0008,  0.4144 },
  {  0.0002, -3.0938 }
  };
*/
  
  
  //[-30 -30 -25 -25]
 const float Ld[4][2] = {
    {  0.7685,  0.0000 },
    {  5.9489,  0.1756 },
    { -0.0000,  0.4299 },
    { -0.0000, -3.1284 }
  };
  

  //[-20 -20 -15 -15] //Por ser tan lento hace que a veces no termine donde debe
  /*
  const float Ld[4][2] = {
    {0.5127, 0.0000},
    {2.3193, 0.1756},
    {0.0000, 0.1741},
    {0.0000, -2.4272}
  };
  */

/* //[-20-3i -20+3i -15 -15]
  const float Ld[4][2] = {
    {0.4231, 0.0977},
    {1.4995, 1.0697},
    {-0.1009, 0.2661},
    {0.7853, -3.1432}
};
*/
  /*
  //[-23 -20 -18 -15] //Este un poquito mas rapido capaz va bien
  const float Ld[4][2] = {
      { 0.5127, -0.0043},
      { 2.3195,  0.1304},
      {-0.0009,  0.2562},
      { 0.0035, -2.8302}
  };
  */
  /*
  //[-32 -30 -28 -25]
  const float Ld[4][2] = {
    { 0.8075, -0.2035},
    { 6.6510, -3.6795},
    {-0.0031,  0.4477},
    {-0.0038, -3.0936}
};*/

  /*
  //[-32 -30 -28 -25]
  const float Ld[4][2] = {
    { 0.8487, -0.1518},
    { 7.3791, -2.9856},
    {-0.0017,  0.4812},
    {-0.0031, -3.0930}
  };
  */

/*
   const float Ld[4][2] ={
    { 0.8207,  0.0062},
    { 6.7770,  0.2734},
    { 0.0001,  0.4514},
    {-0.0002, -3.1514}
};
*/
/*
  //[-20 -20 (-12+3i) (-12-3i)] //El que nos recomendo claudio
  const float Ld[4][2] = {
    {  0.2391,  0.0195},
    { -1.1482,  0.4233},
    { -2.8024,  0.3589},
    { 11.9271, -3.2137}
};
*/
  static float x1_est = 0, x2_est = 0, x3_est = 0, x4_est = 0;

  //Cambio de notacion
  float y1_med = x1_med;
  float y2_med = x3_med;
  float y1_est = x1_est;
  float y2_est = x3_est;

  x1_est = Ad[0][0] * x1_est + Ad[0][1] * x2_est + Ad[0][2] * x3_est + Ad[0][3] * x4_est + Ld[0][0] * (y1_med - y1_est) + Ld[0][1] * (y2_med - y2_est) + Bd[0] * u;
  x2_est = Ad[1][0] * x1_est + Ad[1][1] * x2_est + Ad[1][2] * x3_est + Ad[1][3] * x4_est + Ld[1][0] * (y1_med - y1_est) + Ld[1][1] * (y2_med - y2_est) + Bd[1] * u;
  x3_est = Ad[2][0] * x1_est + Ad[2][1] * x2_est + Ad[2][2] * x3_est + Ad[2][3] * x4_est + Ld[2][0] * (y1_med - y1_est) + Ld[2][1] * (y2_med - y2_est) + Bd[2] * u;
  x4_est = Ad[3][0] * x1_est + Ad[3][1] * x2_est + Ad[3][2] * x3_est + Ad[3][3] * x4_est + Ld[3][0] * (y1_med - y1_est) + Ld[3][1] * (y2_med - y2_est) + Bd[3] * u;

  //Variables de ploteo
  x1_est_print = x1_est;
  x2_est_print = x2_est;
  x3_est_print = x3_est;
  x4_est_print = x4_est;

  // agregado de filtro de media móvil
  /*  
  const int N = 4;  // Orden del filtro de media móvil
  static float buffer_x1[N] = { 0 }, buffer_x2[N] = { 0 }, buffer_x3[N] = { 0 }, buffer_x4[N] = { 0 };
  static int idx = 0;

  // Guardar nuevos valores en el buffer circular
  buffer_x1[idx] = x1_est;
  buffer_x2[idx] = x2_est;
  buffer_x3[idx] = x3_est;
  buffer_x4[idx] = x4_est;
  idx = (idx + 1) % N;

  // Calcular promedios
  float sum_x1 = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
  for (int i = 0; i < N; i++) {
    sum_x1 += buffer_x1[i];
    sum_x2 += buffer_x2[i];
    sum_x3 += buffer_x3[i];
    sum_x4 += buffer_x4[i];
  }

  x1_est_print = sum_x1 / N;
  x2_est_print = sum_x2 / N;
  x3_est_print = sum_x3 / N;
  x4_est_print = sum_x4 / N;
  */

}


void mover_servo(float grados) {

  volatile float g = 0;

  g = grados;

  if (g > 0.7) {
    g = 0.7;
  }
  if (g < -0.5) {
    g = -0.5;
  }

  volatile float pwm = 0;
  pwm = (grados * (MOTOR_MAX - MOTOR_MIN) / PI) + MOTOR_CERO;

  //Checkeo de vuelta que no sobrepase +-45grados (parece que es algo de la alimentacion)

  if (isnan(pwm) || isinf(pwm)) {
    pwm = 1500;
  }

  if (pwm > 1900.0) {
    pwm = 1900.0;
  }
  if (pwm < 1054.0) {
    pwm = 1054.0;
  }

  myservo.writeMicroseconds((int)pwm);
}

void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8, float dato9, float dato10) {
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
  b = (byte *)&dato9;
  Serial.write(b, 4);
  b = (byte *)&dato10;
  Serial.write(b, 4);
}

float estimar_angulo_gyro(float gyro, float angulo_prev) {
  return (angulo_prev + gyro * DELTA);
}

float estimar_angulo_accel(float accel_z, float accel_y) {
  float anguloRadianes = atan2(accel_y, accel_z);
  return anguloRadianes;
}
