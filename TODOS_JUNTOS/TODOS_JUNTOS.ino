#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Math.h>
#include <ESP32Servo.h>
#include "HX711.h"

#define LED_DEBUG 2

// --------------------------------------------- ACELEROMETRO
float media_final=0;
float value =0;
int cont = 0;
float med_x = 0;
float med_y = 0;
float med_z = 0;
float med_z_inicial = 0;
float cond=0;
float soma =0;
Adafruit_MPU6050 mpu;

// --------------------------------------------- FIM DE CURSO
// const int fim_curso2 = 16;
const int fim_curso1 = 34;
// const int fim_curso3 = 17;
// const int fim_curso4 = 35;

// --------------------------------------------- SERVO MOTOR POTENCIOMETRO
#define SERVO1_PIN 12
#define POTENCIOMETER1_PIN 14
#define SERVO2_PIN 27
#define POTENCIOMETER2_PIN 26

Servo myServo1;
Servo myServo2;
int val = 0;
int num = 0;

// --------------------------------------------- MOTOR VIBRACAO
#define MOTOR_VIBRACAP_PIN 25

// --------------------------------------------- LEDS
#define LED1_PIN 15
#define LED2_PIN 2
#define LED3_PIN 4
#define LED4_PIN 13

// --------------------------------------------- CELULA CARGA
const int LOADCELL_DOUT_PIN = 19;
const int LOADCELL_SCK_PIN = 18;
HX711 scale;

// --------------------------------------------- SENSOR IR
const int sensor_ir = 23;

// --------------------------------------------- MOTOR PASSO
const int DIR = 32;
const int STEP = 33;
const int en = 5;
const int steps_per_rev = 1500;



// --------------------------------------------- SETUP ACELEROMETRO
void setup_accelerometer(void) {
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

// --------------------------------------------- FIM DE CURSO
void setup_fim_curso(void) {
  pinMode (fim_curso1, INPUT);
  delay(100);
}

// --------------------------------------------- SERVO MOTOR POTENCIOMETRO
void setup_servo_potenciometro(void ) {
  myServo1.setPeriodHertz(50); // Set PWM frequency to 50Hz (standard for servos)
  myServo1.attach(SERVO1_PIN, 500, 2400); // (pin, min pulse width, max pulse width in microseconds)
  myServo2.setPeriodHertz(50); // Set PWM frequency to 50Hz (standard for servos)
  myServo2.attach(SERVO2_PIN, 500, 2400); // (pin, min pulse width, max pulse width in microseconds)
  delay(100);
}

// --------------------------------------------- MOTOR VIBRACAO
void setup_motor_vibracao(void) {
  pinMode (MOTOR_VIBRACAP_PIN, OUTPUT);
  delay(100);
}

// --------------------------------------------- LEDS
void setup_leds(void) {
  pinMode(LED1_PIN, OUTPUT); 
  pinMode(LED2_PIN, OUTPUT); 
  pinMode(LED3_PIN, OUTPUT); 
  pinMode(LED4_PIN, OUTPUT); 
  delay(100);
}

// --------------------------------------------- CELULA CARGA
void setup_celula_carga(void) {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(100);
}

// --------------------------------------------- SENSOR IR
void setup_sensor_ir(void) {
  pinMode (sensor_ir, INPUT);
  delay(100);
}

// --------------------------------------------- MOTOR PASSO
void setup_motor_passo(void) {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(en, OUTPUT);
  delay(100);
}


// --------------------------------------------- SETUP MAIN --------------------------------------------- 
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  setup_accelerometer();
  setup_fim_curso();
  setup_servo_potenciometro();
  setup_motor_vibracao();  
  setup_sensor_ir();
  setup_motor_passo();
  setup_leds();
  setup_celula_carga();

  pinMode(LED_DEBUG, OUTPUT);
}

// --------------------------------------------- ACELEROMETRO
void acelerometro(void) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("------------------- ACELETROMETRO -------------------");

  Serial.print("X:");
  Serial.print(a.acceleration.x);
  Serial.println(",");
  
  Serial.print("Y:"); 
  Serial.print(a.acceleration.y);
  Serial.println(",");
  
  Serial.print("Z:"); 
  Serial.print(a.acceleration.z);
  Serial.println(",");

  med_z = a.acceleration.z;

  if (cont < 10) {
    cont += 1;
    med_z_inicial = a.acceleration.z;
  }

  if ((med_z > med_z_inicial + 1) || (med_z < med_z_inicial - 1)) {
    Serial.println("MEXEU!");
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
  }
  else {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
  }
}

// --------------------------------------------- FIM DE CURSO
void fim_curso(void) {
  Serial.println("------------------- FIM DE CURSO -------------------");

  int state = digitalRead(fim_curso1);
  Serial.println(state);
  if(state==LOW){
    Serial.println("Object Detected");
    // funcionamento_motor_vibracao_desligar();
    motor_passo();
    celula_carga();
  }
  else {
    Serial.println("All Clear");
    // funcionamento_motor_vibracao_ligar();
  }
}

// --------------------------------------------- SERVO MOTOR POTENCIOMETRO
void servo_motor_potenciometro(void) {
  Serial.println("------------------- SERVO MOTOR POTENCIOMETRO -------------------");

  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    myServo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    myServo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }
}

// --------------------------------------------- CELULA CARGA
void celula_carga(void) {
  Serial.println("------------------- CELULA CARGA -------------------");

  if (scale.is_ready()) {
    digitalWrite(LED3_PIN, HIGH);
    scale.set_scale();    
    Serial.println("Tare... remove any weights from the scale.");
    delay(2000);
    scale.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(2000);
    long reading = scale.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    digitalWrite(LED3_PIN, LOW);
  } 
  else {
    Serial.println("HX711 not found.");
  }
}

// --------------------------------------------- MOTOR VIBRACAO
void funcionamento_motor_vibracao_ligar(void) {
  Serial.println("------------------- MOTOR VIBRACAO LIGAR -------------------");

  digitalWrite(MOTOR_VIBRACAP_PIN, HIGH);
}

void funcionamento_motor_vibracao_desligar(void) {
  Serial.println("------------------- MOTOR VIBRACAO DESLIGAR -------------------");

  digitalWrite(MOTOR_VIBRACAP_PIN, LOW);
}

// --------------------------------------------- SENSOR IR
void funcionamento_sensor_ir(void) {
  Serial.println("------------------- SENSOR IR -------------------");

  if(digitalRead(sensor_ir) == LOW){
    Serial.println(digitalRead(sensor_ir));
    servo_motor_potenciometro();
  }
  else {
    Serial.println(digitalRead(sensor_ir));
  }
}

// --------------------------------------------- MOTOR PASSO
void motor_passo(void) {
  Serial.println("------------------- MOTOR PASSO -------------------");

  digitalWrite(en, LOW);
  digitalWrite(DIR, HIGH);
  Serial.println("Girando no sentido horário...");
  
  for(int i = 0; i<steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEP, LOW);
    delayMicroseconds(100);
  }
  delay(500); 
  
  digitalWrite(DIR, LOW);
  Serial.println("Girando no sentido anti-horário...");

  for(int i = 0; i<steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEP, LOW);
    delayMicroseconds(100);
  }
}

// --------------------------------------------- LOOP MAIN --------------------------------------------- 
void loop() {
  acelerometro();
  fim_curso();
  funcionamento_sensor_ir();

  // servo_motor_potenciometro();

  // --------------------------------------------- LEDS
  
  // digitalWrite(LED4_PIN, HIGH);
  
  delay(500);
}