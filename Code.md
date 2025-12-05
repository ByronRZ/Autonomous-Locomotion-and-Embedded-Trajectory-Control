---
layout: default
title: Code
nav_order: 3
---

# Design of Autonomous Locomotion and Embedded Trajectory Control

```cpp
#include <ArduinoBLE.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "mbed.h"
#include <PID_v1.h>

LSM6DS3 imu(I2C_MODE, 0x6A);
using namespace mbed;

// BLE UUIDs
BLEService miServicio("12345678-1234-5678-1234-56789abcdef0");
BLEIntCharacteristic miCaracteristica("abcdef01-1234-5678-1234-56789abcdef0", BLERead | BLENotify);

// PWM en pines
PwmOut pwm_A(digitalPinToPinName(0));
PwmOut pwm_B(digitalPinToPinName(2));
int salir=0;
// Variables yaw y filtros
float yaw = 0;
float gzOffset = 0.0;
//float deadband = 0.5;
float deadband = 2;//9
unsigned long lastUpdate = 0;

// Variables del filtro Kalman
float x_est_last = 0;
float P_last = 1;
const float Q = 0.01;//0.01
const float R = 100;//100
float Kalm = 0;
float P = 0;
float x_est = 0;

// PID
double setpoint = 0;   // Queremos que yaw sea 0 (recto)
double input, output;
double Kp = 700, Ki = 0, Kd = 80;   // Ajustar en pruebas
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Frecuencias base
const float FRECUENCIA_BASE = 20000;   // Base para ambos
const float AJUSTE_MAXIMO_A = 3000;    // Hasta 37500 (40000 - 2500)
const float AJUSTE_MAXIMO_B = 3000;    // Hasta 37000 (40000 - 3000)
float FrecuenciaA=0;
float FrecuenciaB=0;

// Duty Cycle fijo
float DC_A = 0.7;
float DC_B = 0.7;

// ----------- FUNCIONES -----------

void calibrarGiroscopio() {
  Serial.println("Calibrando giroscopio... Mantén el módulo quieto");
  delay(2000);

  float sum = 0;
  int muestras = 1000;
  for (int i = 0; i < muestras; i++) {
    sum += imu.readFloatGyroZ();
    delay(3);
  }
  gzOffset = sum / muestras;
  Serial.print("Offset de giroscopio Z = ");
  Serial.println(gzOffset);
}

float kalmanFilter(float medida) {
  P = P_last + Q;
  Kalm = P / (P + R);
  x_est = x_est_last + Kalm * (medida - x_est_last);
  P = (1 - Kalm) * P;
  P_last = P;
  x_est_last = x_est;
  return x_est;
}

// ----------- SETUP -----------

void setup() {
  Serial.begin(115200);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  
  // BLE
  if (!BLE.begin()) {
    Serial.println("BLE no inició");
    while (1);
  }
  BLE.setLocalName("XIAO_BLE_20u");
  BLE.setAdvertisedService(miServicio);
  miServicio.addCharacteristic(miCaracteristica);
  BLE.addService(miServicio);
  BLE.advertise();

  // IMU
  if (imu.begin() != 0) {
    Serial.println("Error: No se pudo iniciar el LSM6DS3");
    while (1);
  }
  
  Serial.println("IMU LSM6DS3 inicializada");
  delay(4000);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, LOW);
  calibrarGiroscopio();
  digitalWrite(LED_RED, HIGH);
  lastUpdate = millis();

  // PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-10000, 10000);  // Ajuste total máximo
}

// ----------- LOOP -----------

void loop() {
  BLE.poll();
  digitalWrite(LED_GREEN, LOW);

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  // Leer giroscopio Z y filtrar
  float gz = imu.readFloatGyroZ();
  gz = kalmanFilter(gz);
  gz -= gzOffset;

  if (abs(gz) < deadband) {
    gz = 0;
  }

  // Integrar yaw
  yaw += gz * dt;
  if (yaw > 180) yaw = 180 ;
  if (yaw < -180) yaw = -180;

  // PID
  input = yaw;
  pid.Compute();
  float frecuenciaA = FRECUENCIA_BASE;
  if (output > 0)
    frecuenciaA -= constrain(output, 0, AJUSTE_MAXIMO_A);
    
  else if (output < 0)
    frecuenciaA += constrain(abs(output), 0, AJUSTE_MAXIMO_B);



  pwm_A.period(1.0 / 20000);
  pwm_B.period(1.0 / frecuenciaA);
  pwm_A.write(0.7);
  pwm_B.write(0.7);


  // Frecuencias
   // if (yaw >-1 && yaw <1) {
    //FrecuenciaA=20000;
    //FrecuenciaB=20000;
    //DC_A = 0.28;
    //DC_B = 0.7;
  //} 
 //if (yaw<=-1) {
   //FrecuenciaA=22000;
   //FrecuenciaB=28000;
   //DC_A = 0.69;
    //DC_B = 0.9;
  //} 
  //if (yaw>=1) {
    //FrecuenciaA=41000;
    //FrecuenciaB=20000;
    //DC_A = 0.69;
    //DC_B = 0.4;
  //}
  //pwm_A.period(1.0 / FrecuenciaA);
 // pwm_B.period(1.0 / FrecuenciaB);
  //pwm_A.write(DC_A);
  //pwm_B.write(DC_B);
  // Monitor
//Serial.print("Yaw: ");
  Serial.print(yaw, 1);
 // Serial.print("° | PID: ");
  //Serial.print(output);
    //Serial.print(" | Freq A: ");
 Serial.print(",");
 Serial.print(frecuenciaA);
    //Serial.print(" | Freq B: ");

  Serial.print(",");
  Serial.print(15000);
  Serial.print(",");
  Serial.println(output);

  miCaracteristica.writeValue(yaw);

  delay(10); // 50 Hz
}
