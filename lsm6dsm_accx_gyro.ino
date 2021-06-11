// İbrahim  Özdemir ve Ozgür Bostan tarafından, 
// Deneyap Kart için yazılmıştır.
//
// GitHub: ibrahimcahit
//

// Gerekli kütüphaneleri import ediyoruz
#include "deneyap.h"
#include "lsm6dsm.h"
#include "ServoESP32.h"

// ms cinsinden delay süresi belirliyoruz
#define delayms 300

// IMU (İvmeölçer) sensörünü başlatmak için bir LSM6DSM class'ı tanımlıyoruz
LSM6DSM IMU;

// Servo pinlerini tanımlıyoruz
int servoXa_pin = D0;  // Sağ Alieron
int servoXb_pin = D1;  // Sol Alieorn
int servoY_pin = D4;   // Elevator
int servoZ_pin = D8;   // Rudder

// IMU tarafından okunup, Servo'lara gönderilecek değerlerin tutulacağı değişkenleri tanımlıyoruz
int16_t axA,axB, ay, az;

// Z ekseninden yaw parametrelerini tanımlamak için değerler tanımlıyoruz
float gyroz;
float norm_gyroz;
float yaw = 0;
float tstep = 0.01;

// Timer değişkeni, süre için
unsigned long timer = 0;

// Servolarımızı tanımlıyoruz
Servo servoXa;
Servo servoXb;
Servo servoY;
Servo servoZ;

void setup ( ) {

  // Servolarımızın pinlerini tanımlıyoruz
  servoXa.attach (servoXa_pin);
  servoXb.attach (servoXb_pin);
  servoY.attach (servoY_pin);
  servoZ.attach (servoZ_pin);

  // IMU sensörümüzü başlatıyoruz. IMU, I2C kullanıyor. 
  IMU.begin();

  delay (100);
  
}

void loop ( ) {

  timer = millis();

  // Z eksenindeki Gyro değerini okuyoruz
  gyroz = IMU.readFloatGyroZ();

  // Gyro değerininin zamana göre değişimini bulmamız gerek. Sabit ile çarpıp, normal istediğimiz değerini buluyoruz. 
  norm_gyroz = gyroz * 0.007633f;

  // Yaw değerimizi hesaplıyoruz
  yaw = yaw + norm_gyroz * tstep;

  // X ve Y eksenlerindeki hareketleri (Elevator ve Alieron)ivmelçer ile direkt kontrol edebiliriz. 
  // İvme ölçerin verdiği değerleri 100 ile çarpıp, 0 - 180 aralığında map ediyoruz  
  axA = map (IMU.readFloatAccelX() * -100, -100, 100, 0, 180);
  axB = map (IMU.readFloatAccelX() * -100, -100, 100, 0, 180);
  ay = map (IMU.readFloatAccelY() * 100, -100, 100, 0, 180);
  
  az = map (yaw, -100, 100, 0, 180); 

  // Değerleri servolara yazıyoruz
  servoXa.write (axA);
  servoXb.write (axB);
  servoY.write (ay);
  servoZ.write (az);

  delay((tstep * 1000) - (millis() - timer));

}
