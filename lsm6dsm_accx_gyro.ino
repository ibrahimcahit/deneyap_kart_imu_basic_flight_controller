#include "deneyap.h"
#include "lsm6dsm.h"
#include "ServoESP32.h"

#define delayms 300

LSM6DSM IMU;


int servoXa_pin = D0;  // Sağ Alieron
int servoXb_pin = D1;  // Sol Alieorn
int servoY_pin = D4;   // Elevator
int servoZ_pin = D8;   // Rudder

int16_t axA,axB, ay, az;

float gyroz;
float norm_gyroz;
float yaw = 0;
float tstep = 0.01;

unsigned long timer = 0;

Servo servoXa;
Servo servoXb;
Servo servoY;
Servo servoZ;

void setup ( ) {
  
  servoXa.attach (servoXa_pin);
  servoXb.attach (servoXb_pin);
  servoY.attach (servoY_pin);
  servoZ.attach (servoZ_pin);

  Serial.begin(115200);
  IMU.begin();

  delay (100);
  
}

void loop ( ) {

  timer = millis();

  gyroz = IMU.readFloatGyroZ();
  norm_gyroz = gyroz * 0.007633f;
  yaw = yaw + norm_gyroz * tstep;

  axA = map (IMU.readFloatAccelX() * -100, -100, 100, 0, 180);
  axB = map (IMU.readFloatAccelX() * -100, -100, 100, 0, 180);
  ay = map (IMU.readFloatAccelY() * 100, -100, 100, 0, 180);
  
  az = map (yaw, -100, 100, 0, 180); 

  servoXa.write (axA);
  servoXb.write (axB);
  servoY.write (ay);
  servoZ.write (az);

  delay((tstep * 1000) - (millis() - timer));

}
