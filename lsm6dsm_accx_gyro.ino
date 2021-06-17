#include "deneyap.h"
#include "lsm6dsm.h"
#include "ServoESP32.h"

// (!) Motorlar ters ise 1 olan degeri -1 yap (!)   
//************************************************
float a = 1; //Sag aileron                    // *
float b = 1; //Sol aileron                    // *
float c = 1; //Elevator                       // *
float d = 1; //Rudder                         // *
//************************************************

// Gyro degerlerinde sapma varsa, asagidaki degerleri 
// pozitif ve negatif olarak buyultup kucultebilirsiniz   
//*******************************************************
float e = 0; //Sag aileron                           // *
float f = 0; //Sol aileron                           // *
float g = 0; //Elevator                              // *
float h = 0; //Rudder                                // *
//******************************************************* 

int servo_sag_aileron_pin = D0;  // Sag Alieron servo motoru
int servo_sol_aileron_pin = D1;  // Sol Alieorn servo motoru
int servo_elevator_pin = D4;     // Elevator servo motoru
int servo_rudder_pin = D8;       // Rudder servo motoru

float IMU_sag_aileron, IMU_sol_aileron, IMU_elevator, IMU_rudder;

float gyroz;
float norm_gyroz;
float IMU_rudder_raw = 0;
float tstep = 0.01;

int16_t sag_aileron, sol_aileron, elevator, rudder;

unsigned long timer = 0;

LSM6DSM IMU;

Servo servo_sag_aileron;
Servo servo_sol_aileron;
Servo servo_elevator;
Servo servo_rudder;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Seri port açıldı");
  
  servo_sag_aileron.attach(servo_sag_aileron_pin);
  servo_sol_aileron.attach(servo_sol_aileron_pin);
  servo_elevator.attach(servo_elevator_pin);
  servo_rudder.attach(servo_rudder_pin);
  Serial.println("Servolar aktif edildi");

  IMU.begin();
  Serial.println("IMU devrede");
  
  delay (100);
}

void loop() {

  timer = millis();

  IMU_sag_aileron = IMU.readFloatAccelX() * -100 * a;
  IMU_sol_aileron = IMU.readFloatAccelX() * -100 * b; 
  IMU_elevator = IMU.readFloatAccelY() * 100 * c;

  gyroz = IMU.readFloatGyroZ();
  norm_gyroz = gyroz * 0.007633f;
  IMU_rudder_raw = IMU_rudder_raw + norm_gyroz * tstep;
  IMU_rudder = IMU_rudder_raw * 100 * d;

  Serial.print("*********************");
  Serial.println();

  Serial.print(" Sag Aileron = ");
  Serial.println(IMU_sag_aileron);
  Serial.print(" Sol Aileron = ");
  Serial.println(IMU_sol_aileron);
  Serial.print(" Elevator =    ");
  Serial.println(IMU_elevator);
  Serial.print(" Rudder =     ");
  Serial.println(IMU_rudder);

  sag_aileron = map(IMU_sag_aileron + e, 100, -100, 0, 180);
  sol_aileron = map(IMU_sol_aileron + f, 100, -100, 0, 180);
  elevator = map(IMU_elevator + g, 100, -100, 0, 180);
  rudder = map(IMU_rudder + h, 80, -80, 0, 180);

  servo_sag_aileron.write(sag_aileron);
  servo_sol_aileron.write(sol_aileron);
  servo_elevator.write(elevator);
  servo_rudder.write(rudder);

  delay((tstep * 1000) - (millis() - timer));
}
