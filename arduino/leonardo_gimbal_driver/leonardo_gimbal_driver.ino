/* driver for leonardo gimbal */
#include <Arduino.h>
#include <Servo.h> 

#define PITCH_PWM_PIN 5
#define ROLL_PWM_PIN 6
#define LOOP_PERIOD 20 // [ms] 20 ms ->50 Hz

Servo servo_pitch;
Servo servo_roll;
int roll_home = 1500;
int roll_max = 2000;
int roll_min = 1000;
int pitch_home = 1500;
int pitch_max = 2000;
int pitch_min = 1000;


uint8_t c;
uint8_t buffer[5]; //16bit roll 16 bit pithc 1 termination = 5 bytes

uint16_t roll_sp;
uint16_t pitch_sp;


void setup() {
  Serial.begin(115200);
  while(!Serial);
  servo_roll.attach(ROLL_PWM_PIN);
  servo_pitch.attach(PITCH_PWM_PIN);
  servo_roll.writeMicroseconds(roll_home);
  servo_pitch.writeMicroseconds(pitch_home);
  roll_sp = roll_home;
  pitch_sp = pitch_home;

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    c = Serial.read();
    if(c == '$'){
      Serial.readBytes(buffer, 5);
      if(buffer[4]=='%'){ // packet ok
        roll_sp = 0x00;
        pitch_sp = 0x00;
        roll_sp |=buffer[0]<<8;
        roll_sp |=buffer[1];
        pitch_sp |=buffer[2]<<8;
        pitch_sp |=buffer[3];
      }
    }
    
  }

  servo_roll.writeMicroseconds(roll_sp);
  servo_pitch.writeMicroseconds(pitch_sp);

  delay(LOOP_PERIOD);
  
}
