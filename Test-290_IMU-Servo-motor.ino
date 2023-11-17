#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <Servo.h>

#include "Wire.h"
#include <MPU6050_light.h>

#define YAW_THRESHOLD 70.0
#define SERVO_MIN_ANGLE 20
#define SERVO_MAX_ANGLE 150

#define PWM_PIN 11 // Pin 11 (D3) is PD7 for PWM control

MPU6050 mpu(Wire);
unsigned long timer = 0;
float yaw;
Servo myservo;
int pwmValue;
float xAcceleration; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  myservo.attach(9);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // Stop if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  _delay_ms(1000);
  mpu.calcOffsets(true, true); // Gyro and accelerometer
  Serial.println("Done!\n");

  // Configure Timer/Counter 2 for PWM on pin 11 (D3)
  //TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM, non-inverted mode
  //TCCR2B = (1 << CS20); // No prescaler (full clock speed)
  pinMode(PWM_PIN, OUTPUT); // Set the PWM control pin as an output

  OCR2A = 0; // Initialize the PWM duty cycle to 0
}

void loop() {
  mpu.update();

  // Control D3's brightness based on X-axis acceleration
   xAcceleration =100*mpu.getAccX();
  if(xAcceleration<0){
  pwmValue = map(xAcceleration, -100, 0, 0, 255);
  }else{
pwmValue = map(xAcceleration, 100, 0, 0, 255);

  }
 // Serial.println(xAcceleration);
//Serial.println(pwmValue);
  analogWrite(PWM_PIN,pwmValue);
  //pwmValue = constrain(pwmValue, 0, 255);
  //OCR2A = pwmValue; // Update PWM duty cycle on D3

  yaw = mpu.getAngleZ();
  Serial.print("Roll: ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tACCELERO  X: ");
  Serial.print(mpu.getAccX());
  Serial.print("\tPitch: ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tYaw: ");
  Serial.println(yaw);

  // Control the LED based on yaw angle
  if (yaw > YAW_THRESHOLD) {
    yaw = 70;
    PORTB |= (1 << PORTB5); // Turn on the LED
  } else if (yaw < -YAW_THRESHOLD) {
    yaw = -70;
    PORTB |= (1 << PORTB5); // Turn on the LED
  } else {
    PORTB &= ~(1 << PORTB5); // Turn off the LED
  }

//myservo.write(map(yaw, -180, 180, 180, 0)); //THIS MIGHT MAKE THE SERVO TURN THE CORRECT DIRECTION
  map(yaw, 180, -180, 180, 0); // Modify servo angle as needed
  myservo.write(yaw + 90); // Update the servo angle
  timer = millis();
}
