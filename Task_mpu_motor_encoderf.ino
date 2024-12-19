#include "Wire.h"
#include <MPU6050_light.h>
#include <LiquidCrystal.h>

const int rs = 8, en = 13, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Define motor driver pins
const int motorPin1 = 5;  // IN1 on motor driver
const int motorPin2 = 4;  // IN2 on motor driver
const int enablePin = 6;  // ENA on motor driver (PWM control)

// Define Encoder pins
const int EncoderPin1 = 2; // Encoder A
const int EncoderPin2 = 3; // Encoder B

// Motor control variables
int motorSpeed = 0;        // Motor speed (0-255 for PWM)
int motorDirection = 0;    // Motor direction (1 for clockwise, -1 for counterclockwise, 0 for stop)
volatile int encoderPos = 0; // Encoder position (counts)
float mpuAngle = 0;

MPU6050 mpu(Wire);
unsigned long timer = 0;

// Interrupt function to track encoder position
void encoderISR() {
    int val = digitalRead(EncoderPin2);
    encoderPos += (val == 0) ? 1 : -1;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    //while (status != 0) { } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(); // gyro and accelero

    lcd.begin(16, 2); // Initialize LCD with 16x2 dimensions

    // Set motor control pins as outputs
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Set encoder pins as inputs
    pinMode(EncoderPin1, INPUT_PULLUP);
    pinMode(EncoderPin2, INPUT_PULLUP);

    // Attach interrupt to encoder pin to detect changes
    attachInterrupt(digitalPinToInterrupt(EncoderPin1), encoderISR, RISING);
    Serial.println("Done!\n");
}

void loop() {
    mpu.update();
    mpuAngle = mpu.getAngleY();
    Serial.print("\tY : ");
    Serial.print(mpuAngle);
    //timer = millis();

    // Calculate the desired encoder position based on the MPU angle
    int desiredPos = (mpuAngle / 360.0) * 66;
    Serial.println(desiredPos);

    // Calculate the error between the desired position and the current position
    int error = desiredPos - encoderPos;
    Serial.println(error);
    

// Set motor direction and speed based on the error
  if (error != 0 && abs(error) >= 5 ) {
      motorDirection = (error > 0) ? 1 : -1;
    // Set motor direction based on the error
      digitalWrite(motorPin1, motorDirection == 1 ? HIGH : LOW);
      digitalWrite(motorPin2, motorDirection == -1 ? HIGH : LOW);
      analogWrite(enablePin, abs(error));
  }

   if (encoderPos >= desiredPos -8 && encoderPos <= desiredPos +8) {
       digitalWrite(motorPin1, LOW);
       digitalWrite(motorPin2, LOW);
       analogWrite(enablePin, 0);
          
     }
    Serial.print("\tDirection: ");
    Serial.print(motorDirection == 1 ? "Clockwise" : (motorDirection == -1 ? "Counterclockwise" : "Stopped"));
    Serial.print("\tEncoder Position: ");
    Serial.print(encoderPos);
    Serial.print("\tDesired Position: ");
    Serial.println(desiredPos);

    // Display on LCD
    lcd.setCursor(0, 0); // Set to first row
    lcd.print("Y:");
    lcd.print(mpuAngle);
    lcd.setCursor(0, 1); // Set to second row
    lcd.print(motorDirection == 1 ? "CW " : (motorDirection == -1 ? "CCW" : "Stop"));
    lcd.print("  pos:");
    lcd.print(encoderPos);

    // Small delay to avoid too fast serial output
    delay(100);
}