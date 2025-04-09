/*
Author  : Andrea Lombardo
Site    : https://www.lombardoandrea.com
Source  : https://github.com/AndreaLombardo/L298N/

Here you can see how to work in a common configuration.

L298NX2 is not a new version of module or IC, 
but it stands for a double implementation of L298N library.

With L298NX2 is possible to initialize two motors at once.

Speed range go from 0 to 255, default is 100.
Use setSpeed(speed) to change speed for both motors,
setSpeedA(speed) or setSpeedB(speed) for individual changes.

Sometimes at lower speed motors seems not running.
It's normal, may depends by motor and power supply.

Wiring schema in file "L298NX2 - Schema_with_EN_pin.png"
*/

#include <L298NX2.h>

// Pin definitions
const unsigned int EN_A = 11;
const unsigned int IN1_A = 10;
const unsigned int IN2_A = 9;

const unsigned int EN_B = 3;
const unsigned int IN1_B = 6;
const unsigned int IN2_B = 5;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

void setup() {
    Serial.begin(9600);  
    while (!Serial);  // Wait for Serial Monitor

    motors.setSpeed(200);  // Set initial speed
}

void loop() {
    moveForward(4000);  // Move forward for 3 sec
    stopMotors(2000);   // Stop for 2 sec
    moveBackward(4000); // Move backward for 3 sec
    stopMotors(2000);   // Stop for 2 sec
    turnRight(1500);    // Turn right for 1.5 sec
    stopMotors(2000);   // Stop for 2 sec
    turnLeft(1500);
    stopMotors(2000);
}

// ============================
// Motor Control Functions
// ============================

// Move forward
void moveForward(int duration) {
    motors.forward();
    printMotorStatus();
    delay(duration);
}

// Move backward
void moveBackward(int duration) {
    motors.runA(L298N::BACKWARD);
    motors.runB(L298N::BACKWARD);
    printMotorStatus();
    delay(duration);
}

// Turn right (pivot by slowing down left motor)
// Turn right by spinning wheels in opposite directions
void turnRight(int duration) {
    motors.runA(L298N::FORWARD);   // Left motor forward
    motors.runB(L298N::BACKWARD);  // Right motor backward
    motors.setSpeedA(200);         // You can tweak these to fine-tune slipping
    motors.setSpeedB(200);
    
    printMotorStatus();
    delay(duration);
}


void turnLeft(int duration) {
    motors.runA(L298N::BACKWARD);   // Left motor backward
    motors.runB(L298N::FORWARD);    // Right motor forward
    motors.setSpeedA(200);
    motors.setSpeedB(200);

    printMotorStatus();
    delay(duration);
}


// Stop motors
void stopMotors(int duration) {
    motors.stop();
    printMotorStatus();
    delay(duration);
}

// Print motor status to Serial Monitor
void printMotorStatus() {
    Serial.print("Motor A: ");
    Serial.print(motors.isMovingA() ? "Moving" : "Stopped");
    Serial.print(" | Speed: ");
    Serial.println(motors.getSpeedA());

    Serial.print("Motor B: ");
    Serial.print(motors.isMovingB() ? "Moving" : "Stopped");
    Serial.print(" | Speed: ");
    Serial.println(motors.getSpeedB());
}
