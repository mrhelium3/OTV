#include "Enes100.h"
#include <L298NX2.h>

//#define LEFT_MOTOR_FORWARD 5  // Adjust based on wiring
//#define LEFT_MOTOR_BACKWARD 6
//#define RIGHT_MOTOR_FORWARD 9
//#define RIGHT_MOTOR_BACKWARD 10
//#define MOTOR_SPEED 150  // Adjust for speed control (0-255)
const int EN_A = 3;
const int IN1_A = 5;
const int IN2_A = 6;

const int EN_B = 9;
const int IN1_B = 8;
const int IN2_B = 7;

#define FRONT_TRIG_PIN 14  // A0
#define FRONT_ECHO_PIN 15  // A1
#define SIDE_TRIG_PIN 16   // A2
#define SIDE_ECHO_PIN 17   // A3

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);


// Function prototypes
void navigateTo(float targetX, float targetY, float targetTheta);
void rotateToAngle(float targetTheta);
void moveToPosition(float targetX, float targetY);
//void setMotorSpeed(int leftSpeed, int rightSpeed); //motor speed are set in void setup loop
void stopMotors();

void setup() {
    // Initialize communication with the Vision System
    Enes100.begin("TeamName", FIRE, 3, 8, 9);

    Serial.begin(9600);  
    while (!Serial);  // Wait for Serial Monitor

    motors.setSpeed(200);  // Set initial speed
    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(SIDE_TRIG_PIN, OUTPUT);
    pinMode(SIDE_ECHO_PIN, INPUT);
    
    // Print initial message to confirm setup is complete
    Enes100.println("OTV setup complete");
    Serial.println("OTV setup complete - Ready for mission");
}

void loop() {
    // Example: Move to position (50, 30) with final orientation 90 degrees
    navigateTo(50, 30, 90);
    delay(5000);  // Wait before next movement
}

// Function to navigate to (x, y) and orient to theta
void navigateTo(float targetX, float targetY, float targetTheta) {
    rotateToAngle(atan2(targetY - Enes100.getY(), targetX - Enes100.getX()) * 180 / PI);
    moveToPosition(targetX, targetY);
    rotateToAngle(targetTheta);
}

// Function to rotate the robot to a target angle
void rotateToAngle(float targetTheta) {
    while (true) {
        float currentTheta = Enes100.getTheta();
        float error = targetTheta - currentTheta;

        // Normalize error to range [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (abs(error) < 3) {  // Stop rotating when close enough
            stopMotors();
            break;
        }

        int rotationSpeed = map(abs(error), 0, 180, 80, MOTOR_SPEED);  // Proportional control

        if (error > 0) {
            setMotorSpeed(rotationSpeed, -rotationSpeed);  // Turn right
        } else {
            setMotorSpeed(-rotationSpeed, rotationSpeed);  // Turn left
        }

        delay(100);
    }
}

// Function to move the robot to the target position
void moveToPosition(float targetX, float targetY) {
    while (true) {
        float currentX = Enes100.getX();
        float currentY = Enes100.getY();
        float distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        if (distance < 2) {  // Stop moving when close enough
            stopMotors();
            break;
        }

        int moveSpeed = map(distance, 0, 50, 100, MOTOR_SPEED);  // Proportional speed

        setMotorSpeed(moveSpeed, moveSpeed);  // Move forward

        delay(100);
    }
}

// Move forward
// Basic movement functions
void moveForward() {
  motors.forward();
}

void moveBackward() {
  motors.backward();
}

void stopMotors() {
  motors.stop();
}

void turnLeft() {
    motors.runA(L298N::FORWARD);   // Right motor forward
    motors.runB(L298N::BACKWARD);    // Left motor backward
    motors.setSpeedA(150);
    motors.setSpeedB(150);
    motors.forward();
}

void turnRight() {
    motors.runA(L298N::BACKWARD);   // Right motor backward
    motors.runB(L298N::FORWARD);    // Left motor forward
    motors.setSpeedA(150);
    motors.setSpeedB(150);
    motors.forward();
}

// Stop motors
void stopMotors(int duration) {
    motors.stop();
    printMotorStatus();
    delay(duration);
}
