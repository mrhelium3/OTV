#include "Enes100.h"

#define LEFT_MOTOR_FORWARD 5  // Adjust based on wiring
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 9
#define RIGHT_MOTOR_BACKWARD 10
#define MOTOR_SPEED 150  // Adjust for speed control (0-255)

// Function prototypes
void navigateTo(float targetX, float targetY, float targetTheta);
void rotateToAngle(float targetTheta);
void moveToPosition(float targetX, float targetY);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();

void setup() {
    // Initialize communication with the Vision System
    Enes100.begin("TeamName", FIRE, 3, 8, 9);

    // Set motor pins as outputs
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
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

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    analogWrite(LEFT_MOTOR_FORWARD, leftSpeed > 0 ? leftSpeed : 0);
    analogWrite(LEFT_MOTOR_BACKWARD, leftSpeed < 0 ? -leftSpeed : 0);
    analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed > 0 ? rightSpeed : 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, rightSpeed < 0 ? -rightSpeed : 0);
}

// Function to stop the motors
void stopMotors() {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}
