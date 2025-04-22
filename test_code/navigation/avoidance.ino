// extension of navigation.ino 
// this is for obstacle detection and avoidance.
// 1. need to replace example coordinates
// 2. adjust MOTOR_SPEED based on actual behavior
// 3. Dekay values based on speed of OTV
// 4. pin assignments need to match (ultrasonic / h-bridge)

#include "C:\Users\owenm\OneDrive\Documents\GitHub\OTV\ENES100ArduinoLibrary-master\src\Enes100.h"

const char* teamName = "Chiu Chiu Train";
byte teamType = FIRE;
int markerId = 467; // varies
int roomNumber = 1120;
int wifiModuleTX =12;
int wifiModuleRX = 11;

// Motor pins
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 9
#define RIGHT_MOTOR_BACKWARD 10
#define MOTOR_SPEED 150

// Ultrasonic sensor pins
#define FRONT_TRIG_PIN 14  // A0
#define FRONT_ECHO_PIN 15  // A1
#define SIDE_TRIG_PIN 16   // A2
#define SIDE_ECHO_PIN 17   // A3

// Constants
#define OBSTACLE_DISTANCE 25  // Distance in cm to detect obstacle
#define SAFE_DISTANCE 30      // Distance considered safe from obstacle

// Function prototypes
void navigateTo(float targetX, float targetY, float targetTheta);
void rotateToAngle(float targetTheta);
void moveToPosition(float targetX, float targetY);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
float readUltrasonicDistance(int trigPin, int echoPin);
bool frontObstacleDetected();
bool sideObstacleDetected();
void avoidObstacle(float targetX, float targetY);

void setup() {
    // Initialize serial for debugging
    Serial.begin(9600);
    
    // Initialize communication with the Vision System using your team parameters
    Enes100.begin(teamName, teamType, markerId, roomNumber, wifiModuleTX, wifiModuleRX);
    delay(1000);  // Allow time for the connection to establish
    
    // Set motor pins as outputs
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    
    // Set ultrasonic sensor pins
    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(SIDE_TRIG_PIN, OUTPUT);
    pinMode(SIDE_ECHO_PIN, INPUT);
    
    // Print initial message to confirm setup is complete
    Enes100.println("OTV setup complete");
    Serial.println("OTV setup complete - Ready for mission");
}

void loop() {
    // Check if vision system is connected
    if (!Enes100.updateLocation()) {
        Serial.println("Warning: Cannot update location");
        stopMotors(); // Safety stop if we lose our position
        delay(100);
        return;
    }
    
    // Print current position for debugging
    Serial.print("Current position: (");
    Serial.print(Enes100.location.x);
    Serial.print(", ");
    Serial.print(Enes100.location.y);
    Serial.print("), Theta: ");
    Serial.println(Enes100.location.theta);
    
    // Main mission logic
    // First navigate to mission site (example coordinates - replace with actual mission site)
    navigateTo(1.5, 1.0, 0.0);
    
    // Perform mission operations here
    Enes100.println("Mission operations would be performed here");
    delay(2000);
    
    // Navigate to the destination zone (example coordinates - replace with actual destination)
    navigateTo(3.5, 1.0, 0.0);
    
    // End of mission
    Enes100.println("Mission complete!");
    stopMotors();
    while(1); // Stop execution
}

// Function to navigate to a specific position with a specific final orientation
void navigateTo(float targetX, float targetY, float targetTheta) {
    // First move to the position
    moveToPosition(targetX, targetY);
    
    // Then rotate to the desired orientation
    rotateToAngle(targetTheta);
}

// Function to move to a specific position while avoiding obstacles
void moveToPosition(float targetX, float targetY) {
    float distanceThreshold = 0.15; // 15cm accuracy requirement
    
    while (true) {
        // Update location
        if (!Enes100.updateLocation()) {
            Serial.println("Warning: Cannot update location");
            stopMotors();
            delay(100);
            continue;
        }
        
        // Calculate distance to target
        float deltaX = targetX - Enes100.location.x;
        float deltaY = targetY - Enes100.location.y;
        float distanceToTarget = sqrt(deltaX*deltaX + deltaY*deltaY);
        
        // If we're close enough to the target, stop
        if (distanceToTarget < distanceThreshold) {
            stopMotors();
            Serial.println("Reached target position");
            return;
        }
        
        // Calculate angle to target
        float angleToTarget = atan2(deltaY, deltaX);
        
        // Adjust the angle to keep it between -PI and PI
        while (angleToTarget > PI) angleToTarget -= 2*PI;
        while (angleToTarget < -PI) angleToTarget += 2*PI;
        
        // First rotate to face the target
        rotateToAngle(angleToTarget);
        
        // Check for obstacles
        if (frontObstacleDetected()) {
            // If obstacle detected, avoid it
            avoidObstacle(targetX, targetY);
            continue; // Restart the navigation process
        }
        
        // Move forward towards the target
        setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
        delay(100); // Short delay for movement
    }
}

// Function to rotate to a specific angle
void rotateToAngle(float targetTheta) {
    float angleThreshold = 0.1; // About 5.7 degrees of accuracy
    
    while (true) {
        // Update location
        if (!Enes100.updateLocation()) {
            Serial.println("Warning: Cannot update location");
            stopMotors();
            delay(100);
            continue;
        }
        
        // Calculate the angle difference
        float angleDiff = targetTheta - Enes100.location.theta;
        
        // Adjust the angle to keep it between -PI and PI
        while (angleDiff > PI) angleDiff -= 2*PI;
        while (angleDiff < -PI) angleDiff += 2*PI;
        
        // If we're close enough to the target angle, stop
        if (abs(angleDiff) < angleThreshold) {
            stopMotors();
            Serial.println("Reached target angle");
            return;
        }
        
        // Rotate clockwise or counterclockwise depending on the shortest path
        if (angleDiff > 0) {
            // Turn counterclockwise (left)
            setMotorSpeed(-MOTOR_SPEED, MOTOR_SPEED);
        } else {
            // Turn clockwise (right)
            setMotorSpeed(MOTOR_SPEED, -MOTOR_SPEED);
        }
        
        delay(50); // Short delay for rotation
    }
}

// Function to avoid obstacles
void avoidObstacle(float targetX, float targetY) {
    Serial.println("Avoiding obstacle");
    
    // First, stop and assess the situation
    stopMotors();
    delay(200);
    
    // Get current location
    Enes100.updateLocation();
    float currentX = Enes100.location.x;
    float currentY = Enes100.location.y;
    float currentTheta = Enes100.location.theta;
    
    // Turn 90 degrees left to go parallel to the obstacle
    rotateToAngle(currentTheta + PI/2);
    
    // Drive alongside the obstacle until we can go forward
    while (true) {
        // Move forward a bit
        setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
        delay(300);
        stopMotors();
        
        // Check if we still detect the obstacle to our right
        if (!sideObstacleDetected()) {
            // We've cleared the obstacle, stop and check
            stopMotors();
            delay(200);
            
            // Drive a bit more to clear the obstacle completely
            setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
            delay(500);
            stopMotors();
            
            // Recalculate path to target (will happen in moveToPosition)
            return;
        }
        
        // If we detect an obstacle in front while moving parallel, turn again
        if (frontObstacleDetected()) {
            // Turn another 90 degrees left
            Enes100.updateLocation();
            rotateToAngle(Enes100.location.theta + PI/2);
        }
    }
}

// Function to set the speed of both motors
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Set left motor
    if (leftSpeed >= 0) {
        analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
        analogWrite(LEFT_MOTOR_BACKWARD, 0);
    } else {
        analogWrite(LEFT_MOTOR_FORWARD, 0);
        analogWrite(LEFT_MOTOR_BACKWARD, -leftSpeed);
    }
    
    // Set right motor
    if (rightSpeed >= 0) {
        analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
        analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    } else {
        analogWrite(RIGHT_MOTOR_FORWARD, 0);
        analogWrite(RIGHT_MOTOR_BACKWARD, -rightSpeed);
    }
}

// Function to stop both motors
void stopMotors() {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}

// Function to read distance from ultrasonic sensor
float readUltrasonicDistance(int trigPin, int echoPin) {
    // Clear the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Set the trigger pin high for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the echo pin, convert time to distance
    long duration = pulseIn(echoPin, HIGH, 20000); // Timeout after 20ms
    
    // If timeout occurred, return a large value
    if (duration == 0) {
        return 400.0; // No echo received, assume no obstacle
    }
    
    // Calculate the distance in cm
    float distance = duration * 0.034 / 2.0;
    return distance;
}

// Function to check if there's an obstacle in front
bool frontObstacleDetected() {
    float distance = readUltrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    return (distance < OBSTACLE_DISTANCE);
}

// Function to check if there's an obstacle to the side
bool sideObstacleDetected() {
    float distance = readUltrasonicDistance(SIDE_TRIG_PIN, SIDE_ECHO_PIN);
    return (distance < SAFE_DISTANCE);
}
