// 👇 Add this helper function to measure distance using ultrasonic sensor
#include "Enes100.h"
#include <L298NX2.h>



//define pins for the H-bridge
const int EN_A = 3;
const int IN1_A = 5;
const int IN2_A = 6;

const int EN_B = 9;
const int IN1_B = 8;
const int IN2_B = 7;

L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B); //Initialize the motors



// Pin connected to the transistor base
const int FAN_PIN = 11;  



// Define pins for Sensor 1
const int trigPin1 = 14;
const int echoPin1 = 15;

// Define pins for Sensor 2
const int trigPin2 = 16;
const int echoPin2 = 17;



// Team information - keep the name you saw in the images
const char TEAM_NAME = "It's lit";  // Match what's in the system
const int MARKER_ID = 635;             
const int ROOM_NUMBER = 1120;
const int WIFI_TX = 4;  // ESP32-CAM TX connects to Arduino pin 8
const int WIFI_RX = 2;  // ESP32-CAM RX connects to Arduino pin 9

int counter = 0;



// Function prototypes
void navigateTo(float targetX, float targetY, float targetTheta);
void rotateToAngle(float targetTheta);
void moveToPosition(float targetX, float targetY);



void setup() {
    // Initialize communication with the Vision System
    //Enes100.begin("TEAM_NAME", FIRE, 3, 8, 9);

    Serial.begin(9600);  
    while (!Serial);  // Wait for Serial Monitor

    //Set sensor pins as output
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);

    // Set fan control pin as output
    pinMode(FAN_PIN, OUTPUT);

    
    motors.setSpeed(200);

    Serial.begin(9600);
    Serial.println("Starting WiFi module test...");
  
    // Initialize Vision System communication
    Enes100.begin(TEAM_NAME, FIRE, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
  
    // Wait for connection (with timeout)
    int timeout = 0;
    while (!Enes100.isConnected() && timeout < 100) {
    Serial.println("Waiting for connection...");
    delay(100);
    timeout++;
  }
  
  if (Enes100.isConnected()) {
    Serial.println("Connected to WiFi!");
    // Send a test message
    Enes100.println("WiFi module connected successfully!");
  } else {
    Serial.println("Failed to connect. Check wiring and settings.");
  }
  
    
    // Print initial message to confirm setup is complete
    Enes100.println("OTV setup complete");
    Serial.println("OTV setup complete - Ready for mission");
}


long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
   
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;
    return distance;
}



// 👇 Replace your entire loop() function with this:
void loop() {
  
  long distance1 = getDistance(trigPin1, echoPin1);
  long distance2 = getDistance(trigPin2, echoPin2);
   
  bool object1 = distance1 <= 2;
  bool object2 = distance2 <= 2;
  char status;

  Serial.print("Front Distance: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Serial.print("Side Distance: ");
  Serial.print(distance2);
  Serial.println(" cm");

  navigateTo(50, 30, 90);
  delay(2000);  // Wait before next movement
  
  // Simple test sequence - can be replaced with mission code later
  testFanControl();
  delay(2000);
}



// Function to navigate to (x, y) and orient to theta
void navigateTo(float targetX, float targetY, float targetTheta) {
    rotateToAngle(atan2(targetY - Enes100.getY(), targetX - Enes100.getX()) * 180 / PI);
    moveToPosition(targetX, targetY);
    rotateToAngle(targetTheta);
}

//----------------------------------------------------------------------------------------------------------------
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

        //int rotationSpeed = map(abs(error), 0, 180, 80);  // Proportional control

        if (error > 0) {
            turnRight();  // Turn right
        } else {
            turnLeft();  // Turn left
        }

        delay(100);
    }

  /*if (distance1 > 20) {
    Serial.println("✅ Path clear. Moving forward...");
    moveForward();
  } else {
    Serial.println("🛑 Obstacle detected. Stopping motors...");
    stopMotors();
    delay(200);  // Small delay for stability
  }
  */
  
}

// Rotateto code with varying speed
/*
void rotateToAngle(float targetTheta) {
    const int maxSpeed = 180;
    const int minSpeed = 100;
    const float k = 0.05;  // Steepness of curve, change to change the speed variation

    while (true) {
        float currentTheta = Enes100.getTheta();
        float error = targetTheta - currentTheta;

        // Normalize error to range [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (abs(error) < 3) {
            stopMotors();
            Serial.println("🧭 Aligned to Target Angle");
            break;
        }

        // Calculate curved speed based on angle error
        float curvedSpeed = maxSpeed * (1 - exp(-k * abs(error)));
        int speed = constrain((int)curvedSpeed, minSpeed, maxSpeed);

        if (error > 0) {
            motors.runA(L298N::BACKWARD); // Left motor backward
            motors.runB(L298N::FORWARD);  // Right motor forward
        } else {
            motors.runA(L298N::FORWARD);  // Left motor forward
            motors.runB(L298N::BACKWARD); // Right motor backward
        }

        motors.setSpeedA(speed);
        motors.setSpeedB(speed);

        Serial.print("🔄 Rotating | Error: ");
        Serial.print(error);
        Serial.print("° | Speed: ");
        Serial.println(speed);

        delay(100);
    }
}
*/

//----------------------------------------------------------------------------------------------------------------
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
          moveForward();
          motors.setSpeed(150);
          delay(100);
    }
}

// Move to code with varying speed
/*
void moveToPosition(float targetX, float targetY) {
    const int minDistance = 20;    // cm - safety distance
    const int maxSpeed = 230;
    const float k = 0.05;          // curve steepness
    const int minSpeed = 100;      // avoid stalling

    while (true) {
        float currentX = Enes100.getX();
        float currentY = Enes100.getY();
        float distanceToTarget = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        long frontDistance = getDistance(trigPin1, echoPin1);  // Use front ultrasonic sensor

        // Stop if robot is close enough to the target position
        if (distanceToTarget < 2) {
            stopMotors();
            Serial.println("📍 Reached Target Position");
            break;
        }

        // Stop if obstacle is too close
        if (frontDistance <= minDistance) {
            stopMotors();
            Serial.print("🛑 Obstacle too close (");
            Serial.print(frontDistance);
            Serial.println(" cm). Stopping...");
            break;
        }

        // Calculate curved speed
        float curvedSpeed = maxSpeed * (1 - exp(-k * (frontDistance - minDistance)));
        int speed = constrain((int)curvedSpeed, minSpeed, maxSpeed);

        motors.setSpeed(speed);
        moveForward();

        Serial.print("🚗 Speed: ");
        Serial.print(speed);
        Serial.print(" | Front Distance: ");
        Serial.print(frontDistance);
        Serial.print(" cm | Distance to Target: ");
        Serial.println(distanceToTarget);

        delay(100);  // Small delay for stability
    }
}
*/
//----------------------------------------------------------------------------------------------------------------
//fan control
//----------------------------------------------------------------------------------------------------------------
// Function to turn fans on at specified speed (0-255)
void fanOn(int speed) {
  analogWrite(FAN_PIN, speed);
  Serial.print("Fans ON at speed: ");
  Serial.println(speed);
}

// Function to turn fans off
void fanOff() {
  analogWrite(FAN_PIN, 0);
  Serial.println("Fans OFF");
}

// Test function to demonstrate fan control
void testFanControl() {
  // Full speed
  fanOn(255);
  delay(3000);
  
  // Off
  fanOff();
  delay(2000);
  
  // Half speed
  fanOn(128);
  delay(3000);
  
  // Off
  fanOff();
  delay(2000);
}
//--------------------------------------------------------------------------------------------------------------
//motor control
//-----------------------------------------------------------------------------------------------------------------
//move forward 
void moveForward() {
  motors.forward();
}

//move backwards
void moveBackward(){
  motors.backward();
}

//Stop motors function
void stopMotors() {
  motors.stop();
}

//left turn function
void turnLeft() {
    motors.runA(L298N::BACKWARD);     // Left motor backward  
    motors.runB(L298N::FORWARD);      // Right motor forward
    motors.setSpeedA(150);
    motors.setSpeedB(150);

    printMotorStatus();
    delay(200);
}

//right turn function
void turnRight(){
    motors.runA(L298N::FORWARD);   // Left motor forward
    motors.runB(L298N::BACKWARD);  // Right motor backward
    motors.setSpeedA(150);         // You can tweak these to fine-tune slipping
    motors.setSpeedB(150);
    
    printMotorStatus();
    delay(200);
}

//----------------------------------------------------------------------------------------------------------------
//print status
//----------------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------------
