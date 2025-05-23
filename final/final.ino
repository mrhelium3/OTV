#include <L298NX2.h>

#include <Enes100.h>

//Fan pin
const int FAN_PIN = 11;  

// Define pins for Sensor 1
const int trigPin1 = 14;
const int echoPin1 = 15;

// Define pins for Sensor 2
const int trigPin2 = 16;
const int echoPin2 = 17;

// Define pins for Sensor 3
const int trigPin3 = 18;
const int echoPin3 = 19;

//Define motor pins
const int EN_A = 3;
const int IN1_A = 5;
const int IN2_A = 6;

const int EN_B = 9;
const int IN1_B = 8;
const int IN2_B = 7;

bool missionActive = true;

//Define navigation constants
const int OBSTACLE_THRESHOLD = 20;    // cm
const float ARENA_Y_MIDPOINT = 1.0;   // meters
const float X_GOAL_THRESHOLD = 2.8;   // meters
const int FORWARD_SPEED = 150;
const int SIDEWAYS_SPEED = 80;

//Define Motor
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Team information 
const char* TEAM_NAME = "ChiuChiuTrain";  // Match what's in the system
const int MARKER_ID = 635;             
const int ROOM_NUMBER = 1120;
const int WIFI_TX = 4;  // ESP32-CAM TX connects to Arduino pin 4
const int WIFI_RX = 2;  // ESP32-CAM RX connects to Arduino pin 2

//============================
//Setup Functions
//============================
void setup() {
  Serial.begin(9600);  
  while (!Serial);  
  pinMode(FAN_PIN, OUTPUT);
  ultrasonicSetup();

  //Vision system setup
  Enes100.begin(TEAM_NAME, FIRE, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
  if (Enes100.isConnected()) {
    Serial.println("Connected to WiFi!");
    // Send a test message
    Enes100.println("WiFi module connected successfully!");
  } else {
    Serial.println("Failed to connect. Check wiring and settings.");
  }

}

//initialize ultrasonic pins
void ultrasonicSetup(){
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}

//============================
//Main Loop
//============================

void loop() {
   // missionActive = true;
    //while(missionActive){
   goToStartingLocation();
    delay(500);
    motors.runA(L298N::FORWARD);
    motors.runB(L298N::FORWARD);
    delay(1550);
    delay(400);
    motors.stop();
   // moveForward();
    delay(1000);
    
    //navigateObstacles();
    //delay(1000);
  
    //do activities at mission site
    readOrientation();
    delay(100);
    //countCandles();
    delay(100);
    extinguish();
    delay(1000);
 
    moveBackward();
    delay(500);
    stopMotors();
    backUp();
    
    delay(500);
    
    
   

    //Navigate past the obstacles
    navigateObstacles();
    moveForward();
    
    delay(8000);
    motors.stop();

    //Go to the finish
    //goOverLog();
    //missionActive = false;
  
 

}

//============================
//Subsystem calls
//============================
//Identify starting location
void goToStartingLocation() {
  while (!Enes100.isVisible()) {
    Enes100.println("Waiting for vision lock...");
    delay(100);
  }

  delay(1000);
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  bool isTop = y > 1.0;

  if (isTop) {
    rotateToAngle(-PI/2);
   // moveToPosition(0.55, 0.60);
  } else { 
    rotateToAngle(PI/2);
   
  }

}

void backUp(){
   delay(1000);
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  bool isTop = y > 1.0;

  if (isTop) {
    turnRight();
   // moveToPosition(0.55, 0.60);
  } else { 
    turnLeft();
   
  }
}

//ML count the number of candles
void countCandles(){
  int prediction = Enes100.MLGetPrediction(5);  // Use your correct model index

  Serial.print("Candle prediction: ");
  switch (prediction) {
    case 0:
      Enes100.mission(NUM_CANDLES, 5);
      //Serial.println("5 candles lit");
      break;
    case 1:
      Enes100.mission(NUM_CANDLES, 2);
      break;
    // case 2:
    //  Serial.println("4 candles lit");
    //  break;
    // case 3:
    //   Serial.println("5 candles lit");
    //  break;
    default:
      Serial.println("No prediction or unknown result");
      break;
  }

  delay(5000);
}

//Put out the candles
void extinguish(){
  const int fanSpeed = 250;
  const int fanTime = 3000;

  fanOn(fanSpeed);
  delay(fanTime);
  fanOff();

  Enes100.println("Fire Suprressed!");
}

//Navigate past the obstacles
//Navigate past the obstacles
void navigateObstacles() {
    const int OBSTACLE_THRESHOLD = 20;    // cm
    const float ARENA_Y_MIDPOINT = 1.0;   // meters
    const float X_GOAL_THRESHOLD = 2.8;   // meters
    long sideDistance = getDistance(trigPin3, echoPin3);
  
  while (Enes100.getX() < X_GOAL_THRESHOLD) {
    long frontDistance1 = getDistance(trigPin1, echoPin1);
    long frontDistance2 = getDistance(trigPin2, echoPin2); 
    float currentY = Enes100.getY();

    if (frontDistance1 <= OBSTACLE_THRESHOLD || frontDistance2 <= OBSTACLE_THRESHOLD){
      stopMotors();
      Enes100.println("Obstacle detected!");

      // Always turn left to expose right-side sensor
      turnLeft();
      delay(200);

      // If we're near the top → move FORWARD while Sensor 3 sees obstacle
      // If near bottom → move BACKWARD instead
      if (currentY > ARENA_Y_MIDPOINT) {
        while (sideDistance <= OBSTACLE_THRESHOLD) {
          motors.forward();
          motors.setSpeed(SIDEWAYS_SPEED);
          delay(100);
        }
      } else {
        while (sideDistance <= OBSTACLE_THRESHOLD) {
          motors.backward();
          motors.setSpeed(SIDEWAYS_SPEED);
          delay(100);
        }
      }

      stopMotors();
      delay(300);
      Enes100.println("Cleared obstacle");

      // Turn back right to face forward again
      turnRight();
      delay(300);
    }

    // Continue forward
    motors.forward();
    motors.setSpeed(FORWARD_SPEED);
    delay(100);
  }
  stopMotors();
}

//Go to the finish
void goOverLog() {
  navigateTo(3.1, .55, 0);
  delay(100);
  moveToPosition(3.8, .55);
  Enes100.println("Finished!");
}
//============================
//Navigate Functions
//===========================

//Normalize a given angle



//Rotate to a specified angle Funciton 
void rotateToAngle(float targetTheta) {
    const int maxSpeed = 200;
    const int minSpeed = 120;
    const float k = 0.25;
    const int burstDuration = 75; // ms per burst
    const int timeoutDuration = 32000; // 5 seconds timeout

    unsigned long startTime = millis();  // ⏱ Start timer
    float lastTheta = Enes100.getTheta();

    while (true) {
        // ⏱ Check for timeout
        if (millis() - startTime > timeoutDuration) {
            stopMotors();
            Enes100.println("❌ Timeout: Failed to reach target angle");
            break;
        }

        float currentTheta = Enes100.getTheta();
        float error = targetTheta - currentTheta;

        // ✅ Normalize error to [-PI, PI]
        error = atan2(sin(error), cos(error));

        Enes100.print("Angle Error: ");
        Enes100.println(error);

        if (abs(error) < 0.05) {
            stopMotors();
            Enes100.println("✅ Aligned with target angle");
            break;
        }

        float curvedSpeed = maxSpeed * (1 - exp(-k * abs(error)));
        int speed = constrain((int)curvedSpeed, minSpeed, maxSpeed);

        if (error > 0) {
            motors.runA(L298N::BACKWARD);
            motors.runB(L298N::FORWARD);
        } else {
            motors.runA(L298N::FORWARD);
            motors.runB(L298N::BACKWARD);
        }

        motors.setSpeedA(speed);
        motors.setSpeedB(speed);

        float deltaTheta = currentTheta - lastTheta;
        lastTheta = currentTheta;

        delay(burstDuration);
        stopMotors();

        //Enes100.print("ΔTheta: ");
        //Enes100.println(deltaTheta);

        delay(200);
    }
}














//Move to a target Position
void moveToPosition(float targetX, float targetY) {
    const float tolerance = 0.15;  // within 150mm
    const int speed = 200;

    while (true) {
        float currentX = Enes100.getX();
        float currentY = Enes100.getY();
        float distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        if (distance < 0.2) {  // Stop moving when close enough
            stopMotors();
            break;
        }

        if (distance < tolerance) {
            stopMotors();
            break;
        }

        moveForward();
        motors.setSpeed(speed);
        delay(100);
    }
}

//Navigate to a specified location 
void navigateTo(float targetX, float targetY, float targetTheta) {
    rotateToAngle(atan2(targetY - Enes100.getY(), targetX - Enes100.getX()));
    moveToPosition(targetX, targetY);
    rotateToAngle(targetTheta);
}


//============================
//Motor Control Functions
//============================
//Forward Movement
void moveForward() {
  motors.forward();
}

//Backwards Movement
void moveBackward(){
  motors.backward();
}

//Stop motors function
void stopMotors() {
  motors.stop();
}

//Left turn function
void turnRight() {
    motors.setSpeedA(180);
    motors.setSpeedB(180);
    motors.runA(L298N::BACKWARD);     // Left motor backward  
    motors.runB(L298N::FORWARD);      // Right motor forward
    

    delay(1044);
    stopMotors();
}

//Right turn function
void turnLeft(){
    motors.setSpeedA(160);         // You can tweak these to fine-tune slipping
    motors.setSpeedB(160);
    motors.runA(L298N::FORWARD);   // Left motor forward
    motors.runB(L298N::BACKWARD);  // Right motor backward
    
    

    delay(900);
    stopMotors();
}

//============================
//Ultrasonic Functions
//============================

//Get Distance from ultrasonic sensor
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

//Identify Orientation
void readOrientation(){
  const unsigned long duration = 5000; // 5 seconds
  const unsigned long interval = 200;  // take a reading every 200 ms (25 readings total)
  const int numReadings = duration / interval;

  int countA = 0;
  int countB = 0;
  int countC = 0;

  for (int i = 0; i < numReadings; i++) {
    long distance1 = getDistance(trigPin1, echoPin1);
    long distance2 = getDistance(trigPin2, echoPin2);
    
    bool object1 = distance1 <= 2;
    bool object2 = distance2 <= 2;
    char status;

    if (object1 && object2) {
      status = 'C';
      countC++;
    } else if (object1) {
      status = 'A';
      countA++;
    } else if (object2) {
      status = 'B';
      countB++;
    }

    delay(interval);
  }

  // Determine the most common status
  if (countC >= countA && countC >= countB) {
    Enes100.mission(TOPOGRAPHY, TOP_C);
  } else if (countA >= countB) {
    Enes100.mission(TOPOGRAPHY, TOP_A);
  } else {
    Enes100.mission(TOPOGRAPHY, TOP_B);
  }
}

//============================
//Fan Functions
//============================

//Enable Fan
void fanOn(int speed) {
  analogWrite(FAN_PIN, speed);
  Serial.print("Fans ON at speed: ");
  Serial.println(speed);
}

//Disable Fan
void fanOff() {
  analogWrite(FAN_PIN, 0);
  Serial.println("Fans OFF");
}







