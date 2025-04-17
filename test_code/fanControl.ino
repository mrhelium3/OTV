// Pin connected to the transistor base
const int FAN_PIN = 3;  

void setup() {
  // Set fan control pin as output
  pinMode(FAN_PIN, OUTPUT);
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Fan Control System Ready");
}

void loop() {
  // Simple test sequence - can be replaced with mission code later
  testFanControl();
}

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
