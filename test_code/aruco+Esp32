//this is just a more advanced verison of the esp32 code; it tracks the OTV as well.

#include "Enes100.h"

// Team information - keep the name you saw in the images
const char* TEAM_NAME = "It's lit";  // Match what's in the system
const int MARKER_ID = 9;             
const int ROOM_NUMBER = 1120;
const int WIFI_TX = 8;  // ESP32-CAM TX connects to Arduino pin 8
const int WIFI_RX = 9;  // ESP32-CAM RX connects to Arduino pin 9

int counter = 0;

void setup() {
  // Start Serial for debugging
  Serial.begin(9600);
  Serial.println("Starting ESP32-CAM test...");
  
  // Initialize Vision System communication with ESP32-CAM
  Enes100.begin(TEAM_NAME, FIRE, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
  
  // Wait for connection (with timeout)
  int timeout = 0;
  while (!Enes100.isConnected() && timeout < 100) {
    Serial.println("Waiting for connection...");
    delay(100);
    timeout++;
  }
  
  if (Enes100.isConnected()) {
    Serial.println("ESP32-CAM connected to WiFi!");
    // Send a test message
    Enes100.println("ESP32-CAM module connected successfully!");
  } else {
    Serial.println("Failed to connect. Check wiring and settings.");
  }
}

void loop() {
  // Check connection status
  if (Enes100.isConnected()) {
    Serial.println("ESP32-CAM connected to vision system");
    
    // Send a test message every 5 seconds
    counter++;
    
    // Task 1: Attempt to get position data (will only work if Aruco marker is visible)
    if (Enes100.isVisible()) {
      Serial.print("Position: X=");
      Serial.print(Enes100.getX());
      Serial.print(", Y=");
      Serial.print(Enes100.getY());
      Serial.print(", Theta=");
      Serial.println(Enes100.getTheta());
    } else {
      Serial.println("Aruco marker not visible (expected if not in arena)");
    }
    
    // Task 2: Send test mission message
    Enes100.mission(NUM_CANDLES, counter % 5);
    Serial.print("Sent test candle count: ");
    Serial.println(counter % 5);
    
    // Also send a regular text message for visibility
    Enes100.print("ESP32-CAM Test #");
    Enes100.println(counter);
  } else {
    Serial.println("ESP32-CAM connection lost!");
  }
  
  delay(5000); // Wait 5 seconds between updates
}
