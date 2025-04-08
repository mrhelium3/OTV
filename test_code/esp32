#include "Enes100.h"

// Team information - keep the name you saw in the images
const char* TEAM_NAME = "It's lit";  // Match what's in the system
const int MARKER_ID = 9;             // This is still needed for initialization
const int ROOM_NUMBER = 1120;
const int WIFI_TX = 8;  // ESP8266 TX connects to Arduino pin 8
const int WIFI_RX = 9;  // ESP8266 RX connects to Arduino pin 9

int counter = 0;

void setup() {
  // Start Serial for debugging
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
}

void loop() {
  // Check connection status periodically
  if (Enes100.isConnected()) {
    Serial.println("Still connected to WiFi");
    
    // Send a test message every 5 seconds
    counter++;
    
    // Test message sending (Task 2)
    Enes100.print("Test message #");
    Enes100.println(counter);
    
    // Try sending a mission value too
    Enes100.mission(NUM_CANDLES, counter % 5);
    
    Serial.print("Sent test message #");
    Serial.println(counter);
  } else {
    Serial.println("WiFi connection lost!");
  }
  
  delay(5000); // Wait 5 seconds between messages
}
