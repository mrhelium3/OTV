void rotateToAngle(float targetTheta) {
  const float kP = 50;              // Proportional gain
  const int minSpeed = 50;          // Minimum turning speed
  const int maxSpeed = 200;         // Cap speed to avoid overshoot
  const float threshold = 0.05;     // Acceptable error range (radians)
  const float brakingZone = 0.2;    // When to reduce speed
  const int maxFlips = 5;           // Max oscillations allowed

  int flipCount = 0;
  bool lastSign = true; // Track the last sign of the error
  unsigned long startTime = millis();
  const unsigned long timeout = 10000; // 10-second safety timeout

  while (true) {
    float currentTheta = toPositiveAngle(Enes100.getTheta());
    float error = angleDiff(targetTheta, currentTheta); // currentTheta = current angle, in radians (-π to π)

    if (abs(error) < threshold) {
      stopMotors(); // Stop if within range
      break;
    }

    // Determine direction (sign)
    bool currentSign = (error > 0);
    if (currentSign != lastSign) {
      flipCount++;
      lastSign = currentSign;
    }

    if (flipCount > maxFlips || millis() - startTime > timeout) {
      Serial.println("⚠️ Too many oscillations or timeout — stopping.");
      stopMotors();
      break;
    }

    int speed;
    if (abs(error) < brakingZone) {
      speed = minSpeed; // gentle correction
    } else {
      speed = constrain(abs(error) * kP, minSpeed, maxSpeed);
    }

    if (error > 0) {
      // Turn counter-clockwise
      setMotorSpeed(-speed, speed);
    } else {
      // Turn clockwise
      setMotorSpeed(speed, -speed);
    }

    delay(50); // Small delay for stability
  }
}

