void rotateToAngle(float targetTheta) {
    const int maxSpeed = 150;
    const int minSpeed = 70;
    // const float k = 1.5;

    float lastTheta = Enes100.getTheta(); // ← Initialize before loop
    int steadyCounter = 0;
    float Theta = 0;

    while (true) {
        float currentTheta = Enes100.getTheta();
        const int burstDuration = 100; // ms per burst
        if (currentTheta<0){
        Theta = abs(currentTheta); //move the range from -pi - pi ---> 0 - 2pi
        } else {
          Theta = currentTheta + 3.14;
        }
        float error = targetTheta - Theta; 
        // Normalize error to [-PI, PI]
        //while (error > PI) error -= 2 * PI;
        //while (error < -PI) error += 2 * PI;

        // Check if close enough
        if (error < 0.05) {
            steadyCounter++;
        } else {
            steadyCounter = 0;
        }

        if (steadyCounter > 5) {
            stopMotors();
            Enes100.println("🧭 Aligned!");
            break;
        }

        int speed = 100; // keep it low for now

        if (error > 0) {
            motors.runA(L298N::BACKWARD);
            motors.runB(L298N::FORWARD);
        } else {
            motors.runA(L298N::FORWARD);
            motors.runB(L298N::BACKWARD);
        }

        motors.setSpeedA(speed);
        motors.setSpeedB(speed);

        float deltaTheta = Theta - lastTheta;
        lastTheta = Theta;

        delay(burstDuration);  // Short controlled burst
        stopMotors();

        Enes100.print("Rotating | Error: ");
        Enes100.println(error);
        Enes100.print("ΔTheta: ");
        Enes100.println(deltaTheta);

        delay(200);
    }
}
