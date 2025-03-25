// Define pins for Sensor 1
const int trigPin1 = 14;
const int echoPin1 = 15;

// Define pins for Sensor 2
const int trigPin2 = 16;
const int echoPin2 = 17;

void setup() {
    Serial.begin(9600);
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
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

void loop() {
    long distance1 = getDistance(trigPin1, echoPin1);
    long distance2 = getDistance(trigPin2, echoPin2);
    
    bool object1 = distance1 <= 2;
    bool object2 = distance2 <= 2;
    char status;
    
    if (object1 && object2) {
        status = 'C';
    } else if (object1) {
        status = 'A';
    } else if (object2) {
        status = 'B';
    } else {
        status = '-';
    }
    
    Serial.print("Distance 1: ");
    Serial.print(distance1);
    Serial.print(" cm  |  ");
    Serial.print("Distance 2: ");
    Serial.print(distance2);
    Serial.print(" cm  |  ");
    Serial.print("Status: ");
    Serial.println(status);
    
    delay(500);
}