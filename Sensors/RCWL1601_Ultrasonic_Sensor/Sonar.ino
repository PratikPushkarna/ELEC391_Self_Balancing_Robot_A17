#define TRIGGER_PIN 5 // Define Trigger pin
#define ECHO_PIN 6    // Define Echo pin

void setup() {
    Serial.begin(115200);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float getDistance() {
    // Send a short pulse to trigger the ultrasonic sensor
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    // Read the time it takes for the echo to return
    long duration = pulseIn(ECHO_PIN, HIGH);
    
    // Convert time to distance (Speed of sound is ~343 m/s or 0.0343 cm/us)
    float distance = (duration * 0.0343) / 2;
    return distance;
}

void loop() {
    float distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(500); // Wait before the next measurement
}
