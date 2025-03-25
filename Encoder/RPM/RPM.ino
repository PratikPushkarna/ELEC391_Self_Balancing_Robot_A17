#include <Wire.h>
#include <AS5600.h>

#define TCA_ADDR 0x70  // I2C address of TCA9548A
#define AS5600_ADDR 0x36  // I2C address of AS5600

AS5600 as5600;  // Create an instance of the AS5600

unsigned long lastTime[2] = {0, 0};
uint16_t lastAngle[2] = {0, 0};

void selectMuxChannel(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);  // Send a byte to select the channel
    Wire.endTransmission();
}

float calculateRPM(uint8_t sensorIndex) {
    unsigned long currentTime = millis();
    float currentAngle = as5600.readAngle() * (360.0 / 4096.0);  // Convert raw to degrees
    float rpm = 0;

    if (currentTime > lastTime[sensorIndex]) {
        float deltaAngle = currentAngle - lastAngle[sensorIndex];

        // Handle rollover (360° to 0°)
        if (deltaAngle < -180) deltaAngle += 360;
        if (deltaAngle > 180) deltaAngle -= 360;

        float deltaTime = (currentTime - lastTime[sensorIndex]) / 1000.0;
        rpm = (deltaAngle / deltaTime) * (60.0 / 360.0);  // Convert to RPM

        lastAngle[sensorIndex] = currentAngle;
        lastTime[sensorIndex] = currentTime;
    }
    
    return abs(rpm);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (uint8_t i = 0; i < 2; i++) {
      selectMuxChannel(i);  // Select the AS5600 sensor channel
      if (!as5600.begin()) {
          Serial.print("AS5600 on channel ");
          Serial.print(i);
          Serial.println(" not found!");
      } else {
          Serial.print("AS5600 on channel ");
          Serial.print(i);
          Serial.println(" initialized!");
      }
  }
}

void loop() {
    for (uint8_t i = 0; i < 2; i++) {
        selectMuxChannel(i);  // Select the appropriate channel
        float rpm = calculateRPM(i);
        Serial.print("Encoder ");
        Serial.print(i + 1);
        Serial.print(" RPM: ");
        Serial.println(rpm);
    }

    delay(100);
}