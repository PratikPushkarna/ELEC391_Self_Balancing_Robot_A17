#include <Wire.h>
#include <AS5600.h>
#include "math.h"
#include "Arduino_BMI270_BMM150.h"
#include "string.h"

#define TCA_ADDR 0x70  // I2C address of TCA9548A
#define AS5600_ADDR 0x36  // I2C address of AS5600 

float PWM_A = 0;                                                      // Initial PWM value A
float PWM_B = 0;                                                      // Initial PWM value B

int left_1 = 2;                                                      // PWM from digital pin 2
int left_2 = 3;                                                         // PWM from digital pin 3

int right_1 = 4;                                                         // PWM from digital pin 4
int right_2 = 5;                                                         // PWM from digital pin 5

float tm = 1/99.84;
float degx= 0;
float degy = 0;
float degz= 0;
float degacc= 0;
float degt = 0;

//Global Varibles for the PID
unsigned long previous_T = 0;
float integral = 0;
float previous_angle = 0;

unsigned long timer_value = 0;

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
  //Setup for the Encoder
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

  //Setup for the motor pins
  pinMode(left_1, OUTPUT);
  pinMode(left_2, OUTPUT);
  pinMode(right_1, OUTPUT);
  pinMode(right_2, OUTPUT);

  if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
  }
}

float Kp = 34.5; // 18 27
float Ki = 1; //100
float Kd = 3.35; // 1.35 3.4
int Max_PID = 255;

void loop() {
  //Obtain RPM values
  selectMuxChannel(0);  // Select the appropriate channel
  float rpm0 = calculateRPM(0);
  selectMuxChannel(1);  // Select the appropriate channel
  float rpm1 = calculateRPM(1);

  float gyro_angle;
  float wanted_angle = -0.9;

  float x, y, z,ax,ay,az;
  float kg = 0.9; //gyroscope weight
  float ka = 0.1; //accelerometer weight

  if(Serial.available()>0){
    String str1;
    char a[] = "Kp";
    char b[] = "Ki";
    char c[] = "Kd";
    str1 = Serial.readString();
    int len = str1.length() + 1;
    char str [len];
    str1.toCharArray(str,len);

    if(strcmp(str,a)==0){
      while(Serial.available() == 0);
      if(Serial.available()> 0 ){
        Kp = Serial.parseFloat();
      }
    }else if(strcmp(str,b)==0){
      while(Serial.available() == 0);
      if(Serial.available()> 0 ){
        Ki = Serial.parseFloat();
      }
    }else if(strcmp(str,c)==0){
      while(Serial.available() == 0);
      if(Serial.available()> 0 ){
        Kd = Serial.parseFloat();
      }
    }
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z); 
    degx = degt + x*tm ;
  }

// Accelerometer

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float degacc= atan(ay/az)*180/PI;
    
    degt=kg*degx+ka*degacc;
  }

  float current_T = micros();
  float dT = (current_T - previous_T)/100000.0;
  previous_T = current_T;

  float error = wanted_angle - degt;

  integral += error * dT;
  integral = constrain(integral,-30.0,30.0);

  float derivative = x;

  int result = (Kp * error) + (Ki * integral) + (Kd * derivative);

  if(result > Max_PID){
    result = Max_PID;
  } else if(result < -Max_PID){
    result = -Max_PID;
  }
  if(result > 0){
    PWM_A = PWM_B = result;
    analogWrite(left_1, PWM_A);
    analogWrite(left_2, 0);
    analogWrite(right_1, PWM_B);
    analogWrite(right_2, 0);
  } else {
    PWM_A = PWM_B = -result;
    analogWrite(left_1, 0);
    analogWrite(left_2, PWM_A);
    analogWrite(right_1, 0);
    analogWrite(right_2, PWM_B);
  }

  Serial.print(degt);
  Serial.print('\t');
  Serial.print(PWM_A);
  Serial.print('\t');
  Serial.print(PWM_B);
  Serial.print('\t');
  Serial.print(Kp);
  Serial.print('\t');
  Serial.print(Ki);
  Serial.print('\t');
  Serial.println(Kd);
}
