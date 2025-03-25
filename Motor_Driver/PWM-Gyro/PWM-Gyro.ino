// A17
// Proof of Concepts demo

#include "math.h"
#include "Arduino_BMI270_BMM150.h"
#include "string.h"

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

void setup() {

    Serial.begin(9600);                                                 // Initialize serial communication

    while (!Serial);
    Serial.println("Started");

    pinMode(left_1, OUTPUT);
    pinMode(left_2, OUTPUT);
    pinMode(right_1, OUTPUT);
    pinMode(right_2, OUTPUT);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    
}

/*float PID(float wanted_angle, float* angle){
    float x, y, z,ax,ay,az;
    float kg = 0.5; //gyroscope weight
    float ka = 0.5; //accelerometer weight

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

    float Kp = 12;
    float Ki = 0;
    float Kd = 0;
    int Max_PID = 255;

    float current_T = millis();
    float dT = current_T - previous_T;
    previous_T = current_T;

    float error = wanted_angle - degt;

    integral += error * dT;

    float derivative = x;

    int result = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if(result > Max_PID){
        result = Max_PID;
    } else if(result < -Max_PID){
        result = Max_PID;
    }
    *angle = degt;
    return result;
}*/

float Kp = 18; // 18 27
float Ki = 100; //100
float Kd = 1.35; // 1.35 3.4
int Max_PID = 255;

void loop() {
    float gyro_angle;
    float wanted_angle = -0.39;

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
    gyro_angle = degt;
    
      //PWM_A = PWM_B = PID(wanted_angle, &gyro_angle);
      //if(gyro_angle<-4.0 || gyro_angle>4.0){
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
      //} else{
      //  analogWrite(left_1, 0);
      //  analogWrite(left_2, 0);
      //  analogWrite(right_1, 0);
      //  analogWrite(right_2, 0);
      //}
      Serial.print(gyro_angle);
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
