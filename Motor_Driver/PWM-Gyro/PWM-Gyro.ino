// A17
// Proof of Concepts demo

#include "math.h"
#include "Arduino_BMI270_BMM150.h"

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

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    
}

float check_angle(){
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
    return degt;
}

int PID(float gyro_angle,float wanted_angle){
    float Kp = 2.2;
    float Ki = 10;
    float Kd = 0.019;
    int Max_PID = 255;

    float current_T = millis();
    float dT = current_T - previous_T;
    previous_T = current_T;

    float error = wanted_angle - gyro_angle;

    integral += error * dT;

    float derivative = (previous_angle-gyro_angle)/dT;
    previous_angle = gyro_angle;

    int result = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if(result > Max_PID){
        result = Max_PID;
    } else if(result < -Max_PID){
        result = Max_PID;
    }
    return result;
}

void loop() {
    float gyro_angle;
    float wanted_angle = 0;

    if(millis()-timer_value>10){
      timer_value = millis();
      gyro_angle = check_angle();

      PWM_A = PWM_B = PID(gyro_angle, wanted_angle);

      if(gyro_angle > 0){
        analogWrite(left_1, PWM_A);
        analogWrite(left_2, 0);
        analogWrite(right_1, PWM_B);
        analogWrite(right_2, 0);
      } else {
        analogWrite(left_1, 0);
        analogWrite(left_2, PWM_A);
        analogWrite(right_1, 0);
        analogWrite(right_2, PWM_B);
      }
    }

    
}
