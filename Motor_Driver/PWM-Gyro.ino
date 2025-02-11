// A17
// Proof of Concepts demo

#include "math.h"
#include "Arduino_BMI270_BMM150.h"

int pwm_lf_button = 6;                                                // Button to control PWM rate for Motor A
int pwm_lb_button = 8;                                                // Button to control PWM rate for Motor B

int pwm_rf_button = 10;                                               // Button to control PWM rate for Motor A
int pwm_rb_button = 12;                                               // Button to control PWM rate for Motor B

float PWM_A = 0;                                                      // Initial PWM value A
float PWM_B = 0;                                                      // Initial PWM value B

int left_1 = 2;                                                      // PWM from digital pin 2
int left_2 = 3;                                                         // PWM from digital pin 3

int right_1 = 4;                                                         // PWM from digital pin 4
int right_2 = 5;                                                         // PWM from digital pin 5

void setup() {

  pinMode(pwm_lf_button, INPUT_PULLUP);                               // Use INPUT_PULLUP if no external pull-down resistor
  pinMode(pwm_lb_button, INPUT_PULLUP);                               // Use INPUT_PULLUP if no external pull-down resistor
  pinMode(pwm_rf_button, INPUT_PULLUP);                               // Use INPUT_PULLUP if no external pull-down resistor
  pinMode(pwm_rb_button, INPUT_PULLUP);                               // Use INPUT_PULLUP if no external pull-down resistor

  Serial.begin(9600);                                                 // Initialize serial communication
}

float check_angle(){
    float tm = 1/99.84;
    float degx= 0;
    float degy = 0;
    float degz= 0;
    float degacc= 0;
    float degt = 0;
    float kg= 0.5; //gyroscope weight
    float ka= 0.5; //accelerometer weight
    float x, y, z,ax,ay,az;

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

void loop() {
    float gyro_angle;

    gyro_angle = check_angle();

    if(gyro_angle > 0){
        PWM_A = PWM_B = gyro_angle*255.0/90.0;
        analogWrite(left_1, PWM_A);
        analogWrite(left_2, 0);
        analogWrite(right_1, PWM_B);
        analogWrite(right_2, 0);
    } else {
        PWM_A = PWM_B = (-gyro_angle)*255.0/90.0;
        analogWrite(left_1, 0);
        analogWrite(left_2, PWM_A);
        analogWrite(right_1, 0);
        analogWrite(right_2, PWM_B);
    }

    Serial.print(PWM_A);
    Serial.print('\t');
    Serial.print(PWM_B);
    Serial.print('\t');
    Serial.println(gyro_angle);
}
