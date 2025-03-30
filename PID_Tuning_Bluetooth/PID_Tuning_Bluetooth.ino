// A17
// Self Balancing Robot

#include <ArduinoBLE.h>
#include <Wire.h>
#include <AS5600.h>
#include "math.h"
#include "Arduino_BMI270_BMM150.h"
#include "string.h"

#define BUFFER_SIZE 20

BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");    
BLECharacteristic customCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false); // Define a custom BLE service and characteristic

//Pin Declarations

int left_1 = 2;                                                      // PWM from digital pin 2
int left_2 = 3;                                                      // PWM from digital pin 3

int right_1 = 4;                                                     // PWM from digital pin 4
int right_2 = 5;                                                     // PWM from digital pin 5


//Parameter Declarations

float PWM_A = 0;                                                     // Initial PWM value A
float PWM_B = 0;                                                     // Initial PWM value B

float speed = 0.85;

char sendbuffer[100];

float tm = 1/99.84;
float degx= 0;
float degy = 0;
float degz= 0;
float degacc= 0;
float degt = 0;


//Global Varibles for the PID
float kp = 5.5;
float ki = 0;
float kd = 0;
int Max_PID = 255;
int result=0;

unsigned long previous_T = 0;
float integral = 0;
float previous_angle = 0;

unsigned long timer_value = 0;

AS5600 as5600;  // Create an instance of the AS5600

unsigned long lastTime[2] = {0, 0};
uint16_t lastAngle[2] = {0, 0};



void setup() {

  //Serial.begin(9600);                                               // Initialize serial communication

  //while(!Serial);
  pinMode(LED_BUILTIN, OUTPUT);                                     // Connection Status

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    //Serial.println("Starting BLE failed!");
    while (1);
  }

  //Serial.print("Gyroscope sample rate = ");
  //Serial.println(IMU.gyroscopeSampleRate());

  BLE.setLocalName("BLE-DEVICE-A17");                                                 // Set the device name and local name
  BLE.setDeviceName("BLE-DEVICE-A17");

  customService.addCharacteristic(customCharacteristic);                              // Add the characteristic to the service
  BLE.addService(customService);                                                      // Add the service
  customCharacteristic.writeValue("Waiting for data");                                // Set an initial value for the characteristic

  BLE.advertise();                                                                    // Start advertising the service

  //Serial.println("Bluetooth® device active, waiting for connections...");

  //Setup for the motor pins
  pinMode(left_1, OUTPUT);
  pinMode(left_2, OUTPUT);
  pinMode(right_1, OUTPUT);
  pinMode(right_2, OUTPUT);
  
}

void loop() {

  BLEDevice central = BLE.central();

  char receivedString[100];

  float Tilt_Angle = 0;
  if (central) { 

    while (central.connected()) {                                                       //Bluetooth Input

        float gyro_angle;
        float wanted_angle = -0.5;

        float x, y, z,ax,ay,az;
        float kg = 0.9; //gyroscope weight
        float ka = 0.1; //accelerometer weight
        //Gyroscope
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
        float dT = (current_T - previous_T)/1000000.0;
        previous_T = current_T;

        float error = wanted_angle - degt;

        integral += error * dT;
        integral = constrain(integral,-30.0,30.0);

        float derivative = x;

         result = (kp * error) + (ki * integral) + (kd * derivative);

            /*if( error<-20 || error>20){
              result=0;
            }

            if( error>-1.3 && error<1.3){
              result =(ki * integral) + (kd * derivative);
            }*/

            if( error>-1.3 && error<1.3){
              result =(ki * integral) + (kd * derivative);
            }


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


      digitalWrite(LED_BUILTIN, HIGH);                                                  // Turn on LED to indicate connection                                                       // Keep running while connected
      
      if (customCharacteristic.written()) {                                             // Check if the characteristic was written

        int length = customCharacteristic.valueLength();                                // Get the length of the received data

        const unsigned char* receivedData = customCharacteristic.value();               // Read the received data

        char receivedString[length + 1];                                                // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0';                                                  // Null-terminate the string

        char commandString[2] = "D";

        Serial.print("Received data: ");                                                // Print the received data to the Serial Monitor
        Serial.println(receivedString);


        if (strcmp(receivedString, "F") == 0) {                                         //Forward button pressed

          kp = kp + 0.1;
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "B") == 0) {                                   //Backward button pressed

          kp = kp - 0.1;
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "R") == 0) {   

          kp = kp + 1;
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "L") == 0) {                                    //Left button pressed

          kp = kp -1;
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "A") == 0) {         
          
          kd = kd + 0.02;
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "B2") == 0) {                                   //Change Folder button pressed


          kd = kd - 0.02;
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "C") == 0) {                                   //Change Song button pressed

          kd=kd+1; ;
          
          if (kd>20){

              kd=0;

          }
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "N") == 0) {

        
        }

        else {

          Serial.print("receivedString has other data\t");
          Serial.println(receivedString);
        }
        
        sprintf(sendbuffer," %.2f  %.2f  %.2f", kp, kd, ki);
        customCharacteristic.writeValue(sendbuffer);                               // Optionally, respond by updating the characteristic's value
      }

      /*
        Serial.print("PWM_A Value: ");
        Serial.print(PWM_A);
        Serial.print("\tPWM_B Value: ");
        Serial.print(PWM_B);
        Serial.print("kp Value: ");
        Serial.print(kp);
        Serial.print("kd Value: ");
        Serial.print(kd);
        Serial.print("ki Value: ");
        Serial.print(ki);
        Serial.print("\tTilt Angle: ");
        Serial.println(Tilt_Angle);
      
      */
    }
  }
}
