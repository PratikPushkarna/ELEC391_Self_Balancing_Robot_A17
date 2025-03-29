// A17
// Self Balancing Robot
//Change the PID code if needed


#include "math.h"
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>

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

unsigned long previous_T = 0;                                        //Global Varibles for the PID
float integral = 0;
float previous_angle = 0;

unsigned long timer_value = 0;


char sendbuffer[100];


float kp = 5.5;
float ki = 0;
float kd = 0;
int Max_PID = 255;

void setup() {

  Serial.begin(9600);                                               // Initialize serial communication

  while(!Serial);
  pinMode(LED_BUILTIN, OUTPUT);                                     // Connection Status

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.println(IMU.gyroscopeSampleRate());

  BLE.setLocalName("BLE-DEVICE-A17");                                                 // Set the device name and local name
  BLE.setDeviceName("BLE-DEVICE-A17");

  customService.addCharacteristic(customCharacteristic);                              // Add the characteristic to the service
  BLE.addService(customService);                                                      // Add the service
  customCharacteristic.writeValue("Waiting for data");                                // Set an initial value for the characteristic

  BLE.advertise();                                                                    // Start advertising the service

  Serial.println("Bluetooth® device active, waiting for connections...");
  
}

void loop() {

  BLEDevice central = BLE.central();

  char receivedString[100];

  float Tilt_Angle = 0;
  if (central) { 

    while (central.connected()) {                                                       //Bluetooth Input

        float TimeSample = 1/IMU.gyroscopeSampleRate();
        float K_gy= 0.5;                                                     // Gyroscope weight
        float K_acc= 0.5;                                                    // Accelerometer weight

        float Gy_x, Gy_y, Gy_Z;
        float w_x;

        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(Gy_x, Gy_y, Gy_Z);
            w_x = Tilt_Angle + Gy_x*TimeSample;
          }

          float ax, ay, az;
          float acc_x;

          if (IMU.accelerationAvailable()) {

            IMU.readAcceleration(ax, ay, az);
            acc_x= atan(ay/az)*180/PI;
          }

          Tilt_Angle = -(K_gy*w_x + K_acc*acc_x)*2;
        
          float wanted_angle = 0;

          if (millis()-timer_value>10) {

            timer_value = millis();
            

            float current_T = millis();
            float dT = current_T - previous_T;
            previous_T = current_T;

            float error = wanted_angle - Tilt_Angle;

            integral += error * dT;

            float derivative = (previous_angle-Tilt_Angle)/dT;
            previous_angle = Tilt_Angle;

            int result = (kp * error) + (ki * integral) + (kd * derivative);

            if(result > Max_PID) {

                result = Max_PID;
            }
            
            else if (result < -Max_PID) {

                result = Max_PID;
            }
            PWM_A = PWM_B = result;
          }
        

        else {

          Serial.print("receivedString has other data\t");
          Serial.println(receivedString);
        }


        if ( PWM_A >= 0 ) {

          analogWrite(left_1, PWM_A);
          analogWrite(left_2, 0);
        }

        else {

          analogWrite(left_1, 0);
          analogWrite(left_2, abs(PWM_A));
        }

        if ( PWM_B >= 0 ) {

          analogWrite(right_1, PWM_B);
          analogWrite(right_2, 0);
        }

        else {

          analogWrite(right_1, 0);
          analogWrite(right_2, abs(PWM_B));
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

          kp = kp + 0.2;
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "B") == 0) {                                   //Backward button pressed

          kp = kp - 0.2;
          
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
          
          kd = kd + 0.04;
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "B2") == 0) {                                   //Change Folder button pressed


          kd = kd - 0.04;
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "C") == 0) {                                   //Change Song button pressed

          kd = 0.39;
          kp = 17;
          
          strcpy(commandString, "N");
        }

        else if (strcmp(receivedString, "N") == 0) {

        
        }
        sprintf(sendbuffer," %.2f  %.2f  %.2f", kp, kd, ki);
        customCharacteristic.writeValue(sendbuffer);                               // Optionally, respond by updating the characteristic's value
      }

      
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
      
  
    }
  }
}
