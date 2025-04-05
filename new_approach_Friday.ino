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

int left_1 = 3;                                                      // PWM from digital pin 3
int left_2 = 4;                                                      // PWM from digital pin 4

int right_1 = 5;                                                     // PWM from digital pin 5
int right_2 = 6;                                                     // PWM from digital pin 6


//Parameter Declarations

float PWM_A = 0;                                                     // Initial PWM value A
float PWM_B = 0;                                                     // Initial PWM value B

float speed = 0.85;

char sendbuffer[100];
char userInput;

float kg = 0.95; //gyroscope weight
float ka = 0.1; //accelerometer weight
float tm = 1/99.84;  //sample rate
float degx= 0; // gyroscope angle
float degy = 0; //not used
float degz= 0; //not used
float degacc= 0; //accelerometer angle
float degt = 0; // tilt angle
float gyro_angle; //not used
float wanted_angle = 0.6;
float kg2= 0.6; //gyroscope weight
float gx, gy, gz,ax,ay,az;
float prev_degacc=0; //previous acceleration angle
float filter_angle = 0; // angle change within which weights kg2 are used and outside it  kg is used
int noresponse_flag = 0; // if flag==1 then result =0 within no response angle

// values that work somwhat  kp = 31-32(31.5), kd = 4-5 (4.16), ki=0 ; 

//Global Varibles for the PID
float kp = 0;
float ki = 0;
float kd = 0;
float derivative;
float kf = 0;  //encoder rpm weight
int Max_PID = 255;
int result=0;
float new_error = 0;
float prev_error = 0;
float ki_angle=0; // angle within which integral = 0
float noresponse_angle=0; //angle within which pwm = 0
float maxintegral = 300; // max value which the integral is constrained to


unsigned long previous_T = 0;
float integral = 0;
float previous_angle = 0;
float Tilt_Angle = 0;

unsigned long timer_value = 0;

//new variables
int targetOffset=0;
int turnOffset=0;


//Encoder
AS5600 as5600;  // Create an instance of the AS5600
#define AS5600_ADDR 0x36  // I2C address of AS5600

unsigned long lastTime = 0;
uint16_t lastAngle = 0;
float rpm1=0;


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

  Serial.print("Gyroscope sample rate = ");
  Serial.println(IMU.gyroscopeSampleRate());

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
  Wire.begin();

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    degacc=-atan(ay/az)*180/PI;
  }
  
}

void loop() {

  BLEDevice central = BLE.central();

  char receivedString[100];

  if (central) { 

    while (central.connected()) {                                                       //Bluetooth Inpu

      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        degacc = atan2(ay, az) * (-180.0 / PI);
        degt = kg * (degt + gx * deltaTime) + (1 - kg) * degacc;

        prev_error = new_error;
        new_error = degt - (wanted_angle + targetOffset);

        //P = Kp * Error;
        integral += ki * new_error * deltaTime;
        integral = constrain(integral, -maxintegral, maxintegral);
        


        derivative = kd * (new_error - prev_error) / deltaTime;

        result = kp*new_error + integral + derivative;



        // Adjust each wheel speed
        int leftSpeed = result + turnOffset;
        int rightSpeed = result - turnOffset;

        leftSpeed = constrain(leftSpeed, -255, 255);
        rightSpeed = constrain(rightSpeed, -255, 255);

        // Apply to motors
        setMotor(left_1, left_2, leftSpeed);
        setMotor(right_1, right_2, rightSpeed);
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
        }

        else if (strcmp(receivedString, "B") == 0) {                                   //Backward button pressed
          kp = kp - 0.1;
        }

        else if (strcmp(receivedString, "R") == 0) {   
          kp = kp + 1;
        }

        else if (strcmp(receivedString, "L") == 0) {                                    //Left button pressed

          kp = kp -1;
        }

        else if (strcmp(receivedString, "A") == 0) {         
          
          kd = kd + 0.02;
        }

        else if (strcmp(receivedString, "B2") == 0) {                                   //Change Folder button pressed
          kd = kd - 0.02;
        }

        else if (strcmp(receivedString, "C") == 0) {                                   //Change Song button pressed
          kd=kd+1; ;

          if (kd>20){
              kd=0;
          }

        }

        else if (strcmp(receivedString, "D") == 0) {                                   //Change Folder button pressed
          kd = kd - 1;
        }
        
        else if (strcmp(receivedString, "E") == 0) {                                   //Change Folder button pressed
          kg = kg + 0.03;

          if(kg>1.0001){
            kg=0;
          }

          ka=1-kg;
          sprintf(sendbuffer,"ga: %.2f  %.2f", kg, ka);
          customCharacteristic.writeValue(sendbuffer);  
        }

        else if (strcmp(receivedString, "F2") == 0) {                                   //Change Folder button pressed
          ki = ki +10 ;
        }

        else if (strcmp(receivedString, "G") == 0) {                                   //Change Folder button pressed
          ki = ki - 10;
        }

        else if (strcmp(receivedString, "H") == 0) {                                   //Change Folder button pressed
          ki = ki + 1;
        }

        else if (strcmp(receivedString, "I") == 0) {                                   //Change Folder button pressed
          ki = ki - 1;
        }
        
        else if (strcmp(receivedString, "K") == 0) {                                   //Change Folder button pressed

          wanted_angle = wanted_angle + 0.05;
          
          sprintf(sendbuffer,"wa: %.2f",wanted_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "L2") == 0) {                                   //Change Folder button pressed
          wanted_angle = wanted_angle - 0.05;
          
          sprintf(sendbuffer,"wa: %.2f",wanted_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }


        else if (strcmp(receivedString, "M") == 0) {                                   //Change Folder button pressed
          kf = kf + 0.02;

          sprintf(sendbuffer,"enc: %.2f %.2f",kf,rpm1);
          customCharacteristic.writeValue(sendbuffer); 
        }
  
        else if (strcmp(receivedString, "N") == 0) {                                   //Change Folder button pressed
          kf = kf - 0.02;
          sprintf(sendbuffer,"enc: %.2f %.2f",kf,rpm1);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "O") == 0) {                                   //Change Folder button pressed
          kf = kf + 0.5;
          sprintf(sendbuffer,"enc: %.2f %.2f",kf,rpm1);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "P") == 0) {                                   //Change Folder button pressed
          kf = kf - 0.5;

          sprintf(sendbuffer,"enc: %.2f %.2f",kf,rpm1);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "Q") == 0) {                                   //Change Folder button pressed

          Max_PID = Max_PID + 5;

          if(Max_PID>255){
            Max_PID = 0;
          }

          sprintf(sendbuffer,"PWM: %d",Max_PID);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "R2") == 0) {                                   //Change Folder button pressed
          ki_angle = ki_angle + 0.5;
          
          sprintf(sendbuffer,"kia: %.2f",ki_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "S") == 0) {                                   //Change Folder button pressed

          ki_angle = ki_angle - 0.5;
          
          sprintf(sendbuffer,"kia: %.2f",ki_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "T") == 0) {                                   //Change Folder button pressed
          ki_angle = ki_angle + 5;
          
          sprintf(sendbuffer,"kia: %.2f",ki_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "U") == 0) {                                   //Change Folder button pressed

          ki_angle = ki_angle - 5;
          
          sprintf(sendbuffer,"kia: %.2f",ki_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "V") == 0) {                                   //Change Folder button pressed

          noresponse_angle = noresponse_angle + 0.05;
          
          sprintf(sendbuffer,"nra: %.2f",noresponse_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "W") == 0) {                                   //Change Folder button pressed

          noresponse_angle = noresponse_angle - 0.05;
          
          sprintf(sendbuffer,"nra: %.2f",noresponse_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "X") == 0) {                                   //Change Folder button pressed
          kp=0;
          kd=0;
          ki=0;
          sprintf(sendbuffer,"tiltang: %.2f",degt);
          customCharacteristic.writeValue(sendbuffer);
           
        }

        else if (strcmp(receivedString, "Y") == 0) {                                   //Change Folder button pressed
          sprintf(sendbuffer,"degt res: %.2f %d",degt,result);
          customCharacteristic.writeValue(sendbuffer);    

        }


        else if (strcmp(receivedString, "NUM1") == 0) {                                   //Change Folder button pressed

          Max_PID = Max_PID - 5;
          sprintf(sendbuffer,"pwm: %d",Max_PID);
          customCharacteristic.writeValue(sendbuffer);    

        }

        else {

          Serial.print("receivedString has other data\t");
          Serial.println(receivedString);
        }
        
        sprintf(sendbuffer," %.2f  %.2f  %.2f", kp, kd, ki);
        customCharacteristic.writeValue(sendbuffer);                               // Optionally, respond by updating the characteristic's value
      }
      
    }
  }
}

void setMotor(int pinForward, int pinBackward, int speed) {
  if (speed >= 0) {
    analogWrite(pinForward, speed);
    analogWrite(pinBackward, 0);
  } else {
    analogWrite(pinForward, 0);
    analogWrite(pinBackward, -speed);
  }
}
