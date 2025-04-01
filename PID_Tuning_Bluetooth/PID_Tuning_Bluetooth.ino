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
char userInput;

float kg = 0.9; //gyroscope weight
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
float x, y, z,ax,ay,az;
float prev_degacc=0; //previous acceleration angle

// values that work somwhat  kp = 31-32(31.5), kd = 4-5 (4.16), ki=0 ; 

//Global Varibles for the PID
float kp = 0;
float ki = 0;
float kd = 0;
float kf = 0;  //encoder rpm weight
int Max_PID = 255;
int result=0;
float new_error = 0;
float previous_error = 0;
float ki_angle=0; // angle within which integral = 0
float noresponse_angle=0; //angle within which pwm = 0

unsigned long previous_T = 0;
float integral = 0;
float previous_angle = 0;
float Tilt_Angle = 0;

unsigned long timer_value = 0;


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

        //Gyroscope
        if (IMU.gyroscopeAvailable()) {
          IMU.readGyroscope(x, y, z); 
          degx = degt + x*tm ;
        }

      /*if(Serial.available()> 0){ 
    
      userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 

          Serial.print(degt);
          Serial.print(",");
          Serial.print(degx);
          Serial.print(",");
          //Serial.print(prev_degacc);
          //Serial.print(",");
          Serial.println(degacc);
      }
    }*/

      // Accelerometer 
        if (IMU.accelerationAvailable()) {

          IMU.readAcceleration(ax, ay, az);
          prev_degacc = degacc;
          degacc= -1*atan(ay/az)*180/PI;

          if ((degacc-prev_degacc) > 10 || (degacc-prev_degacc)< 10 ){   // worked somewhat by setting both side as 10 instead of 10 and -10

            degt=kg*degx+ka*degacc;
          }

          else{
            degt=kg2*degx+(1-kg2)*degacc;
          }
        }
   
        //degt=kg*degx+ka*degacc;

        float current_T = micros();
        float dT = (current_T - previous_T)/1000000.0;
        previous_T = current_T;

        previous_error = new_error;

        new_error = wanted_angle - degt;

        integral += new_error * dT;
        integral = constrain(integral,-30.0,30.0);

        float derivative = (new_error- previous_error)/dT;
        rpm1 = calculateRPM();

        if (new_error< ki_angle && new_error> ki_angle*(-1)){
          integral=0;
        }

        result = (kp * new_error) + (ki * integral) + (kd * derivative) - kf*(rpm1);


        /*if( new_error<noresponse_angle && new_error>noresponse_angle*(-1)){
         result = 0;
        }*/

        if(new_error >= noresponse_angle){
          result = result - noresponse_angle;

        }

        else if(new_error < -noresponse_angle){

          result = result + noresponse_angle;

        }


    
        if(result > Max_PID){
          result = Max_PID;
        } 
        
        else if(result < -Max_PID){
              result = -Max_PID;
        }
        
        if(result > 0){
          PWM_A = PWM_B = result;
          /*
          analogWrite(left_1, PWM_A);
          analogWrite(left_2, 0);
          analogWrite(right_1, PWM_B);
          analogWrite(right_2, 0);
          */
          
          analogWrite(left_1, 255);
          analogWrite(left_2, 255-PWM_A);
          analogWrite(right_1, 255);
          analogWrite(right_2, 255-PWM_B);
        }
        
        else {
          PWM_A = PWM_B = -result;
          analogWrite(left_1, 255-PWM_A);
          analogWrite(left_2, 255);
          analogWrite(right_1, 255-PWM_A);
          analogWrite(right_2, 255);
          
        }
          
          //sprintf(sendbuffer,"degt: %.2f", degt);
          //customCharacteristic.writeValue(sendbuffer);  


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

          Max_PID = Max_PID + 25;

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

        else if (strcmp(receivedString, "Z") == 0) {                                   //Change Folder button pressed

          kg2 = kg2 + 0.03;

          if(kg2>1.0001){
            kg2=0;
          }
          sprintf(sendbuffer,"tiltang: %.2f",kg2);
          customCharacteristic.writeValue(sendbuffer);    

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
//        Serial.print("PWM_A Value: ");
//        Serial.print(PWM_A);
//        Serial.print("\tPWM_B Value: ");
//        Serial.print(PWM_B);
        Serial.print("kp*error Value: ");
        Serial.print(kp*new_error);
        Serial.print("\tkd*derivative Value: ");
        Serial.print(kd*derivative);
        Serial.print("\tki*integral Value: ");
        Serial.print(ki*integral);
        Serial.print("\tresult: ");
        Serial.print(result);
        Serial.print("\tTilt Angle: ");
        Serial.println(degt);
      */
      
    }
  }
}


float calculateRPM() {
    unsigned long currentTime = millis();
    float currentAngle = as5600.readAngle() * (360.0 / 4096.0);  // Convert raw to degrees
    float rpm = 0;

    if (currentTime > lastTime) {
        float deltaAngle = currentAngle - lastAngle;

        // Handle rollover (360° to 0°)
        if (deltaAngle < -180) deltaAngle += 360;
        if (deltaAngle > 180) deltaAngle -= 360;

        float deltaTime = (currentTime - lastTime) / 1000.0;
        rpm = (deltaAngle / deltaTime) * (60.0 / 360.0);  // Convert to RPM

        lastAngle = currentAngle;
        lastTime = currentTime;
    }
    
    return rpm;
}
