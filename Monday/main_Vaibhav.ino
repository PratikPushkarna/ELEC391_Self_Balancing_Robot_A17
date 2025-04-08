// A17
// Self Balancing Robot

#include <ArduinoBLE.h>
#include <Wire.h>
#include <AS5600.h>
#include "math.h"
#include "Arduino_BMI270_BMM150.h"
#include "string.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

#define BUFFER_SIZE 20

#define TRIGGER_PIN 7                                                 // Define Trigger pin
#define ECHO_PIN 8                                                    // Define Echo pin

#define SERVO_PIN 9                                                   // Define Servo Pin

/* Servo */
Servo myservo;                                                        // create servo object to control a servo
int pos = 0;                                                          // variable to store the servo position

/* Sonar */
float distance = 0 ;
float temp_distance = 0;
int sonar_enable_flag = 1;
int sonar_counter = 0;



BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");    
BLECharacteristic customCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false); // Define a custom BLE service and characteristic

/* Motors */
//Pin Declarations

int left_1 = 5;                             //white                         // PWM from digital pin 3
int left_2 = 6;                                                      // PWM from digital pin 4

int right_1 = 3;                                 //blue                    // PWM from digital pin 5
int right_2 = 4;                                                     // PWM from digital pin 6


//Parameter Declarations

float PWM_A = 0;                                                     // Initial PWM value A
float PWM_B = 0;                                                     // Initial PWM value B

float speed = 0.85;

char sendbuffer[100];
char userInput;

float kg = 0.95; //gyroscope weight
// float ka = 0.05; //accelerometer weight not used
// float tm = 1/99.84;  //sample rate not used
float degx= 0; // gyroscope angle
//  float degy = 0; //not used
//  float degz= 0; //not used
float degacc= 0; //accelerometer angle
float degt = 0; // tilt angle
float wanted_angle = 1.15;
float kg2= 0.6; //gyroscope weight
float gx, gy, gz,ax,ay,az;



// values that work somwhat  kp = 31-32(31.5), kd = 4-5 (4.16), ki=0 ; 

//Global Varibles for the PID
float kp = 13;
float ki = 150;
float kd = 1.44;
float derivative;
float kf = 0;  //encoder rpm weight
int Max_PWM = 300;
int result=0;
float new_error = 0;
float prev_error = 0;
float ki_angle=0; // angle within which integral = 0
float noresponse_angle=0; //angle within which pwm = 0
float maxintegral = 200; // max value which the integral is constrained to
float leftwheel_gain = 0.9;

unsigned long previous_T = 0;
float integral = 0;
float previous_angle = 0;
float Tilt_Angle = 0;

unsigned long timer_value = 0;

//movement variables and constants
float offset_angle = 0;
float offset_turn = 0;
float offset_movement = 0;

unsigned long direction_counter = 0;
unsigned long direction_delay = 200;

#define Fangle_offset 1
#define Bangle_offset -1.3
#define F_offset 15
#define B_offset 15
#define Rturn_offset -15
#define Lturn_offset 10

unsigned long lastTime = 0;

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

  //Serial1.begin(9600); // DFP player initialisation

 // myservo.attach(SERVO_PIN); // Servo initialisation

  //Serial.println("BluetoothÂ® device active, waiting for connections...");

  myservo.attach(SERVO_PIN); // Servo initialisation

  pinMode(TRIGGER_PIN, OUTPUT);  // Servo Initialisation
  pinMode(ECHO_PIN, INPUT);       // Servo Initialisation

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

    while (central.connected()) {                                                       //Bluetooth Input

      digitalWrite(LED_BUILTIN, HIGH);                                                  // Turn on LED to indicate connection                                                       // Keep running while connected


      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        degacc = atan2(ay, az) * (-180.0 / PI);
        degt = kg * (degt + gx * deltaTime) + (1 - kg) * degacc;

        prev_error = new_error;

        if ((millis() - direction_counter) > direction_delay) {

          offset_angle = 0;
          offset_turn = 0;
        }

        new_error = degt - (wanted_angle + offset_angle);

        //P = Kp * Error;
        integral +=  ki*new_error * deltaTime;
        integral =  constrain(integral, -maxintegral, maxintegral);
        
        if (new_error < ki_angle && new_error > -ki_angle ){
            integral = 0;
        }

        if (new_error > 15 || new_error < -15){
          derivative = 0.2 * (new_error - prev_error) / deltaTime;
        }

        else{
          derivative = kd * (new_error - prev_error) / deltaTime;
        }

        result = kp*new_error + integral + derivative;

        if (new_error < noresponse_angle && new_error > -noresponse_angle){
          result = 0;
        }

        // Adjust each wheel speed
        int leftSpeed = result*leftwheel_gain + offset_turn;
        int rightSpeed = result - offset_turn;

        leftSpeed = constrain(leftSpeed, -Max_PWM, Max_PWM);
        rightSpeed = constrain(rightSpeed, -Max_PWM, Max_PWM);

        // Apply to motors
        setMotor(left_1, left_2, leftSpeed);
        setMotor(right_1, right_2, rightSpeed);
      }

      if (sonar_enable_flag == 1){

        temp_distance = getDistance();

        if(temp_distance > 0){
          distance = temp_distance;
        }

        sonar_counter++;

        if (sonar_counter == 20){

          sprintf(sendbuffer,"dist: %.2f",distance);
          customCharacteristic.writeValue(sendbuffer);

          sonar_counter = 0;
          } 
        
        //sprintf(sendbuffer,"dist: %.2f",distance);
      }

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
          offset_angle = Fangle_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "B") == 0) {                                   //Backward button pressed
          offset_angle = Bangle_offset;
          //offset_movement = B_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "R") == 0) {   
          offset_turn = Rturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "L") == 0) {                                    //Left button pressed

          offset_turn = Lturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "FR") == 0) {                                         //Forward button pressed
          offset_angle = Fangle_offset;
          offset_turn = Rturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "FL") == 0) {                                   //Backward button pressed
          offset_angle = Fangle_offset;
          offset_turn = Lturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "BR") == 0) {   
          offset_angle = Bangle_offset;
          offset_turn = Rturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "BL") == 0) {                                    //Left button pressed
          offset_angle = Bangle_offset;
          offset_turn = Lturn_offset;

          direction_counter = millis();
        }

        else if (strcmp(receivedString, "S") == 0) {                                   //Change Folder button pressed

          offset_angle = 0;
          offset_turn = 0;
          
          sprintf(sendbuffer,"kia: %.2f",ki_angle);
          customCharacteristic.writeValue(sendbuffer); 
        }

            else if (strcmp(receivedString, "T1") == 0) {                                   //Change Folder button pressed
            pos = pos - 15;
            myservo.write(pos);
            pos = constrain(pos, 0, 180);
          
            sprintf(sendbuffer,"position: %d",pos);
            customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "T2") == 0) {                                   //Change Folder button pressed
          
          pos = pos + 15;

          pos = constrain(pos, 0, 180);
          myservo.write(pos);
          
          sprintf(sendbuffer,"position: %d",pos);
          customCharacteristic.writeValue(sendbuffer); 
        }

        else if (strcmp(receivedString, "SONAR_OFF") == 0) {                                   //Change Folder button pressed
          
          sonar_enable_flag = 0;
          
          sprintf(sendbuffer,"s disabled", sonar_enable_flag);
          customCharacteristic.writeValue(sendbuffer); 
        } 

        else if (strcmp(receivedString, "SONAR_ON") == 0) {                                   //Change Folder button pressed
          
          sonar_enable_flag = 1;
          
          sprintf(sendbuffer,"s enabled %d", sonar_enable_flag);
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

/**
 * Custom pulseIn function replacement (portable version)
 * 
 * @param pin The pin number to read pulses from
 * @param state The pulse state to measure (HIGH or LOW)
 * @param timeout The maximum time to wait for the pulse in microseconds (default 1 second)
 * @return The pulse duration in microseconds, or 0 if timeout occurred
 */
unsigned long customPulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL) {
    // Normalize state to HIGH or LOW
    state = (state == HIGH) ? HIGH : LOW;
    
    unsigned long startMicros = micros();
    unsigned long maxMicros = startMicros + timeout;
    
    // Wait for any previous pulse to end
    while (digitalRead(pin) == state) {
        if (micros() > maxMicros) return 0;
    }
    
    // Wait for the pulse to start
    while (digitalRead(pin) != state) {
        if (micros() > maxMicros) return 0;
    }
    
    unsigned long pulseStart = micros();
    
    // Wait for the pulse to end
    while (digitalRead(pin) == state) {
        if (micros() > maxMicros) return 0;
    }
    
    return micros() - pulseStart;
}

float getDistance() {                                                       // Send a short pulse to trigger the ultrasonic sensor
    
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    //long duration = pulseIn(ECHO_PIN, HIGH, 10000);                                // Read the time it takes for the echo to return

    //long duration = 1;                                // commenting out the PulseIn function and using this as an example makes the whole thing work

    long duration = customPulseIn (ECHO_PIN, HIGH, 10000);
    float distance = (duration * 0.0343) / 2;                               // Convert time to distance (Speed of sound is ~343 m/s or 0.0343 cm/us)

    return distance;
    
    
    
}

