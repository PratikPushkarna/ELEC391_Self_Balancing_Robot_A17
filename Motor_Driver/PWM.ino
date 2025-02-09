
/*
  Fading

  This example shows how to fade an LED using the analogWrite() function.

  The circuit:
  - LED attached from digital pin 9 to ground through 220 ohm resistor.

  created 1 Nov 2008
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Fading
*/

int BOUT1 = 8;  // PWM from digital pin 8
int BOUT2 = 9;  // PWM from digital pin 9
int buttonPin = 7; // Button connected to digital pin 7
int pwmValue = 0;  // Initial PWM value

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Use INPUT_PULLUP if no external pull-down resistor
   Serial.begin(9600); // Initialize serial communication
}

void loop() {

  if (digitalRead(buttonPin) == LOW) { // Button pressed (LOW if using INPUT_PULLUP)
    pwmValue += 20; // Increase PWM by 20
    if (pwmValue > 255) {
      pwmValue = 0; // Cap at max PWM value
    }
    analogWrite(BOUT1, pwmValue);
    analogWrite(BOUT2, pwmValue);
    Serial.print("PWM Value: ");
    Serial.println(pwmValue); // Print PWM value to serial monitor
    delay(200); // Debounce delay
  }

  
            
  /*

  
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }*/




}
