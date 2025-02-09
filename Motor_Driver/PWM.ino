// A17
// Proof of Concepts demo

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

void loop() {

  if (digitalRead(pwm_lf_button) == LOW) {                            // Button pressed (LOW if using INPUT_PULLUP)

    PWM_A += 255/4;                                                   // Increase PWM by 25%

    if (PWM_A > 255) {

      PWM_A = 0;                                                      // Cap at max PWM value

    }

  }

  if (digitalRead(pwm_lb_button) == LOW) {                            // Button pressed (LOW if using INPUT_PULLUP)

    PWM_A -= 255/4;                                                   // Increase PWM by 25%

    if (PWM_A < -255) {

      PWM_A = 0;                                                      // Cap at max PWM value

    }
  }

  if (digitalRead(pwm_rf_button) == LOW) {                            // Button pressed (LOW if using INPUT_PULLUP)

    PWM_B += 255/4;                                                   // Increase PWM by 25%

    if (PWM_B > 255) {

      PWM_B = 0;                                                      // Cap at max PWM value

    }

  }

  if (digitalRead(pwm_rb_button) == LOW) {                            // Button pressed (LOW if using INPUT_PULLUP)

    PWM_B -= 255/4;                                                   // Increase PWM by 25%

    if (PWM_B < -255) {

      PWM_B = 0;                                                      // Cap at max PWM value

    }
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

  Serial.print("PWM_A Value: ");
  Serial.print(PWM_A);
  Serial.print("\tPWM_B Value: ");
  Serial.println(PWM_B);

  delay(200); // Debounce delay          
}
