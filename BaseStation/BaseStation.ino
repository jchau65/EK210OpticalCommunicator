#include <LiquidCrystal_I2C.h> //Include Library for the LCD screen
#include <IRremote.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

float bearing = -45.0;

// state variable to determine whether our motor is going clockwise or not
bool clockwise = true;

// state variable to determine whether or not the base station has found the
bool found = false; 

void setup() {
  // set the speed at 10 rpm:
  myStepper.setSpeed(10);
  // initialize the serial port:
  Serial.begin(9600);
   // initialize the lcd
  lcd.init();
  lcd.backlight();
  myStepper.step(-stepsPerRevolution/8);
}

void loop() {
  if (!found) {
    if (clockwise) {
      myStepper.step(stepsPerRevolution/720);
      bearing += 0.5;
      if (bearing >= 45.0) {
        clockwise = false;
      }
    }

    else if (!clockwise) {
      myStepper.step(-stepsPerRevolution/720);
      bearing -= 0.5;
      if (bearing <= -45.0) {
        clockwise = true;
      }
    }

    if (Serial.available() > 0) {
      found = (Serial.readStringUntil('\n') == "FOUND");
      lcd.print(bearing);
    }
  }

  else if (found) {
    lcd.clear();
    lcd.print(bearing);
  }
  
}