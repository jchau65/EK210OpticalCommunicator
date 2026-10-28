#include <IRremote.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

// pin for the beacon's emitter (this will be hooked up to a mosfet)
const int emitterPin = 3;

// State variable to determine whether the code has been inputted
bool codeInputted = false;

// State variable to determine whether the beacon has connected
bool connected = false;

// State variable to determine whether code was received
bool received = false;

// State variable to determine whether bearing was received
bool bearingReceived = false;

const byte ROWS = 4;
const byte COLS = 4;

const char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'},
};

byte rowPins[ROWS] = {6, 7, 8, 9};
byte colPins[COLS] = {10, 11, 12, 13};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

LiquidCrystal_I2C lcd(0x27, 16, 2);

int digits = 0;

String bearing;

// Variables needed for after the code has been inputted
int dotIterator = 0;

char code[11] = {'*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '\0'};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(3, OUTPUT);

  IrSender.begin(emitterPin);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print(code);
}

void loop() {
  // Section that runs before the code is inputted
  if (!codeInputted) {
    // Variable that stores a char when the keypad is pressed 
    char key = keypad.getKey();

    // Run this section only when a key is pressed and the code isn't fully inputted yet
    if (key && digits < 10) {
      code[digits] = key;
      digits++;

      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print(code);
    }

    // Run this section when the code is fully inputted
    else if (digits == 10) {
      codeInputted = true; // Switch states to PINGING
      lcd.clear();
    }
  }

  else if(codeInputted && !connected) {
    IrSender.sendNEC(0x45, 0x11, 1); // Send PING signals until the base station sends a received signal
    delay(100);

    String waitingMsg = "Waiting";

    for (int i = 0; i <= dotIterator % 3; i++) {
      waitingMsg.concat(".");
    }

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print(waitingMsg);

    dotIterator++;
    
    // Section that moves on to the next state when a message from the computer is received (FOR TESTING PURPOSES)
    if (Serial.available() > 0) {
      connected = (Serial.readStringUntil('\n') == "CONNECTED");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connected!");

      delay(1000);
    }
  }

  else if (connected && !received) {
    IrSender.sendNEC(0x45, 0x98, 1);
    delay(250);
    
    for (int i = 0; i < 10; i++) {
      String sendingMsg = "Sending code";
      for (int i = 0; i <= dotIterator % 3; i++) {
        sendingMsg.concat(".");
      }

      dotIterator++;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(sendingMsg);

      uint32_t signal;

      switch (code[i]) {
        case '0':
          signal = 0x00;
          break;
        case '1':
          signal = 0x01;
          break;
        case '2':
          signal = 0x02;
          break;
        case '3':
          signal = 0x03;
          break;
        case '4':
          signal = 0x04;
          break;
        case '5':
          signal = 0x05;
          break;
        case '6':
          signal = 0x06;
          break;
        case '7':
          signal = 0x07;
          break;
        case '8':
          signal = 0x08;
          break;
        case '9':
          signal = 0x09;
        default:
          signal = 0x99;
      }

      IrSender.sendNEC(0x45, signal, 1);
      delay(250);
    }

    IrSender.sendNEC(0x45, 0x99, 1);

    delay(500);

    if(Serial.available() > 0) {
      received = (Serial.readStringUntil('\n') == "RECEIVED");
    }
  }

  else if (received && !bearingReceived) {
    String receivingMsg = "Receiving";
    for (int i = 0; i <= dotIterator % 3; i++) {
      receivingMsg.concat(".");
    }

    dotIterator++;
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(receivingMsg);

    // Since we don't have a bearing signal yet I'm going to just use the serial
    if (Serial.available() > 0) {
      bearing = Serial.readStringUntil('\n');
      Serial.println(bearing);
      bearingReceived = true;
    }

    delay(250);
  }

  else if (bearingReceived) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Code: ");
    lcd.print(code);
    lcd.setCursor(0, 1);
    lcd.print("Bearing: ");
    lcd.print(bearing);
    delay(1000);
  }
}

