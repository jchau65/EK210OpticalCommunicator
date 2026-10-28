#include <IRremote.hpp>         // .hpp is the current include for IRremote 4.x. why? idk
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>  // LCD screen

// ---------------- Protocol  ----------------
const uint16_t BEACON_ADDR = 0x45;
const uint16_t BASE_ADDR   = 0x46;
const uint16_t BEACON_DATA_ADDR = 0x47; 

const uint8_t CMD_PING          = 0x11;  // beacon -> base
const uint8_t CMD_CONNECTED     = 0x12;  // base -> beacon
const uint8_t CMD_RECEIVED      = 0x13;  // acknowledgment, both directions
const uint8_t CMD_BEARING_START = 0x97;  // base -> beacon, followed by a value frame and a check frame
const uint8_t CMD_START         = 0x98;  // beacon -> base, before the 10 digits
const uint8_t CMD_END           = 0x99;  // beacon -> base, after the 10 digits
// digits are sent as commands 0x00-0x09

const float BEARING_BIAS = 64.0;  // bearing byte = (bearing + 64) * 2, covers -64 to +63.5 deg at 0.5 deg resolution

// ---------------- Hardware ----------------
const int RECEIVER_PIN = 2;
const int EMITTER_PIN  = 3;  // MOSFET gate for the LED ring (Timer2 output on an Uno)

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'},
};
byte rowPins[ROWS] = {6, 7, 8, 9};
byte colPins[COLS] = {10, 11, 12, 13};  // pin 13 is why LED feedback is disabled below
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Settings ----------------
const int CODE_LENGTH = 10;
const unsigned long PING_LISTEN_MS  = 150;  // listen window after each PING (~4 PINGs per second)
const unsigned long ACK_LISTEN_MS   = 500;  // wait for RECEIVED after sending the code
const unsigned long BEARING_WAIT_MS = 1000; // wait for a bearing broadcast
const unsigned long DATA_TIMEOUT_MS = 300;  // max wait between the bearing's value and check frames
const unsigned long FRAME_GAP_MS    = 40;   // gap between consecutive frames we send
const unsigned long REPLY_DELAY_MS  = 20;   // pause before replying, so the base can clear its own echo

// ---------------- State ----------------
enum State { ENTER_CODE, PINGING, SENDING_CODE, RECEIVING_BEARING, DONE };
State state = ENTER_CODE;

char code[CODE_LENGTH + 1] = "**********";
int digits = 0;
float bearing = 0.0;
int dotCount = 0;

// ---------------- Helpers ----------------

// Sends one frame to the base station.
void sendToBase(uint8_t cmd) {
  IrSender.sendNEC(BEACON_ADDR, cmd, 0);  // 0 repeats: repeat frames decode as "last command again"
  IrReceiver.restartAfterSend();
}

// Listens for up to `ms`. Returns true (and sets cmd) when a fresh, non-repeat
// NEC frame from the BASE STATION arrives. Our own echoes (address 0x45) are
// discarded immediately so the receiver is free for the base station's reply.
bool listenForBase(unsigned long ms, uint8_t &cmd) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (IrReceiver.decode()) {
      IRData &d = IrReceiver.decodedIRData;
      bool valid = (d.protocol == NEC || d.protocol == NEC2) && d.address == BASE_ADDR; // for some reason the IrReceiver sometimes returns NEC2 when commands are received in short bursrts
                   //&& !(d.flags & IRDATA_FLAGS_IS_REPEAT); was causing issues due to short command bursts
      cmd = d.command;

      // Debug information
      Serial.print(F("rx addr 0x")); 
      Serial.print(d.address, HEX);
      Serial.print(F(" cmd 0x"));    
      Serial.println(d.command, HEX);
      Serial.print(F(" pro "));
      Serial.println(getProtocolString(d.protocol));


      IrReceiver.resume();
      if (valid) return true; // only functions when the signal is valid
    }
  }
  return false;
}

// Keeps listening for up to `ms` until the base station sends want1 or want2.
// Anything else from the base station is ignored.
bool waitForBase(unsigned long ms, uint8_t want1, uint8_t want2, uint8_t &cmd) {
  unsigned long start = millis();
  unsigned long elapsed;
  while ((elapsed = millis() - start) < ms) {
    if (listenForBase(ms - elapsed, cmd) && (cmd == want1 || cmd == want2)) return true;
  }
  return false;
}

// Call right after a BEARING_START. Reads the value and check frames and
// returns true if they agree (value + check == 255).
bool readBearingFrames() {
  uint8_t value, check;
  if (!listenForBase(DATA_TIMEOUT_MS, value)) return false;
  if (!listenForBase(DATA_TIMEOUT_MS, check)) return false;
  if ((uint8_t)(value + check) != 0xFF) {
    Serial.println(F("Bearing check failed"));
    return false;
  }
  bearing = value / 2.0 - BEARING_BIAS;
  return true;
}

void acknowledgeBearing() {
  for (int i = 0; i < 5; i++) {  // needed to send like 5 times for this to work for some reason
    delay(REPLY_DELAY_MS);
    sendToBase(CMD_RECEIVED);
  }
}

String readSerialLine() {
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();  // strips \r if the monitor sends "Both NL & CR"
    return s;
  }
  return "";
}

// ---------------- Display ----------------
void showCodeEntry() {
  lcd.clear();
  lcd.print(F("Enter code:"));
  lcd.setCursor(3, 1);
  lcd.print(code);
}

// Prints a label with 1-3 cycling dots, without clearing the screen (avoids flicker).
void showAnimated(const __FlashStringHelper *label) {
  lcd.setCursor(0, 0);
  lcd.print(label);
  for (int i = 0; i < 3; i++) lcd.print(i <= dotCount % 3 ? '.' : ' ');
  lcd.print(F("    "));
  dotCount++;
}

void printBearing(float b) {
  if (b >= 0) lcd.print('+');
  lcd.print(b, 1);
}

void showResult() {
  lcd.clear();
  lcd.print(F("ID "));
  lcd.print(code);
  lcd.setCursor(0, 1);
  lcd.print(F("Bearing "));
  printBearing(bearing);
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(9600);

  IrSender.begin(EMITTER_PIN, DISABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
  IrReceiver.begin(RECEIVER_PIN, DISABLE_LED_FEEDBACK);

  lcd.init();
  lcd.backlight();
  showCodeEntry();
}

void loop() {
  uint8_t cmd;

  switch (state) {

    case ENTER_CODE: {
      char key = keypad.getKey();

      if (key >= '0' && key <= '9' && digits < CODE_LENGTH) {  // digits only
        code[digits++] = key;
        showCodeEntry();
      } else if (key == '*' && digits > 0) {                   // * = backspace
        code[--digits] = '*';
        showCodeEntry();
      }

      if (digits == CODE_LENGTH) {
        state = PINGING;
        lcd.clear();
      }
      break;
    }

    case PINGING:
      sendToBase(CMD_PING);

      if (waitForBase(PING_LISTEN_MS, CMD_CONNECTED, CMD_CONNECTED, cmd)
          || readSerialLine() == "CONNECTED") {  // test hook
        state = SENDING_CODE;
        lcd.clear();
        break;
      }
      showAnimated(F("Waiting"));
      break;

    case SENDING_CODE:
      showAnimated(F("Sending code"));

      sendToBase(CMD_START);
      delay(FRAME_GAP_MS);
      for (int i = 0; i < CODE_LENGTH; i++) {
        sendToBase(code[i] - '0');  // '7' -> 0x07
        delay(FRAME_GAP_MS);
      }
      sendToBase(CMD_END);

      // A BEARING_START also counts: it means the base got the code but its RECEIVED was lost.
      if (waitForBase(ACK_LISTEN_MS, CMD_RECEIVED, CMD_BEARING_START, cmd)
          || readSerialLine() == "RECEIVED") {   // test hook
        state = RECEIVING_BEARING;
        lcd.clear();
      }
      // otherwise loop and resend the whole code
      break;

    case RECEIVING_BEARING: {
      showAnimated(F("Receiving"));

      if (waitForBase(BEARING_WAIT_MS, CMD_BEARING_START, CMD_BEARING_START, cmd)
          && readBearingFrames()) {
        acknowledgeBearing();
        state = DONE;
        showResult();
        Serial.print(F("Bearing: ")); Serial.println(bearing);
        break;
      }

      String s = readSerialLine();  // test hook: type a number like -12.5
      if (s.length() > 0) {
        bearing = s.toFloat();
        state = DONE;
        showResult();
      }
      break;
    }

    case DONE:
      // If our RECEIVED was lost, the base keeps resending the bearing. Acknowledge again.
      if (waitForBase(BEARING_WAIT_MS, CMD_BEARING_START, CMD_BEARING_START, cmd)) {
        readBearingFrames();
        acknowledgeBearing();
        showResult();
      }
      break;
  }
}