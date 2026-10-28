#include <LiquidCrystal_I2C.h>  // LCD screen
#include <IRremote.hpp>         // .hpp is the current include for IRremote 4.x
#include <Stepper.h>

// ---------------- Protocol  ----------------
const uint16_t BEACON_ADDR = 0x45;
const uint16_t BASE_ADDR   = 0x46;

const uint8_t CMD_PING          = 0x11;  // beacon -> base
const uint8_t CMD_CONNECTED     = 0x12;  // base -> beacon
const uint8_t CMD_RECEIVED      = 0x13;  // acknowledgment, both directions
const uint8_t CMD_BEARING_START = 0x97;  // base -> beacon, followed by a value frame and a check frame
const uint8_t CMD_START         = 0x98;  // beacon -> base, before the 10 digits
const uint8_t CMD_END           = 0x99;  // beacon -> base, after the 10 digits
// below command is for position encoding because too many packets were lost in testing for a 10 digit sequence straight
const uint8_t DIGIT_BASE        = 0x20;

const float BEARING_BIAS = 64.0; // Higher number means more precise for bearing

// ---------------- Hardware ----------------
const int STEPS_PER_REV = 2048;
Stepper myStepper(STEPS_PER_REV, 8, 10, 9, 11);
const int STEPPER_PINS[] = {8, 9, 10, 11};

const int RECEIVER_PIN = 2;
const int EMITTER_PIN  = 3;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Settings ----------------
const float OFFSET_DEG = 10;                     // calibrated: how far before the beacon the first hit occurs -- lowk need to test this at different distances
const float STEP_DEG = 2.5;                          // sweep step size
const float SWEEP_LIMIT_DEG = 45.0 + OFFSET_DEG;  // symmetric limits so edge beacons aren't already "in view" at turnaround
const unsigned long LISTEN_MS = 350;              // longer than the beacon's PING period

const int CODE_LENGTH = 10;
const unsigned long ACK_LISTEN_MS  = 500;  // wait for RECEIVED after sending the bearing
const unsigned long FRAME_GAP_MS   = 40;   // gap between consecutive frames we send
const unsigned long REPLY_DELAY_MS = 20;   // pause before replying, so the beacon can clear its own echo
const unsigned long ACK_SETTLE_MS  = 80; // extra pause before acknowledeing the code
const int MAX_BEARING_TRIES = 10;

// ---------------- State ----------------
enum State { WAIT_FOR_START, SWEEPING, CONNECTING, RECEIVING_CODE, SENDING_BEARING, DONE };
State state = WAIT_FOR_START;

long positionSteps = 0;  // 0 = power-on position (assumed centered); positive = clockwise
int direction = +1;      // +1 clockwise, -1 counterclockwise
float bearing = 0.0;

char code[CODE_LENGTH + 1] = "";
int codeCount = 0;
uint16_t digitMask = 0; 
int bearingTries = 0;
unsigned long startTime = 0;

int dotCount = 0;
unsigned long lastDotTime = 0;
const unsigned long DOT_PERIOD_MS = 300;  // how often the dots advance

// ---------------- Helpers (wrote here because the logic was way simpler if I did) ----------------
long degToSteps(float deg) { return lround(deg * STEPS_PER_REV / 360.0); }
float stepsToDeg(long steps) { return steps * 360.0 / STEPS_PER_REV; }

void releaseCoils() {
  for (int pin : STEPPER_PINS) digitalWrite(pin, LOW);
}

void moveToSteps(long target) {
  myStepper.step(target - positionSteps);
  positionSteps = target;
  //releaseCoils(); this needed to go otherwise we would have motor drift
}

// Listens for up to `ms`. Returns true (and sets cmd) when an
// NEC frame from the BEACON arrives. Our own echoes are discarded.
bool listenForBeacon(unsigned long ms, uint8_t &cmd) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (IrReceiver.decode()) {
      IRData &d = IrReceiver.decodedIRData;
      // IRremote flags ANY full frame that arrives soon after another one as a
      // "repeat" and relabels it NEC2. Quick replies and back-to-back code frames
      // always look like that, so accept NEC2 and don't check the repeat flag.
      // Neither unit sends real NEC repeats.
      bool valid = (d.protocol == NEC || d.protocol == NEC2) && d.address == BEACON_ADDR;
      cmd = d.command;

      IrReceiver.resume();

      Serial.print(F("rx addr 0x")); Serial.print(d.address, HEX);
      Serial.print(F(" cmd 0x"));    Serial.print(d.command, HEX);
      Serial.print(F(" "));          Serial.print(getProtocolString(d.protocol));
      Serial.print(F(" flags 0x"));  Serial.println(d.flags, HEX);

      IrReceiver.resume();
      if (valid) return true;
    }
  }
  return false;
}

// Keeps listening for up to `ms` until the beacon sends want1 or want2.
bool waitForBeacon(unsigned long ms, uint8_t want1, uint8_t want2, uint8_t &cmd) {
  unsigned long start = millis();
  unsigned long elapsed;
  while ((elapsed = millis() - start) < ms) {
    if (listenForBeacon(ms - elapsed, cmd) && (cmd == want1 || cmd == want2)) return true;
  }
  return false;
}

// Sends one frame to the beacon.
void sendToBeacon(uint8_t cmd) {
  IrSender.sendNEC(BASE_ADDR, cmd, 0);  // 0 repeats: repeat frames decode as "last command again"
  IrReceiver.restartAfterSend();
}

// Replies after a short pause so the beacon has time to clear its own echo.
void replyToBeacon(uint8_t cmd, int copies) {
  for (int i = 0; i < copies; i++) {
    delay(REPLY_DELAY_MS);
    sendToBeacon(cmd);
  }
}

void ackCode() {
  delay(ACK_SETTLE_MS);
  replyToBeacon(CMD_RECEIVED, 2);
}

bool isDigitFrame(uint8_t cmd) {
  return cmd >= DIGIT_BASE && cmd < DIGIT_BASE + CODE_LENGTH * 10;
}

void resetCode() {
  for (int i = 0; i < CODE_LENGTH; i++) code[i] = '-';
  code[CODE_LENGTH] = '\0';
  digitMask = 0;
  codeCount = 0;
}

void showDigits(); //predeclaring

// Stores one digit in its slot. Duplicates just overwrite; codeCount only counts new slots.
void storeDigit(uint8_t cmd) {
  uint8_t v = cmd - DIGIT_BASE;
  uint8_t pos = v / 10;
  code[pos] = '0' + (v % 10);
  if (!(digitMask & (1 << pos))) {
    digitMask |= (1 << pos);
    codeCount++;
  }
  showDigits();  // only redraw when something changed (I2C LCD writes are slow)
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
// Prints a label on line 1 with 1-3 cycling dots, without clearing the screen
// (avoids flicker). The dots advance at most every DOT_PERIOD_MS, so this can
// be called on every pass through loop().
void showAnimated(const __FlashStringHelper *label) {
  if (millis() - lastDotTime < DOT_PERIOD_MS) return;
  lastDotTime = millis();

  lcd.setCursor(0, 0);
  lcd.print(label);
  for (int i = 0; i < 3; i++) lcd.print(i <= dotCount % 3 ? '.' : ' ');
  for (int i = strlen_P((PGM_P)label) + 3; i < 16; i++) lcd.print(' ');  // clear leftovers
  dotCount++;
}

// Clears the screen for a new stage and makes the dots redraw immediately.
void newStage() {
  lcd.clear();
  lastDotTime = 0;
}

void showValue(const __FlashStringHelper *label, float value) {
  lcd.setCursor(0, 1);
  lcd.print(label);
  lcd.print(value, 1);
  lcd.print(F("     "));
}

// Shows how many code digits have arrived so far, e.g. "Digits 7/10".
void showDigits() {
  lcd.setCursor(0, 1);
  lcd.print(F("Digits "));
  lcd.print(min(codeCount, CODE_LENGTH));
  lcd.print('/');
  lcd.print(CODE_LENGTH);
  lcd.print(F("    "));
}

void printBearing(float b) {
  if (b >= 0) lcd.print('+');
  lcd.print(b, 1);
}

void showResult(bool confirmed) {
  lcd.clear();
  lcd.print(F("ID "));
  lcd.print(code);
  lcd.setCursor(0, 1);
  lcd.print(F("B "));
  printBearing(bearing);
  lcd.print(F(" T "));
  lcd.print((millis() - startTime) / 1000.0, 1);
  lcd.print('s');
  if (!confirmed) lcd.print('?');  // beacon never acknowledged the bearing
}

void finish(bool confirmed) {
  state = DONE;
  showResult(confirmed);
  Serial.print(F("Code: ")); Serial.println(code);
  Serial.print(F("Bearing: ")); Serial.println(bearing);
  Serial.print(F("Elapsed (s): ")); Serial.println((millis() - startTime) / 1000.0);
  if (!confirmed) Serial.println(F("Bearing not acknowledged by beacon"));
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  newStage();

  myStepper.setSpeed(10);
  IrReceiver.begin(RECEIVER_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(EMITTER_PIN, ENABLE_LED_FEEDBACK);

  moveToSteps(degToSteps(-SWEEP_LIMIT_DEG));  // assumes power-on position is center
}

void loop() {
  uint8_t cmd;

  switch (state) {

    case WAIT_FOR_START:
      showAnimated(F("Send START"));
      if (readSerialLine() == "START") {
        startTime = millis();  // 30 s timer starts here
        state = SWEEPING;
        newStage();
      }
      break;

    case SWEEPING: {
      showAnimated(F("Searching"));
      IrReceiver.resume();  // discard anything captured while the motor was moving
      bool hit = listenForBeacon(LISTEN_MS, cmd) && cmd == CMD_PING;

      if (readSerialLine() == "FOUND") hit = true;  // test hook

      if (hit) {
        // first hit is the leading edge of the window, so correct in the direction of travel
        bearing = stepsToDeg(positionSteps) + direction * OFFSET_DEG;
        moveToSteps(degToSteps(bearing));  // point receiver + LED at the beacon
        IrReceiver.resume();

        state = CONNECTING;
        newStage();
        showValue(F("Bearing: "), bearing);
        Serial.print(F("Bearing: ")); Serial.println(bearing);
        break;
      }

      // advance the sweep, reversing at the limits
      float next = stepsToDeg(positionSteps) + direction * STEP_DEG;
      if (next > SWEEP_LIMIT_DEG || next < -SWEEP_LIMIT_DEG) {
        direction = -direction;
        next = stepsToDeg(positionSteps) + direction * STEP_DEG;
      }
      moveToSteps(degToSteps(next));
      showValue(F("Angle: "), stepsToDeg(positionSteps));
      break;
    }

    case CONNECTING:
      showAnimated(F("Connecting"));
      // Reply right after a PING: that's exactly when the beacon is listening.
      // Keep replying until the beacon moves on and sends START.
      if (listenForBeacon(1000, cmd)) {
        if (cmd == CMD_PING) {
          replyToBeacon(CMD_CONNECTED, 1);
        } else if (cmd == CMD_START) {
          resetCode();
          state = RECEIVING_CODE;
          newStage();
          showDigits();
        }

        else if (isDigitFrame(cmd)) {
          resetCode();
          state = RECEIVING_CODE;
          newStage();
          storeDigit(cmd);
        }
      }
      break;

    case RECEIVING_CODE:
      showAnimated(F("Getting code"));
      if (listenForBeacon(1000, cmd)) {
        if (cmd == CMD_START) {
        } else if (isDigitFrame(cmd)) {
          storeDigit(cmd);                             
        } else if (cmd == CMD_END) {
          if (codeCount == CODE_LENGTH) {
            code[CODE_LENGTH] = '\0';
            Serial.print(F("Code received: ")); Serial.println(code);
            ackCode();                                
            bearingTries = 0;
            state = SENDING_BEARING;
            newStage();
            showValue(F("Bearing: "), bearing);
          } else {
            Serial.print(F("Incomplete code (")); Serial.print(codeCount);
            Serial.println(F(" digits), waiting for resend"));
          }
        } else if (cmd == CMD_PING) {
          replyToBeacon(CMD_CONNECTED, 1);             // beacon missed our CONNECTED
        }
      }
      break;

    case SENDING_BEARING: {
      showAnimated(F("Sending brg"));
      uint8_t value = constrain(lround((bearing + BEARING_BIAS) * 2), 0, 255);

      sendToBeacon(CMD_BEARING_START);
      delay(FRAME_GAP_MS);
      sendToBeacon(value);
      delay(FRAME_GAP_MS);
      sendToBeacon(255 - value);  // check frame: value + check == 255
      bearingTries++;

      if (waitForBeacon(ACK_LISTEN_MS, CMD_RECEIVED, CMD_END, cmd)) {
        if (cmd == CMD_RECEIVED) {
          finish(true);
          break;
        }
        // END means the beacon is resending the code: our RECEIVED was lost
        ackCode();
      }

      if (bearingTries >= MAX_BEARING_TRIES) finish(false);
      break;
    }

    case DONE:
      // If the beacon is still resending its code, acknowledge it again.
      if (waitForBeacon(1000, CMD_END, CMD_END, cmd)) {
        ackCode();
      }

      releaseCoils(); // so I can adjust for the next test
      break;
  }
}