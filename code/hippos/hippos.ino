/*
 * hippos.ino v1.0.03
 *
 * Edit History
 *

 * v1.0.04 - moving to Arduino Nano 33 IoT for the power pin.
 * v1.0.03 - moving to ESP32-S3 (different servo library. Leverage builtin TFT)
 * v1.0.02 - Added servo tuning feature, tied to serial port.
 * v1.0.01 - initial creation
 *
 * Hungry Hungry Hippos – Adaptive Game Controller
 * Developed by Fox Devices, LLC
 *
 * There are 4 adaptive switches controlling 4 servos that are mounted to games pieces. The servos
 * normally remain in the "closed" state until a button is pressed. When pressed, the servo arm moves
 * to the "open" state. It remains open until either the button is released or for a preconfigured
 * time value like half a second.
 *
 * The code supports two modes for servo operation. In mode 1, the hippo head is extended for a fixed duration of time
 * and then retracts automatically. (This time duration can be set below in the line SERVO_OPEN_TIME = 500;) In mode 2, 
 * the hippo head remains extended until the user releases the button. Any user can toggle between modes by double-tapping
 * their switch within .5 seconds. The code will assure that only one person changes modes at a time. One of the LEDS
 * will indicate the current mode (green or blue).
 *
 * The switches are active low which means when not being pressed, the code will read a 1 on their
 * input pins but while pressed, the code will see a 0.
 *
 * The code lights a different colored LED whenever a hippo's button is pressed.  (orange for the organge hippo,
 * green for the green hippo, etc). In order for the code to know which color to use it assumes a specific
 * order of the adaptive switches controlling specific hippo servos. For this reason, the DC switch jacks and the
 * servo cables are labelled. The game will operate fine even if these are mixed up. It will just throw off
 * the color associations seen in the LED strip.
 *
 * Learn all about how to use neopixels in the Adafruit Neopixel Uberguide - https://learn.adafruit.com/adafruit-neopixel-uberguide/the-magic-of-neopixels
 *
 * The code also checks for commands on the serial port. This allows the user to fine tune servo 
 * response by trying new values for SERVO_OPEN_POS and SERVO_CLOSED_POS while the game is running.
 * For example, one can change the servo closed position to 65 degrees by typing "C65" into 
 * the input window in the Adruino IDE serial monitor. The command format is Oxx (for open) and Cxx for 
 * close. New values are not remembered and must be transferred manually to the appropriate lines below
 *
 * This technique is useful in early development when you're still fine-tuning motion ranges.
 * You can adjust values live via the serial monitor without needing to re-upload your sketch.
 * For a more permanent solution, update the values of SERVO_OPEN_POS and SERVO_CLOSED_POS in the code.
 * The code also debounces the switches. There is logic to avoid accidental double activiations of a switch
 * or sensing multiple switch activations when the user only pressed the button once.
 * 
 * This code is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0).
 * 
 * You are free to use, modify, and share this code for personal, non-commercial use, provided that you credit Fox Devices, LLC as the original author.
 * 
 * Commercial use is not permitted without prior written consent.
 * 
 * For full license details, visit:
 * https://creativecommons.org/licenses/by-nc/4.0/
 */
 
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

const int BUILTIN_LED = 13;  // Built-in LED
const unsigned long BLINK_INTERVAL = 2000;  // Total cycle time (2 seconds)
const unsigned long ON_DURATION = 100;     // LED on for 1 second
unsigned long previousBlink = 0;
bool ledOn = false;
const int NUM_HIPPOS = 4;

/******************************/
/*  Neopixel LED declarations */
/******************************/

#define NEOPIXEL_PIN A0      // Choose just about any free  pin
#define NUM_PIXELS   8        // Full strip length, even if you're only using 4
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);  //neopixel object that maintains state
uint32_t hippoColors[4] = {
  pixels.Color(125, 50, 0),   // for when ORANGE_HIPPO switch is activated
  pixels.Color(0, 125, 0),     // for when GREEN_HIPPO  switch is activated
  pixels.Color(125, 125, 0),   // for when YELLOW_HIPPO switch is activated
  pixels.Color(0, 0, 125)     // for when BLUE_HIPPO   switch is activated
};
#define MODE_1_COLOR   pixels.Color(0, 0, 30)
#define MODE_2_COLOR   pixels.Color(0, 30, 0)
#define STATUS_PIXEL   7        // use one pixel for the mode status

#define OFF pixels.Color(0, 0, 0)

/*****************************************/
/*  Adaptive switch related declarations */
/*****************************************/

const int adaptiveSwitchPins[NUM_HIPPOS] = {A4, A5, A6, A7}; // Adaptive switch input pins (see breadboard)

const unsigned long DEBOUNCE_TIME = 50;     // in millisseconds. Shouldn't need to change. Makes sure button press is only seen once.
const unsigned long DOUBLE_TAP_WINDOW = 500; // in millisseconds, to detect and avoid accidental double-activation
bool buttonState[NUM_HIPPOS] = {false}; // Debounce tracking for each switch
bool lastButtonState[NUM_HIPPOS] = {false}; // Debounce tracking for each switch
unsigned long lastDebounceTime[NUM_HIPPOS] = {0}; // Debounce tracking for each switch
// Double-tap mode toggle tracking
unsigned long lastTapTime[NUM_HIPPOS] = {0}; // for each switch
int tapCount[NUM_HIPPOS] = {0};

/*******************************/
/*  Servo related declarations */
/*******************************/

const int servoPins[NUM_HIPPOS] = {6, 9, 10, 11};    // Servo output pins for Arduino Nano 33 IoT (see breadboard)
Servo hippoServos[NUM_HIPPOS]; // Servo objects, each one controls a hippo via a physical pin listed in servoPins[]

// ⚙️ NOTE: This implementation uses a single open/close value (below) for all 4 servos.
// If you'd like to tune each servo individually (for example, to account for mechanical differences),
// you can change SERVO_OPEN_POS and SERVO_CLOSED_POS from single int variables
// to arrays: int servoOpenPos[NUM_HIPPOS]; and int servoClosedPos[NUM_HIPPOS];
// Then update the code wherever SERVO_OPEN_POS or SERVO_CLOSED_POS is used
// to access the value specific to that servo (e.g., servoOpenPos[i]).
// This approach fits well with the current structure, which already uses index-based arrays.

int SERVO_OPEN_POS  = 0;   // Servo position when hippo head extends out. Change this as necessary.
int SERVO_CLOSED_POS = 75; // Servo position when hippo head returns. Change this as necessary.

enum ModeType { MODE_HOLD_TO_OPEN = 2, MODE_FIXED_DURATION = 1 };
ModeType mode = MODE_FIXED_DURATION; //game can operate in 2 different modes. Holding the servo open or timed.
//ModeType mode = MODE_HOLD_TO_OPEN; //game can operate in 2 different modes. Holding the servo open or timed.
const unsigned long SERVO_OPEN_TIME = 500;  // in millisseconds (adjustable for Mode 1 - Fixed Duration)
bool servoIsOpen[NUM_HIPPOS] = {false}; // Need to track servo timing if we're using  mode MODE_FIXED_DURATION
unsigned long servoCloseTime[NUM_HIPPOS] = {0}; // Need to track servo timing if we're using  mode MODE_FIXED_DURATION

/* Forward Declarations for all functions */
void setup();
void loop();
void processSerial();
void processServos();
void setOpenPosition(int pos);
void setClosePosition(int pos);
void setLEDs(int LEDnum, boolean new_state);
void toggleMode();
void waitForSerial(unsigned long timeout_ms = 3000);

void setup() {
  Serial.begin(9600);                     // start the serial port object so we can read and write to it
  waitForSerial(3000);  // Wait up to 3000ms (3 seconds) for Serial connection, then move on
  Serial.println("🐗 Hungry Hippos Controller Ready");
  Serial.println("Default Mode: MODE 1 (Fixed Duration)");
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);  // Start with LED off
  pixels.begin();       // Initialize NeoPixel LED strip
  pixels.clear();       // Make sure all pixels are off
  pixels.show();        // Push that update to the strip
  delay(200);
  for (int i = 0; i < NUM_HIPPOS; i++) { // loop thru all the servos and...
    pinMode(adaptiveSwitchPins[i], INPUT_PULLUP); // declare their I/O pin. PULLUP causes the pin to have a 1 value except while the button is pressed it's a 0.
    hippoServos[i].attach(servoPins[i]);       // associate that I/O pin with their server "object"
    hippoServos[i].write(SERVO_CLOSED_POS);    // set the servo arm to the closed position
  }
  pixels.setPixelColor(STATUS_PIXEL, MODE_1_COLOR);  // Change mode color on LED 7 so user knows what mode (green or blue)
  pixels.show();        // Push that update to the strip
}

void loop() {
unsigned long currentTime = millis();
bool anyServoOpen = false;

// need to know if any buttons are being pressed, so the blining heartbeat doesn't exteinguish the builtin LED.
for (int i = 0; i < NUM_HIPPOS; i++) {
  if (servoIsOpen[i]) {
    anyServoOpen = true;
    break;  // Exit early if any servo is open
  }
}

  if (!ledOn && (currentTime - previousBlink >= BLINK_INTERVAL)) {
    digitalWrite(BUILTIN_LED, HIGH);
    previousBlink = currentTime;
    ledOn = true;
  }

// only extinguish the builtin LED on the heartbeat timer if also no servo is being held open.
if (anyServoOpen) {
  digitalWrite(BUILTIN_LED, HIGH);  // Keep LED on if any servo is open
  } else {
    if (ledOn && (currentTime - previousBlink >= ON_DURATION)) {
      digitalWrite(BUILTIN_LED, LOW);
      Serial.println("in new code setting BUILTIN_LEF to OFF");
      ledOn = false;
    }
  }
  
  if (Serial.available() >= 3) {  // Wait for full 3-character command (like "C75") on serial port
      processSerial();
    }
  processServos();
}

/*********************/
/*  Helper Functions */
/*********************/

void processSerial() {
  char command = Serial.read();    // 'O' or 'C'
  char digit1 = Serial.read();     // First digit for servo degrees
  char digit2 = Serial.read();     // Second digit for servo degrees

  // NOTE: These changes are temporary and will not persist if the board is restarted.
  // To make permanent, update the SERVO_OPEN_POS and SERVO_CLOSED_POS constants in code.

  if ((command == 'c' || command == 'C'|| command == 'o'|| command == 'O') && // Check if all are valid ASCII digits
      isDigit(digit1) && isDigit(digit2)) {
    int value = (digit1 - '0') * 10 + (digit2 - '0');
    if (command == 'O' || command == 'o') {
      setOpenPosition(value);
    } else {
      setClosePosition(value);
    }
  } else {
    Serial.println("Invalid input. Format: Oxx or Cxx. Saw comand " + String(command) + String(digit1) + String(digit2));
  }
  while (Serial.available()) Serial.read(); // Clear any leftover bytes to keep things clean
}

void processServos(){
unsigned long now = millis();
  /*
   * Loop through each hippo/switch pair, using the loop index `i` to access elements in our arrays.
   * Note: `i` is just an index — it does NOT refer to an actual pin number.
   *
   * For example, when i == 3:
   *   adaptiveSwitchPins[3] == 4      → this means we’re reading from input pin 4
   *   servoPins[3]          == 10     → this means we’re writing to servo on output pin 10
   *   hippoServos[3]                  → this is the Servo object that was attached to pin 10
   *                                     in setup() via hippoServos[i].attach(servoPins[i]); 
   *
   * So when we see a LOW signal (button press) on pin 4, we move the servo connected to pin 10.
   * This pattern repeats for all 4 hippos, with each `i` controlling its corresponding pair.
   */

  for (int i = 0; i < NUM_HIPPOS; i++) { 
    bool reading = digitalRead(adaptiveSwitchPins[i]) == LOW; // Active LOW = button pressed
   
    if (reading != lastButtonState[i]) {  // Debounce logic for the buttons
      lastDebounceTime[i] = now;
      lastButtonState[i] = reading;
    }

    if ((now - lastDebounceTime[i]) > DEBOUNCE_TIME) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading;  // Update debounced state
        if (buttonState[i]) { // ---------- Edge event (button just pressed) ----------
          if ((now - lastTapTime[i]) < DOUBLE_TAP_WINDOW) { // Double-tap logic
            tapCount[i]++;
          } else {
            tapCount[i] = 1;
          }
          lastTapTime[i] = now;
          if (tapCount[i] >= 2) {
            toggleMode(); // user did a double-tap to toggle modes. Le's do it!
            tapCount[i] = 0;
            for (int j = 0; j < NUM_HIPPOS; j++) { // Also reset other players' tap counts to prevent race toggles
              if (j != i) tapCount[j] = 0;
            }
          }

          if (mode == MODE_FIXED_DURATION) {   // Mode 1: Timed open, so open servo and start timing it
            Serial.println(String(i) + " is being pressed in MODE_FIXED_DURATION, open = " + String(SERVO_OPEN_POS));
            setLEDs(i, true);
            hippoServos[i].write(SERVO_OPEN_POS);
            servoIsOpen[i] = true;
            servoCloseTime[i] = now + SERVO_OPEN_TIME;
          }

          if (mode == MODE_HOLD_TO_OPEN) { // Mode 2: Just pressed → open, open servo but don't bother timing it.
            Serial.println(String(i) + " PRESSED → OPEN in MODE_HOLD_TO_OPEN, open = " + String(SERVO_OPEN_POS));
            setLEDs(i, true);
            hippoServos[i].write(SERVO_OPEN_POS);
            servoIsOpen[i] = true;
          }
        }

        else { // We're in mode 2 and saw an edge event (button just released) so close it! ----------
          if (mode == MODE_HOLD_TO_OPEN) {
            Serial.println(String(i) + " RELEASED → CLOSE in MODE_HOLD_TO_OPEN, closed = " + String(SERVO_CLOSED_POS));
            hippoServos[i].write(SERVO_CLOSED_POS);
            servoIsOpen[i] = false;
            setLEDs(i, false);
          }
        }
      }
    }

    // We're in Mode 1 and the servo duration timed out. Close it! ----------
    if (mode == MODE_FIXED_DURATION && servoIsOpen[i] && now >= servoCloseTime[i]) {
      Serial.println(String(i) + " is closing due to MODE_FIXED_DURATION, closed = " + String(SERVO_CLOSED_POS));
      hippoServos[i].write(SERVO_CLOSED_POS);
      setLEDs(i, false);
      servoIsOpen[i] = false;
    }
  }  
}

void setOpenPosition(int pos) { // User is temporarily redefining the open position of the servo
  if (pos < 0 || pos > 180) {
    Serial.println("Servo position must be between 0 and 180.");
    return;
  }
  SERVO_OPEN_POS = pos;
  Serial.println("Setting OPEN position to " + String(SERVO_OPEN_POS));
}

void setClosePosition(int pos) { // User is temporarily redefining the closed position of the servo
  if (pos < 0 || pos > 180) {
    Serial.println("Servo position must be between 0 and 180.");
    return;
  }
  SERVO_CLOSED_POS = pos;
  Serial.println("Setting CLOSED position to " + String(SERVO_CLOSED_POS));
}

void setLEDs(int LEDnum, boolean new_state){
  if (LEDnum < 0 || LEDnum > NUM_PIXELS - 1) {
    Serial.println("Neopixel number must be between 0 and " + String(NUM_PIXELS - 1) + ". " + String(LEDnum) + " was seen.");
    return;
  }
  Serial.println("In setLEDs():  LED[" + String(LEDnum) + "] to color[" + String(new_state) + "]");
 
  if (new_state){
    Serial.println("setting LED[" + String(LEDnum) + "] to color");
    pixels.setPixelColor(LEDnum, hippoColors[LEDnum]);  // Turn on LED
    digitalWrite(BUILTIN_LED, HIGH);
  }
  else{
    Serial.println("setting LED[" + String(LEDnum) + "] to OFF");
    pixels.setPixelColor(LEDnum, OFF);  // Turn on LED OFF
    digitalWrite(BUILTIN_LED, LOW);
  }
  pixels.show();     // Push to LED strip
  delay(10);
}

void toggleMode() { // toggle servos between  staying open a preset lenght of time or waiting for button release
  if (mode == MODE_FIXED_DURATION) {
    mode = MODE_HOLD_TO_OPEN;
    pixels.setPixelColor(STATUS_PIXEL, MODE_2_COLOR);  // Change mode color on LED 0 so user knows what mode
    Serial.println("🔄 Switched to MODE 2: Hold-to-Open");
  } else {
    mode = MODE_FIXED_DURATION;
    pixels.setPixelColor(STATUS_PIXEL, MODE_1_COLOR);  // Change mode color on LED 0 so user knows what mode
    Serial.println("🔄 Switched to MODE 1: Timed Press");
  }
  pixels.show();    // Push new LED color to LED strip
}

void waitForSerial(unsigned long timeout_ms) {
  unsigned long start = millis();
  while (!Serial && (millis() - start < timeout_ms)) {
    delay(10);
  }
}

