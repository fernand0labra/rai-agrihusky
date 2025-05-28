#include <SDI12.h>

// ——— Pin Definitions ———
#define STEP_PIN           10    // Stepper step pin
#define DIR_PIN            9     // Stepper direction pin
#define EN_PIN             8     // Stepper enable pin (ST330: HIGH=on, LOW=off)

#define TOP_LIMIT_PIN      3     // Main/retract limit switch (top position)
#define SLEEVE_LIMIT_PIN   2     // Probe fully-inserted limit switch
#define POT_PIN            A0    // Linear potentiometer analog pin
#define SDI12_DATA_PIN     50    // SDI-12 data line

// ——— Interface Logic ———
#define PROBE_START         0x00    // Signal from controller to start the probing
#define PROBE_STALL         0xDF    // ERROR: Insertion stalled. Retracting...
#define PROBE_NO_CONTACT    0xEF    // ERROR: No initial pot contact. Retracting...
#define PROBE_DATA_FAIL     0x0F    // ERROR: Unsuccesful Reading (NULL)
#define PROBE_DATA_SUCCESS  0xFF    // Successful Reading

// ——— Control Parameters ———
const int pulseWidthMicros      = 1000;   // µs for each half-pulse
const int maxStepCount          = 5000;   // Absolute safety limit
const int contactThreshold      = 50;     // ΔA0 to detect initial contact
const int potDeltaThreshold     = 2;      // ΔA0 to consider “moving”
const unsigned long settleTime  = 1000;   // ms to wait for “stalled” detection
const unsigned long readDelay   = 10000;  // ms sensor stabilization

SDI12 mySDI12(SDI12_DATA_PIN);

// ——— Direction-change delay support ———
bool lastStepDir;       // last direction passed to stepOnce()
bool firstStep = true;  // true until the very first call to stepOnce()

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(TOP_LIMIT_PIN, INPUT_PULLUP);
  pinMode(SLEEVE_LIMIT_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  // Start with driver disabled (motor free)
  digitalWrite(EN_PIN, LOW);
  mySDI12.begin();

  Serial.println("Ready. Send 0 to home then sample.");
}

void loop() {
  if (Serial.available() && Serial.read() == PROBE_START) {
    Serial.println("Homing to top limit...");
    homeActuator();

    Serial.println("Homing complete. Starting probe sequence...");
    runProbeSequence();

    Serial.println("Sequence complete. Back to default.");
  }
}

// ——— Home: drive UP until TOP_LIMIT_PIN is pressed ———
void homeActuator() {
  while (digitalRead(TOP_LIMIT_PIN) == HIGH) {
    // up direction = false
    stepOnce(false);
  }
  // free the motor
  digitalWrite(EN_PIN, LOW);
}

// ——— Full probe sequence ———
void runProbeSequence() {
  if (!moveUntilPotChange()) {
    retractToTop();
    Serial.println("ERROR: No initial pot contact. Retracting...");
    Serial.write(PROBE_NO_CONTACT);
    return;
  }
  if (!insertMode()) {
    retractToTop();
    Serial.println("ERROR: Insertion stalled. Retracting...");
    Serial.write(PROBE_STALL);
    return;
  }
  char* data = readSDIData();
  retractToTop();
  if (data == NULL) {
    Serial.write(PROBE_DATA_FAIL);
  } else {
    Serial.write(PROBE_DATA_SUCCESS);
    Serial.write(data);
  }
  
}

// ——— Move down until pot shifts ———
bool moveUntilPotChange() {
  int baseline = analogRead(POT_PIN);
  unsigned long start = millis();
  int steps = 0;

  Serial.print("Baseline pot: ");
  Serial.println(baseline);

  while (steps < maxStepCount) {
    stepOnce(true);  // down
    steps++;
    int curr = analogRead(POT_PIN);
    if (abs(curr - baseline) > contactThreshold) {
      Serial.print("Pot changed: ");
      Serial.println(curr);
      return true;
    }
    if (millis() - start > 10000UL) {
      return false;  // timed out
    }
  }
  return false;
}

// ——— Continue inserting until limit or stall ———
bool insertMode() {
  unsigned long lastChange = millis();
  int lastVal = analogRead(POT_PIN);
  int steps = 0;

  Serial.println("Entering insert mode...");
  while (steps < maxStepCount) {
    if (digitalRead(SLEEVE_LIMIT_PIN) == LOW) {
      Serial.println("Sleeve limit reached!");
      return true;
    }
    stepOnce(true);
    steps++;
    int curr = analogRead(POT_PIN);
    if (abs(curr - lastVal) > potDeltaThreshold) {
      lastVal    = curr;
      lastChange = millis();
    } else if (millis() - lastChange > settleTime) {
      return false;  // stalled
    }
  }
  return false;
}

// ——— SDI-12 sampling ———
char* readSDIData() {
  Serial.println("Reading SDI sensor...");
  delay(readDelay);
  mySDI12.sendCommand("0R0!");
  delay(500);
  if (mySDI12.available()) {
    const char *data= mySDI12.readString().c_str();
    Serial.print("SDI Response: ");
    Serial.println(data);
    return data;
  } else {
    Serial.println("No response from SDI-12 sensor");
    return NULL;
  }
}

// ——— Retract to top limit ———
void retractToTop() {
  Serial.println("Retracting to top...");
  while (digitalRead(TOP_LIMIT_PIN) == HIGH) {
    stepOnce(false);  // up
  }
  digitalWrite(EN_PIN, LOW);
  Serial.println("Reached top. Motor free.");
}

// ——— Step once, with 1s pause on direction changes ———
void stepOnce(bool down) {
  // If first call ever, or direction changed, wait 1s
  if (firstStep || down != lastStepDir) {
    delay(1000);
    firstStep      = false;
    lastStepDir    = down;
  }
  // Set direction & enable driver
  digitalWrite(DIR_PIN,    down ? HIGH : LOW);
  digitalWrite(EN_PIN,     HIGH);

  // Pulse
  digitalWrite(STEP_PIN,   HIGH);
  delayMicroseconds(pulseWidthMicros);
  digitalWrite(STEP_PIN,   LOW);
  delayMicroseconds(pulseWidthMicros);
}
