#include "HX711.h"
#include <Adafruit_NeoPixel.h>

#define NEO_PIN            6
#define NUM_LEDS          12
Adafruit_NeoPixel ring(NUM_LEDS, NEO_PIN, NEO_GRB + NEO_KHZ800);
const uint8_t ledOrder[12] = {1,2,3,4,5,6,7,8,9,10,11,0};

const int dirA             = 12;
const int pwmA             = 3;
const int brakeA           = 9;
const int vryPin           = A3;
const int BUTTON_PIN       = 4;
const int BUZZER_PIN       = 8;
const int LOADCELL_DT_PIN  = 5;
const int LOADCELL_SCK_PIN = 2;

#define ENCODER_CLK   10
#define ENCODER_DT    11
#define ENCODER_SW     7

const float ACTUATOR_SPEED_INCHES_PER_SEC = 1.33858;
const int ACTUATOR_PWM = 250;
float actuatorPositionInches = 0.0;

HX711 scale;
float calibration_factor = -7050.0;
float empty_weight = 0.0, full_weight = 0.0;
const unsigned long HOLD_TIME = 500;

int neutralY = 512;
const int upThreshold   = neutralY + 150;
const int downThreshold = neutralY - 150;

bool button_held = false, action_triggered = false, weight_set = false;
unsigned long button_press_time = 0;

float cached_percent = 100.0;
unsigned long last_scale_read = 0;
bool actuator_moving = false;

int currentLedCount = 0;           // positive = green/up, negative = red/down
int committedLedCount = 0;
bool encoderModeActive = false;
unsigned long lastRotationTime = 0;
const unsigned long REVERT_TIMEOUT = 3000;

int8_t lastA = -1, lastB = -1;

// Your original alarm tune
const int alarm_melody[] = {
  523,659,784,1047,784,659,523,440,880,1047,1319,1760,1319,1047,880,784,
  587,698,880,1175,880,698,587,523,1047,1319,1568,2093,1568,1319,1047,784
};
const int alarm_durations[] = {
  120,120,120,240,120,120,120,240,120,120,120,240,120,120,120,240,
  120,120,120,240,120,120,120,240,120,120,120,240,120,120,120,120
};
const int TUNE_LEN = 32;

bool alarm_active = false;
int note_index = 0;
unsigned long last_note_time = 0;

void ledsOff() { ring.clear(); ring.show(); }

void setRingPreview(int count) {
  ring.clear();
  int a = abs(count);
  uint32_t col = (count > 0) ? ring.Color(255,0,0) : ring.Color(0,255,0);  // Green = up, Red = down
  for (int i = 0; i < a; i++) ring.setPixelColor(ledOrder[i], col);
  ring.show();
}

// PERFECT 38.64 SECOND FULL STROKE – SCALES LINEARLY
void moveActuatorInches(float inches) {
  if (abs(inches) < 0.01) return;

  bool extend = (inches > 0);
  int absLedCount = abs(currentLedCount);
  if (absLedCount == 0) return;

  const unsigned long FULL_STROKE_MS = 38640UL;           // 38.64 seconds
  unsigned long duration = (FULL_STROKE_MS * absLedCount) / 12;  // perfect scaling

  digitalWrite(brakeA, LOW);
  digitalWrite(dirA, extend ? HIGH : LOW);
  analogWrite(pwmA, ACTUATOR_PWM);
  ledsOff();
  actuator_moving = true;

  unsigned long start = millis();
  while (millis() - start < duration) {
    int y = analogRead(vryPin);
    if (y > upThreshold || y < downThreshold) {
      analogWrite(pwmA, 0);
      digitalWrite(brakeA, HIGH);
      actuator_moving = false;
      return;
    }
    delay(1);
  }

  analogWrite(pwmA, 0);
  digitalWrite(brakeA, HIGH);
  actuator_moving = false;

  actuatorPositionInches += inches;
  committedLedCount = currentLedCount;
  currentLedCount = 0;
  setRingPreview(0);
  encoderModeActive = false;
}

void startupWithBeeps() {
  ring.clear();
  for (int i = 0; i < 12; i++) {
    ring.setPixelColor(ledOrder[i], ring.Color(0, 0, 150));
    ring.show();
    tone(BUZZER_PIN, 900 + i*100, 40);
    delay(70);
  }
  noTone(BUZZER_PIN);
  ledsOff();
}

void smoothGreenFillWithBeeps() {
  ring.clear(); ring.show();
  tone(BUZZER_PIN, 1500, 100); delay(120);
  tone(BUZZER_PIN, 2000, 150); delay(180);
  for (int i = 0; i < 12; i++) {
    ring.setPixelColor(ledOrder[i], ring.Color(0, 255, 0));
    ring.show();
    delay(50);
  }
  noTone(BUZZER_PIN);
  delay(150);
}

void updateWeightDisplay() {
  if (millis() - last_scale_read >= 300) {
    float current = scale.get_units(5);
    if (current < empty_weight) current = empty_weight;
    cached_percent = ((current - empty_weight) / (full_weight - empty_weight)) * 100.0;
    cached_percent = constrain(cached_percent, 0, 100);
    last_scale_read = millis();
  }
  int ledsOn = (int)(cached_percent * 12 / 100.0 + 0.5);
  ring.clear();
  uint32_t color = (cached_percent > 66.6) ? ring.Color(0,255,0) :
                   (cached_percent > 33.3) ? ring.Color(255,150,0) : ring.Color(255,0,0);
  for (int i = 0; i < ledsOn; i++) ring.setPixelColor(ledOrder[i], color);
  ring.show();
}

void setup() {
  Serial.begin(9600);
  ring.begin();
  ring.setBrightness(80);
  ring.show();

  scale.begin(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(15);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  digitalWrite(brakeA, HIGH);

  empty_weight = scale.get_units(3);
  if (empty_weight < 0) empty_weight = 0;

  startupWithBeeps();
}

void loop() {
  int y = analogRead(vryPin);

  // Joystick control
  if (y > upThreshold || y < downThreshold) {
    digitalWrite(brakeA, LOW);
    digitalWrite(dirA, (y > upThreshold) ? HIGH : LOW);
    analogWrite(pwmA, ACTUATOR_PWM);
    actuator_moving = true;
    ledsOff();
    noTone(BUZZER_PIN);
    alarm_active = false;
    encoderModeActive = false;
    currentLedCount = 0;
    committedLedCount = 0;
  } else {
    analogWrite(pwmA, 0);
    digitalWrite(brakeA, HIGH);
    if (actuator_moving) actuator_moving = false;
  }

  // Rotary encoder – RIGHT = UP (GREEN), LEFT = DOWN (RED)
  if (!actuator_moving) {
    int8_t a = digitalRead(ENCODER_CLK);
    int8_t b = digitalRead(ENCODER_DT);

    if (a != lastA || b != lastB) {
      if (lastA == HIGH && a == LOW) {
        if (b == LOW)  currentLedCount++;      // RIGHT → GREEN → UP
        else           currentLedCount--;      // LEFT → RED → DOWN
      }
      currentLedCount = constrain(currentLedCount, -12, 12);

      encoderModeActive = true;
      lastRotationTime = millis();
      setRingPreview(currentLedCount);
    }
    lastA = a;
    lastB = b;
  }

  // 3-second timeout → revert
  if (encoderModeActive && millis() - lastRotationTime > REVERT_TIMEOUT) {
    currentLedCount = committedLedCount;
    setRingPreview(currentLedCount);
    encoderModeActive = false;
  }

  // Encoder button press → move actuator
  static bool lastSwState = HIGH;
  bool swState = digitalRead(ENCODER_SW);
  if (lastSwState == HIGH && swState == LOW) {
    if (currentLedCount != 0) {
      float inches = currentLedCount * 2.0;
      moveActuatorInches(inches);
    }
  }
  lastSwState = swState;

  // Set 100% button
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!button_held) {
      button_press_time = millis();
      button_held = true;
      action_triggered = false;
    } else if (!action_triggered && millis() - button_press_time >= HOLD_TIME) {
      float current = scale.get_units(10);
      if (current < empty_weight + 5.0) {
        Serial.println("Too light!");
        tone(BUZZER_PIN, 300, 400);
      } else {
        full_weight = current;
        weight_set = true;
        Serial.print("100% SET = "); Serial.println(full_weight, 2);
        smoothGreenFillWithBeeps();
      }
      action_triggered = true;
    }
  } else {
    button_held = false;
    action_triggered = false;
  }

  // Weight display & low-weight alarm
  if (!actuator_moving && weight_set && !encoderModeActive) {
    updateWeightDisplay();

    if (cached_percent <= 20.0) {
      if (!alarm_active) {
        alarm_active = true;
        note_index = 0;
        last_note_time = millis();
      }
      if (millis() - last_note_time >= alarm_durations[note_index]) {
        tone(BUZZER_PIN, alarm_melody[note_index]);
        note_index = (note_index + 1) % TUNE_LEN;
        last_note_time = millis();
      }
      if ((millis() / 125) % 2 == 0) {
        for(int i=0; i<12; i++) ring.setPixelColor(ledOrder[i], ring.Color(255,0,0));
      } else {
        ring.clear();
      }
      ring.show();
    } else if (alarm_active) {
      noTone(BUZZER_PIN);
      alarm_active = false;
      note_index = 0;
    }
  }

  // Serial monitor
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Weight: "); Serial.print(scale.get_units(5),1);
    Serial.print("g → "); Serial.print(cached_percent,1); Serial.print("%");
    Serial.print(" | Pos: "); Serial.print(actuatorPositionInches, 2); Serial.println("\"");
    lastPrint = millis();
  }
}