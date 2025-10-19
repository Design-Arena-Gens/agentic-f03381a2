#include <SoftwareSerial.h>

// Pin configuration (ATmega328P / Arduino Uno)
const uint8_t PIN_LED = 13;

// HC-05 SoftwareSerial wiring
const uint8_t PIN_BT_RX = 10; // HC-05 TX -> Arduino 10 (SoftwareSerial RX)
const uint8_t PIN_BT_TX = 11; // HC-05 RX <- Arduino 11 (SoftwareSerial TX)

// L293D - Steering (M1)
const uint8_t PIN_IN1 = 2;  // IN1
const uint8_t PIN_IN2 = 3;  // IN2
const uint8_t PIN_EN1 = 5;  // EN1 (PWM)

// L293D - Propulsion (M3)
const uint8_t PIN_IN3 = 4;  // IN3
const uint8_t PIN_IN4 = 6;  // IN4
const uint8_t PIN_EN3 = 7;  // EN3 (PWM)

SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX); // RX, TX

// Forward declarations
void applyStopNoAck();
void handleCommand(char c);
void propulsionForward();
void propulsionBackward();
void steeringLeft();
void steeringRight();
void allStop();
void blinkAck();

void setup() {
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_EN1, OUTPUT);

  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_EN3, OUTPUT);

  // Ensure safe state
  allStop();

  // Self-diagnostic: 3 slow pulses
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(300);
    digitalWrite(PIN_LED, LOW);
    delay(300);
  }

  btSerial.begin(9600);
}

void loop() {
  if (btSerial.available() > 0) {
    char c = (char)btSerial.read();
    handleCommand(c);
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'F':
      propulsionForward();
      blinkAck();
      break;
    case 'B':
      propulsionBackward();
      blinkAck();
      break;
    case 'L':
      steeringLeft();
      blinkAck();
      break;
    case 'R':
      steeringRight();
      blinkAck();
      break;
    case 'S':
      allStop();
      blinkAck();
      break;
    default:
      // Treat unknown as Stop but do not blink (silent fail-safe)
      applyStopNoAck();
      break;
  }
}

void propulsionForward() {
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_EN3, 255);
}

void propulsionBackward() {
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_EN3, 255);
}

void steeringLeft() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_EN1, 255);
}

void steeringRight() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_EN1, 255);
}

void allStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_EN1, 0);
  analogWrite(PIN_EN3, 0);
}

void applyStopNoAck() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_EN1, 0);
  analogWrite(PIN_EN3, 0);
}

void blinkAck() {
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  delay(50);
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}
