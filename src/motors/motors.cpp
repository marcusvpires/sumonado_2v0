#include "motors.h"

#include <Arduino.h>

void Motors::setup() {
    pinMode(pwmA, OUTPUT);
    pinMode(Ain1, OUTPUT);
    pinMode(Ain2, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(Bin1, OUTPUT);
    pinMode(Bin2, OUTPUT);

    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, LOW);
    analogWrite(pwmA, 0);
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, LOW);
    analogWrite(pwmB, 0);
}

void Motors::write(int va, int vb) {
  if (va > (int) 0) {
    // Serial.println("A HIGH LOW");
    digitalWrite(Ain2, LOW);
    digitalWrite(Ain1, HIGH);
  } else if (va < (int) 0) {
    // Serial.println("A LOW HIGH");
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);
  } else {
    // Serial.println("A LOW LOW");
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, LOW);
  }
  if (vb > (int) 0) {
    // Serial.println("b HIGH LOW");
    digitalWrite(Bin2, LOW);
    digitalWrite(Bin1, HIGH);
  } else if (vb < (int) 0) {
    // Serial.println("b LOW HIGH");
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, HIGH);
  } else {
    // Serial.println("b HIGH HIGH");
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, LOW);
  }
  // Serial.println(abs(va));
  // Serial.println(abs(vb));
  analogWrite(pwmA, abs(va));
  analogWrite(pwmB, abs(vb));
}