#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include <IRremote.h>

#include "motors/motors.h"

VL53L0X sensor;
Motors motors;

#define ledTeste 4
#define IR_RECEIVE_PIN 36

bool running = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();
  motors.setup();

  pinMode(ledTeste, OUTPUT);
  digitalWrite(ledTeste, LOW);

  IrReceiver.begin(IR_RECEIVE_PIN);
}

void loop(){
    if (running) {
        Serial.println(sensor.readRangeContinuousMillimeters());
        if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        if (sensor.readRangeContinuousMillimeters() < 250) {
            Serial.println("ATACAR");
            motors.write(255, 255);
        } else {
            Serial.println("GIRAR");
            motors.write(-255, 255);
        }
    } else {
        motors.write(0, 0);
    }

    if (IrReceiver.decode()){
        IrReceiver.resume();
        int command = IrReceiver.decodedIRData.command;
        Serial.print(command);
        digitalWrite(ledTeste, HIGH);
        delay(20);
        digitalWrite(ledTeste, LOW);
        if (command == 1) running = true;
        else if (command == 2) running = false;     
    }
}