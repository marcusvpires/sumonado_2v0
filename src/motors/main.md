#include <Arduino.h>
#include <analogWrite.h>
#include <IRremote.h>
#include <VL53L0X.h>    //sensores de distância
#include <Wire.h>       //Auxiliar dos sensores

#include "motors/motors.h"

Motors motors;

// definindo a pinagem da ponte h sem pwm
#define ledTeste 4
#define SBORDER_E 15
#define SBORDER_D 23
#define IR_RECEIVE_PIN 36
#define SDIST_L 18
#define SDIST_C 19
#define SDIST_R 5
#define SCL_SDIST 22
#define SDA_SDIST 21

VL53L0X sensorL;  //Sensor da esquerda
VL53L0X sensorC;  //Sensor da frente
VL53L0X sensorR;  //Sensor da direita

int control = 0;
int controlChangeLED = 0;
int ligado = 0;
bool running = false;

int distL;  //Valor lido pelo sensor da esquerda
int distC;  //Valor lido pelo sensor da frente
int distR;  //Valor lido pelo sensor da direita

unsigned long startTime;
unsigned long relativetime;

void setup() {
  motors.setup();
  pinMode(SBORDER_D,INPUT);
  pinMode(SBORDER_E,INPUT);
  pinMode(ledTeste,OUTPUT);
  digitalWrite(ledTeste, LOW);

  IrReceiver.begin(IR_RECEIVE_PIN);
  Serial.begin(9600);
  Wire.begin();

  pinMode(SDIST_L, OUTPUT);
  pinMode(SDIST_C, OUTPUT);
  pinMode(SDIST_R, OUTPUT);
  digitalWrite(SDIST_L, LOW);
  digitalWrite(SDIST_C, LOW);
  digitalWrite(SDIST_R, LOW);
  
  pinMode(SDIST_L, INPUT);
  sensorL.init(true);
  sensorL.setAddress((uint8_t)0x21); //endereço do sensor da esquerda
  pinMode(SDIST_C, INPUT);
  sensorC.init(true);
  sensorC.setAddress((uint8_t)0x23); //endereço do sensor da frente
  pinMode(SDIST_R, INPUT);
  sensorR.init(true);
  sensorR.setAddress((uint8_t)0x25); //endereço do sensor da direita

  sensorL.setTimeout(20);
  sensorC.setTimeout(20);
  sensorR.setTimeout(20);  
}

void loop() {
  if (IrReceiver.decode()){
    IrReceiver.resume();
    Serial.println(IrReceiver.decodedIRData.command);
    control = IrReceiver.decodedIRData.command;
    if (control == 1) {
      Serial.println("start");
      startTime = millis();
      motors.write(255, 255);
      running = true;
      digitalWrite(ledTeste, HIGH);
      delay(200);
      digitalWrite(ledTeste, LOW);
    } else if (control == 2) {
      Serial.println("stop");
      motors.write(0, 0);
      running = false;
      digitalWrite(ledTeste, HIGH);
      delay(200);
      digitalWrite(ledTeste, LOW);
    } else if (control == 3) {
      motors.write(-255, -255);
      Serial.println("de costas");
      digitalWrite(ledTeste, HIGH);
      delay(200);
      digitalWrite(ledTeste, LOW);
    } else if (control == 0) {
      digitalWrite(ledTeste, HIGH);
      delay(200);
      digitalWrite(ledTeste, LOW);
    }
  }

  if (running) {
    relativetime = millis() - startTime;

    Serial.print("Esquerda: ");
    Serial.print(digitalRead (SBORDER_E));
    Serial.print("    Direita: ");
    Serial.print(digitalRead (SBORDER_D));

    if (relativetime < 1000) {
      motors.write(255, 255);
    } else {
      motors.write(255, -255);
    }

  }
}