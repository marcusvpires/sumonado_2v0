#include <Arduino.h>
#include <analogWrite.h>
#include <IRremote.h>
#include <VL53L0X.h>    //sensores de distância
#include <Wire.h>       //Auxiliar dos sensores

// definindo a pinagem da ponte h sem pwm
#define Ain1 13
#define Ain2 12
#define Bin1 25
#define Bin2 26
#define pwmA 14
#define pwmB 27
#define ledTeste 4
#define SBORDER_E 15
#define SBORDER_D 23
#define IR_RECEIVE_PIN 36
#define SDIST_L 19
#define SDIST_C 18
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
unsigned long ledTime;

void writeMotors(int va, int vb) {
  if (va > (int) 0) {
    Serial.println("A HIGH LOW");
    digitalWrite(Ain2, LOW);
    digitalWrite(Ain1, HIGH);
  } else if (va < (int) 0) {
    Serial.println("A LOW HIGH");
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);
  } else {
    Serial.println("A LOW LOW");
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, LOW);
  }
  if (vb > (int) 0) {
    Serial.println("b HIGH LOW");
    digitalWrite(Bin2, LOW);
    digitalWrite(Bin1, HIGH);
  } else if (vb < (int) 0) {
    Serial.println("b LOW HIGH");
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, HIGH);
  } else {
    Serial.println("b HIGH HIGH");
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, LOW);
  }
  Serial.println(abs(va));
  Serial.println(abs(vb));
  analogWrite(pwmA, abs(va));
  analogWrite(pwmB, abs(vb));
}


void setup() {
  pinMode(pwmA,OUTPUT);
  pinMode(Ain1,OUTPUT);
  pinMode(Ain2,OUTPUT);
  pinMode(pwmB,OUTPUT);
  pinMode(Bin1,OUTPUT);
  pinMode(Bin2,OUTPUT);
  pinMode(SBORDER_D,INPUT);
  pinMode(SBORDER_E,INPUT);
  pinMode(ledTeste,OUTPUT);

  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, LOW);
  analogWrite(pwmA, 0);
  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, LOW);
  analogWrite(pwmB, 0);
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

  sensorL.setTimeout(100);
  sensorC.setTimeout(100);
  sensorR.setTimeout(100);  
}

void loop() {
  if (IrReceiver.decode()){
    IrReceiver.resume();
    Serial.println(IrReceiver.decodedIRData.command);

    control = IrReceiver.decodedIRData.command;
    if (control == 1) {
      Serial.println("start");
      writeMotors(255, 255);
    } else if (control == 2) {
      Serial.println("stop");
      writeMotors(0, 0);
    } else if (control == 3) {
      writeMotors(-255, -255);
      Serial.println("de costas");
    }
    digitalWrite(ledTeste, HIGH);
    delay(200);
    digitalWrite(ledTeste, LOW);
  }
}