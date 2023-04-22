// importacao das bibliotecas
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

int direcao = 0;
int l = 0;
int j;

//Variáveis que vão receber os valores lidos dos pinos dos sensores
int distL;  //Valor lido pelo sensor da esquerda
int distC;  //Valor lido pelo sensor da frente
int distR;  //Valor lido pelo sensor da direita

// definindo a pinagem do sensor VL

void setup() {

  // definicao das pinagens
  
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

  //Iniciando o endereçamento dos sensores
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

  if(direcao == 0){
  analogWrite(pwmA, 255);
  analogWrite(pwmB, 255);
  
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);

  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  } 
  
  if(direcao == 1) {
  analogWrite(pwmA, 255);
  analogWrite(pwmB, 255);

  digitalWrite(Ain1, HIGH);
  digitalWrite(Ain2, LOW);

  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, HIGH);

  }

  if(direcao == 2){
  analogWrite(pwmA, 55);
  analogWrite(pwmB, 55);
  
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);

  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  } 
  
  if(direcao == 3) {
  analogWrite(pwmA, 55);
  analogWrite(pwmB, 55);

  digitalWrite(Ain1, HIGH);
  digitalWrite(Ain2, LOW);

  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, HIGH);

  }
  
  if(direcao == 4) {
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);

  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, LOW);

  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, LOW);

  }

  //Armazena os valores lidos nas respectivas variáveis
  distL = sensorL.readRangeSingleMillimeters();
  distC = sensorC.readRangeSingleMillimeters();
  distR = sensorR.readRangeSingleMillimeters();  
  
  j++;
  if(j > 15){
  Serial.print("Esquerda: ");
  Serial.print(digitalRead (SBORDER_E));
  Serial.print("    Direita: ");
  Serial.print(digitalRead (SBORDER_D));

  
  // Mostra o valor de cada sensor na tela e a decisão escolhida
  Serial.print("   L: ");
  Serial.print(distL);
  Serial.print("\t");
  Serial.print("C: ");
  Serial.print(distC);
  Serial.print("\t");
  Serial.print("R: ");
  Serial.print(distR);
  Serial.println("\t\t");

  j = 0;
  }

  if (IrReceiver.decode()){
      	IrReceiver.resume();
      	Serial.println(IrReceiver.decodedIRData.command);

        direcao = IrReceiver.decodedIRData.command;
        
        if(direcao == 0 || direcao == 1 || direcao == 2 || direcao == 3 || direcao == 4 )
        if(l == 0){
            digitalWrite(ledTeste, HIGH);
            l = 1;
        } else {
            digitalWrite(ledTeste, LOW);
            l = 0;
        }
        delay(30);
         
  }
}