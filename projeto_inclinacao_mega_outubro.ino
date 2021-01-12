/*
 * Author/Coder : Bruno A. Patrocinio
 * Version : 4.0.0 (Acelerometro MPU6050 e LCD Display) 19/07/2020
 */

#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include <nRF24L01.h>
#include <RF24.h>

//Instância do objeto da classe RF24
RF24 radio(7, 8);

//objeto acelerômetro
MPU6050 mpu;

// I2C address of the MPU-6050
const int endereco = 0x68; 

// endereço para envio das informações Transm e recep. devem ter o mesmo endereço
const byte address[6] = "00002";

//variáveis de leitura brutas estas não permitem valores negativos
unsigned int acelX, acelY, acelZ, temperatura;
unsigned int giroX, giroY, giroZ;

//variáveis de contador
long timer = 0;

//variaveis globais gerais
float data;
double eixoX, eixoY, eixoZ;
double mediaX = 0;

//prototipos de função
void printaSerial();
void filtro();

void setup() {
  //RF
  Serial.begin(9600);
  radio.begin(); //inicia a comunicação sem fio RF

  radio.openWritingPipe(address); //define o endereço p/ envio de dados ao recept.
  radio.setPALevel(RF24_PA_HIGH); //define o nível do amplificador de potência
  radio.stopListening(); //define o modulo como transmis. apenas
  
  //Acelerômetro
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
  Wire.write(0b00000000);
  Wire.beginTransmission(endereco);//inicia a transmissão de Wire
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // estabelece em 0, acorda o MPU6050
  Wire.endTransmission(true);

  mpu.initialize();
  //verifica a conexão e informa a versão
  Serial.println("CODE INCLINACAO MPU6050_2.0.5_COM FUNCAO MAP E LCD_TFT updated 02_05");
  Serial.println("Testando conexão do dispositivo...");

  Serial.println(mpu.testConnection() ? "MPU6050 conectado com sucesso!" : "MPU6050 conexão falhou.");
  mpu.setXAccelOffset(0);

}//fim de setup

void loop() {
 // Wire.beginTransmission(endereco);
 // Wire.write(0x3B); // inicia com o registro 0x3B (ACCEL_XOUT_H)
 // Wire.endTransmission(false);
 // Wire.requestFrom(endereco, 14, true); // requisita um total de 14 registros

  
  mpu.getMotion6(&acelX, &acelY, &acelZ, &giroX, &giroY, &giroZ);
  
  // remap das leituras função map()
  eixoX = map(acelX, 0, 1023, 0, 255);
  eixoY = map(acelY, 0, 1023, 0, 255);
  eixoZ = map(acelZ, 0, 1023, 0, 255);
  
  // chama as funções modularizadas
  printaSerial();
  filtro();
  
}// fim de loop

void printaSerial(){ //printa a cada 1 seg. usando a função millis()
  if(millis() - timer > 1000){
    //char saida[5];
  Serial.print("AcX = "); Serial.print(acelX);
  Serial.print(" | AcY = "); Serial.print(acelY);
  Serial.print(" | AcZ = "); Serial.print(acelZ);
  Serial.print(" | Tmp = "); Serial.print(temperatura / 340.00 + 36.53); // converte a temperatura em graus C
  Serial.print(" | GyX = "); Serial.print(giroX);
  Serial.print(" | GyY = "); Serial.print(giroY);
  Serial.print(" | GyZ = "); Serial.print(giroZ);
  Serial.print(" | Media X = ");Serial.print(mediaX);
  Serial.print(" | Eixo X = ");Serial.println(eixoX);

  data = mediaX;
  
  radio.write(&data, sizeof(data));
  //radio.write(&text, sizeof(text));
 
  timer = millis();
  }
}//fim de printaSerial

void filtro(){
  //faz uma media para filtro passa baixa
  double amostras[64];
  double soma = 0;
  int i,o;
  for(int i=0; i<=64; i++){
    amostras[i] = eixoX;
  }
  soma = soma += amostras[i];
  mediaX = soma / 64;

  //verifica posição X
  //... a inserir ainda
}//fim de filtro
