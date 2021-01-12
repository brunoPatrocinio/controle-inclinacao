/*
 * Author/Coder : Bruno A. Patrocinio
 * Version : 1.0.0 Receptor de Radiofrequência com Arduino nano
 */
#include <SPI.h> 
#include <nRF24L01.h> 
#include <RF24.h> 
 
RF24 radio(7, 8); //Cria o objeto radio, passando os pinos 7 e 8 (CE, CSN)
 
const byte address[6] = "00002"; //CRIA UM ENDEREÇO PARA ENVIO DOS
//DADOS (O TRANSMISSOR E O RECEPTOR DEVEM SER CONFIGURADOS COM O MESMO ENDEREÇO)
long timer = 0;
 
void setup() {
  Serial.begin(38400);
  radio.begin(); //Inicia a comunicação sem fio RF
  radio.openReadingPipe(0, address); //Define o endereço para recebimento
  radio.setPALevel(RF24_PA_HIGH); //Coloca o modulo RF em alta potência
  radio.startListening(); //Define o modulo para apenas recebimento
}
 
void loop() {
  if (radio.available()) { 
    float data;
    char text[32] = ""; //Var. que armazena os dados recebidos do emissor
   
    radio.read(&data, sizeof(data)); // Lê os dados

    if(millis() - timer > 500){
      Serial.println(data); // Imprime os dados no Monitor Serial
 
      timer = millis();
    }
    
  }
  
}
