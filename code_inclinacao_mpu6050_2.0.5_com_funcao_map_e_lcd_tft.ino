/*
 * Author/Coder : Bruno A. Patrocinio
 * Version : 3.0.5 (Acelerometro MPU6050) 11/05/2019
 */
#include <I2Cdev.h>
#include<MPU6050.h>
#include <Wire.h>

//constantes sensor ultrassom
#define trigger 5  //pino de trigger na porta 5 por constante emitindo o pulso
#define echo 4    //pino echo na porta 4 atraves de uma constante ouvindo o pulso

//constantes dos pinos led e buzzer
const int pinoLedRed1 = 3;
const int pinoLedRed2 = 2; 
const int pinoLedGreen = 6;
const int pinoLedYellow = 7;
const int pinoLedBlue = 12;
const int pinoSom = 13;

double mediaX = 0.0;
double variancia = 0.0;
double desvioPadrao = 0.0;
double mediaGiro = 0.0;
double distCent = 0.0;
double distRefinada = 0.0;
double eixoZ;
long timer = 0;

//variaveis sensor ultrassom
float distancia = 0; // essa variavel receberá a distancia em metros
float centimetros = 0; //variavel de tempo para calcular qtd de centimetros
float milimetros = 0;
float tempo = 0;  //variavel de medida de retorno do pulso
bool traseiroCivic = false; //identifica vidro especifico 
bool traseiroCity = false;  //identifica vidro especifico
bool frontalCivic = false;  //identifica vidro especifico
bool frontalCity = false; //identifica vidro especifico
bool frontalFit = false;  //identifica vidro especifico
bool ehTraseiro = false;   //var. bool. que generaliza o vidro traseiro


#define G_GAIN 0.00875
#define AA 0.98

float acelx, acely, acelz, rate_gyr_x, rate_gyr_y, rate_gyr_z, gyroXangle, gyroYangle, gyroZangle;
float AccXangle, AccYangle, AccZangle, CFangleX, CFangleY, CFangleZ;
float const_calib = 16071.82;
float const_gravid = 9.81;

unsigned long pT;

//Protótipos de Função
void printSerial();
void verificaVidro();
void verificaPosX();

MPU6050 mpu6050; // objeto acelerometro

unsigned int ax, ay, az;
unsigned int gx, gy, gz;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  unsigned long pT = 0; // contador para determinar tempo de inicialização
    
    Serial.begin(38400);
    Wire.write(0b00000000);
    
    // config iniciais pinos ultrassom
    pinMode(trigger, OUTPUT);                                 
    digitalWrite(trigger, LOW);  
    pinMode(echo, INPUT);  
    
    //config iniciais dos leds e buzzer sound
    pinMode(pinoLedRed1, OUTPUT); //setando pino led verm. como saida.
    pinMode(pinoLedRed2, OUTPUT); //setando pino led verm. como saida.
    pinMode(pinoLedGreen, OUTPUT); //setando pino led verde como saida.
    pinMode(pinoLedYellow, OUTPUT);//setando pino led amarelo como saida.
    pinMode(pinoLedBlue, OUTPUT);//setando pino led azul como saida.
    pinMode(pinoSom, OUTPUT); //setando pino do som como saída.
    
    //os pinos de led verm e amarelo iniciam em LOW e o verde em HIGH
    digitalWrite(pinoLedRed1, LOW);
    digitalWrite(pinoLedRed2, LOW);
    digitalWrite(pinoLedYellow, LOW);
    digitalWrite(pinoLedBlue, LOW);
    digitalWrite(pinoLedGreen, HIGH);
  
    //iniciando disp.
    Serial.println("Iniciando...");
    mpu6050.initialize();

    //Executa o teste do dispositivo
    Serial.println("Testando conexão do dispositivo...");
    Serial.println(mpu6050.testConnection() ? "MPU6050 conectado com sucesso!" : "MPU6050 conexão falhou.");

}

void loop() {
  // leituras brutas
  mpu6050.getMotion6(&ay, &ax, &az, &gx, &gy, &gz);

  unsigned long cT = micros(); // contar tempo de loop

  unsigned long dT = cT - pT;
  pT = cT;
  
  acelx = ax * const_gravid / const_calib;
  acely = ay * const_gravid / const_calib;
  acelz = az * const_gravid / const_calib;

   // Converte valor do acelerometro com base nos 3 eixos
  AccXangle = (atan2(ax, sqrt(pow(ay,2) + pow(az,2)))*180) / 3.14;
  AccYangle = (atan2(ay, sqrt(pow(ax,2) + pow(az,2)))*180) / 3.14;
  AccZangle = (atan2(az, sqrt(pow(ax,2) + pow(ay,2)))*180) / 3.14;
  
  // Converte valor do giro em graus por seg
  // multiplicando uma contante relacionada à taxa de amostragem do sensor
  // nesse caso, a taxa é +-250g -> 0.00875
  rate_gyr_x = gx*G_GAIN;
  rate_gyr_y = gy*G_GAIN;
  rate_gyr_z = gz*G_GAIN;
  
  // Calcula a distância percorrida por integração simples
  // com base no tempo de loop (dT = cT – pT)
  gyroXangle+=rate_gyr_x*dT;
  gyroYangle+=rate_gyr_y*dT;
  gyroZangle+=rate_gyr_z*dT;
  
  // Fusão dos dados: giro + accel
  // Métodos: filtro complementar ou filtro de kalman
  // Optei pelo Filtro Complementar por ser mais simples de se aplicar do que o Filtro de Kalman.
  // Eficiencia bastante satisfatoria, segundo teoria
  // Atribui peso de 0.98 ao valor do giro e 0.02 ao acelerometro
  // O giroscópio tem leitura muito boa, mas também apresenta oscilação do valor.
  // Acelerômetro, por outro lado, é muito ruidoso, mas o desvio é zero.

  CFangleX=AA* (CFangleX+rate_gyr_x* (dT/1000000)) +(1 - AA) * AccXangle;
  CFangleY=AA* (CFangleY+rate_gyr_y* (dT/1000000)) +(1 - AA) * AccYangle;
  CFangleZ=AA* (CFangleZ+rate_gyr_z* (dT/1000000)) +(1 - AA) * AccZangle;
  
  //remap do eixo Z
  eixoZ = map(az, 0, 1023, 0, 255);
  
  //chama funções
  printSerial();
  verificaVidro();
  verificaPosX();
  
}//fim de loop

void printSerial(){
  if(millis() - timer > 1000){
  Serial.println("Dados:");
  Serial.print("X raw: ");Serial.print(ax);
  Serial.print(" Y raw: ");Serial.print(ay);
  Serial.print(" Z raw: ");Serial.print(az);
  Serial.print(" Giro Xraw: ");Serial.print(gx);
  Serial.print(" Media Giro X: ");Serial.print(mediaGiro);
  Serial.print(" Media X: ");Serial.print(mediaX);
  Serial.print(" Centimetros: ");Serial.print(centimetros);
  Serial.print(" Dist Metros: ");Serial.print(distCent);
  Serial.print(" Milimetros: ");Serial.print(milimetros);
  Serial.print(" Variancia: ");Serial.print(variancia);
  Serial.print(" Desvio Padrao: ");Serial.print(desvioPadrao);
  Serial.print(" Angulo X: ");Serial.print(CFangleX);
  Serial.print("\n");

  timer = millis();
  }
}//fim de printSerial

void verificaPosX(){
  //faz uma media de 100 amostras para estabilização dos dados do eixo X
  double amostras[10];
  double soma = 0.0;
  double vetorVar[10];
  double potenciaVar[10];
  int i, o, p;
  for(int i = 0; i <= 9; i++){
    amostras[i] = ax;
  }
  soma = soma += amostras[i];
  mediaX = soma / 10;
  
  //calcula a variancia
  for(p = 0; p <=9; p++){
    vetorVar[p] = amostras[i] - mediaX;
    potenciaVar[p] = pow(vetorVar[p],2);
  }
  variancia = (variancia += potenciaVar[p]) / 10;

 //calcula o desvio padrão da variancia das amostras
 desvioPadrao = sqrt(variancia);

  //faz uma media de 100 amostras para estabilizar o giroscopio
  double amostraGiro[10];
  double somaGiro = 0.0;
  for(int o=0; o<=9; o++){
    amostraGiro[i] = gx;
  }
  somaGiro = somaGiro += amostraGiro[i];
  mediaGiro = somaGiro / 10;
  
  if(eixoZ >= 2300 && ehTraseiro == true)
  {
  if(mediaX <= 215){
    delay(700);
    if(mediaX <= 215){
    Serial.print("Não Vira\n");
    digitalWrite(pinoLedRed1, HIGH);
    tone(pinoSom, 3000, 300); //speaker apita 
    digitalWrite(pinoLedGreen, LOW);
   }
 }
  else if(mediaX >= 722){
    delay(700);
    if(mediaX >= 722){
    digitalWrite(pinoLedRed2, HIGH);
    Serial.print("Não Vira\n");
    tone(pinoSom, 3000, 300); //speaker apita
    digitalWrite(pinoLedGreen, LOW);
  }
 }
  else{
    digitalWrite(pinoLedRed1, LOW); // deixa o pino dos leds em LOW
    digitalWrite(pinoLedRed2, LOW);
    digitalWrite(pinoLedGreen, HIGH);
    noTone(pinoSom);
  }
  }else{
    digitalWrite(pinoLedRed1, LOW); // deixa o pino dos leds em LOW
    digitalWrite(pinoLedRed2, LOW);
    digitalWrite(pinoLedGreen, HIGH);
    noTone(pinoSom);
  }
}//fim de verificaPosX

void verificaVidro(){
  int i;
  double soma = 0;
  double amostrasD[100];
  int contador = 0;
  digitalWrite(trigger,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger,LOW);
  tempo = pulseIn(echo,HIGH);    
  tempo = tempo/1000000;
  distancia = (tempo * 340)/2; //calcula distancia em metros
  centimetros = distancia * 100; //calcula distancia em centimetros
  milimetros = centimetros / 0.10;//calcula em milimetros
   for(int i = 0; i<=99; i++){//pega 100 amostras 
    amostrasD[i] = distancia;
  } 
    soma = soma += amostrasD[i]; //faz media
    distRefinada = soma / 100;
    distCent = distRefinada *100;

  if(centimetros >= 2 && centimetros <= 10){
    Serial.println("Possui um vidro traseiro do city");
    digitalWrite(pinoLedYellow,HIGH);
    traseiroCivic = true;
    ehTraseiro = true;
  }
  else if(centimetros >=11 && centimetros <= 14){
    Serial.println("Possui um vidro traseiro do civic");
    digitalWrite(pinoLedYellow, HIGH);
    traseiroCity = true;
    ehTraseiro = true;
  }
  else if(centimetros >=15 && centimetros <=18){
    Serial.println("Possui um vidro frontal do Civic");
    digitalWrite(pinoLedBlue, HIGH);
    ehTraseiro = true; //aqui está assim só para acionar a verificação da posiçãol
    frontalCivic = true;
  }
  else if(centimetros >=19 && centimetros <=22){
    Serial.println("Possui um vidro frontal do Fit");
    digitalWrite(pinoLedBlue, HIGH);
    ehTraseiro = true; //aqui está assim só para acionar a verificação da posição
    frontalFit = true;
  }
  else if(centimetros >=23 && centimetros <=26){
    Serial.println("Possui um vidro frontal do City");
    digitalWrite(pinoLedBlue, HIGH);
    ehTraseiro = true; //aqui está assim só para acionar a verificação da posição
    frontalCity = true;
    
  }
  else{
    digitalWrite(pinoLedYellow,LOW);
    digitalWrite(pinoLedBlue, LOW);
    traseiroCivic = false;
    traseiroCity = false;
    frontalCivic = false;
    frontalCity = false;
    frontalFit = false;
    ehTraseiro = false;
  }

}//fim verificaVidro
