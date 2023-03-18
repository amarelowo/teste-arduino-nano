#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SparkFun_TB6612.h>
#include <SparkFun_TB6612.cpp>
#include <IRremote.h>
#include <QTRSensors.h>


//Teste 2
// Criando o objeto para o sensor QTR-8RC
// Não há mais uma distinção de classes entre os sensores QTR-xA e QTR-xRC
QTRSensors qtr;


// Criando objeto para comunicacao com o bluetooth
// SoftwareSerial SerialBT(3,11);



const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//---------DEFINICAO DOS MOTORES---------//
//antes 245
//Testes com a 3s pararam em 200
#define velocidadeMaxima 200

//Ponte H
#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

//Offset
const int offsetA = 1;
const int offsetB = 1;

bool noSensor = false;

//Motor esquerdo
Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

//Motor direito
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
//-------------------------------------//

//---------VARIAVEIS AUXILIARES--------//

//---VALORES (35,5s)---
// KP 0,098
// KD 0,160
// velocidadeMaxima 245

// Testes com a 3ds:
// Com os valores (KP 0,098 | KD 0,160 | velocidadeMaxima 245) o seguidor consegue fazer a pista, mesmo com a 3s
// 1 - Aumentando o valor de velocidade máxima até começar a oscilar demais
// velocidadeMaxima = 110
// OBS: continua capaz de seguir a linha e o faz vagarosamente. É notável que mesmo em linhas retas começou a oscilar um pouco
// velocidadeMaxima = 120
// OBS: continua capaz de seguir a linha e o faz vagarosamente. Ainda oscila durante as retas e está fazendo curvas mais abertas
// velocidadeMaxima = 130
// OBS: continua capaz de seguir a linha.
//      Oscilando mais nas retas
//      Curvas continuam abertas
//      "Flickando" um pouco durante as curvas (checar suportes | pista)
// velocidadeMaxima = 140
// OBS: as mesmas observações anteriores ainda são válidas
//      Fez o percurso de testes em 11,94s
// velocidadeMaxima = 150
// OBS: as mesmas observações anteriores ainda são válidas
//      Fez o percurso de testes em 10,99s e 11,16s
//velocidadeMaxima = 160
// OBS: as mesmas observações anteriores ainda são válidas
//      Fez o percurso de testes em 10,25s e 10,35s
//velocidadeMaxima = 170
// OBS: as mesmas observações anteriores ainda são válidas
//      Curvasa ainda mais abertas**
//      Fez o percurso de testes em 9,62s e 9,73s
//velocidadeMaxima = 180
// OBS: as mesmas observações anteriores ainda são válidas
//      Fez o percurso de testes em 9,00s e 9,16s
//velocidadeMaxima = 190
// OBS: as mesmas observações anteriores ainda são válidas
//      Fez o percurso de testes em 9,00s e 9,16s
// Melhorando os valores do PID (KP = 0,111 | KD = 0,160 | velocidadeMaxima = 210)
//


//PID
const float KP = 0.098; //0,035
const float KD = 0.160; //0,105
const float KI = 0.000;
double integral = 0;
double erro = 0;
double ultimoErro = 0;
double objetivoLinha = 3500; //Para o seguidor seguir centrado

int leiturasBorda = 0;

//Variável de indicação de curvas
int curva;
bool isSensorOnCurve = false;


//Sensores de borda
#define sensorFim 10
#define sensorCurva 12


// Criando objeto receptor IR
IRrecv irrecv(sensorCurva);
decode_results results;

//-------------------------------------//

double tempoAntes;
double tempo;
double tempoParada;


void aguardaComandoIR();

double ajuste (unsigned int position)
{
  integral = integral + erro;
  
  double  ajuste = KP*erro + KD*(erro - ultimoErro) + KI*integral;

  ultimoErro = erro;

  return  ajuste;
}

void calibrar()
{
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setTimeout(1000);

  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Calibração durará cerca de 10 segundos
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

void verifica_chegada()//A gente já chegou? Função aplicada ao fim do laço loop que parará o seguidor caso encontre o indicativo de término de percurso
{
  int chegada = digitalRead(sensorFim);
//  Serial.print("(sensorFim) Sensor de chegada: ");
//  Serial.println(chegada);
  if(chegada == 1){
    if(noSensor == false){
      noSensor = true;
      leiturasBorda++;
    }
  }
  if(chegada == 0){
    if(noSensor == true){
      noSensor = false;
    }
  }

  tempoAntes = millis();
  
  if(leiturasBorda == 10){
    tempo = millis(); 
    leiturasBorda++;  
  }

//  Serial.print("tempoAntes: ");
//  Serial.println(tempoAntes);
//  Serial.print("tempo: ");
//  Serial.println(tempo);
//  Serial.print("leiturasBorda: ");
//  Serial.println(leiturasBorda);
  if(leiturasBorda >= 11 && (tempoAntes - tempo) >= 500)
  {
    motor1.drive(0);
    motor2.drive(0);
    delay(100000);
  }

//  tempoAntes == millis();
//
//  Serial.print("tempoAntes: ");
//  Serial.println(tempoAntes);
//  Serial.print("tempoParada");
//  Serial.println(tempoParada);
//  Serial.print("diferença: ");
//  Serial.println(tempoAntes - tempoParada);

//  if(tempoAntes - tempoParada >= 45000) {
//    motor1.drive(0);
//    motor2.drive(0);
//    delay(100000);
//  }

}

// void comunicationBT(){
//   if(SerialBT.available()){
//     String valorRecebido = SerialBT.readString();
//     Serial.println(valorRecebido);

//     //formato do dado recebido
//     //     kP     ki      kD
//     // 000.000;000.000;000.000
//     if(valorRecebido == "estado_a" || valorRecebido == "Cliente Conectado!" || valorRecebido == "Cliente Desconectado!"){
//       // Serial.println(valorRecebido);
//       SerialBT.print(valorRecebido);
//     }
//     else if(valorRecebido == "ok"){
//       SerialBT.print(valorRecebido);
//     }
//     else{
//       String stgkP = valorRecebido.substring(0,7);
//       double kP = stgkP.toDouble();
//       String stgkI = valorRecebido.substring(8,15);
//       double kI = stgkI.toDouble();
//       String stgkD = valorRecebido.substring(16,23);
//       double kD = stgkD.toDouble();
//       if((kP + kI + kD) < 100){
//         Serial.println("parece que deu certo essa caralha aqui");
//       }

//       // prova real de que o numero está chegando inteiro

//       Serial.print(kP, 6);
//       Serial.print(" | ");
//       Serial.print(kI, 6);
//       Serial.print(" | ");
//       Serial.println(kD, 6);
//       SerialBT.print(kP);

//     }
//   }
// }

void setup() {
  
  Serial.begin(9600);
  // SerialBT.begin(9600);
  irrecv.enableIRIn();
  
  // pinMode(sensorCurva, INPUT); //Sensor indicativo de curva
  pinMode(sensorFim, INPUT);  //Sensor indicativo de início término de circuito
  
  calibrar();

  aguardaComandoIR();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  tempoParada = millis();
}

int tempo1 = 0;

void loop(){
//  qtr.calibrate();
  
  int position = qtr.readLineWhite(sensorValues);

  erro = objetivoLinha - position;
  
  double correcao = ajuste(position);

//    if((millis() - tempo1) >= 10) {
//      position = qtr.readLineWhite(sensorValues);
//      correcao = ajuste(position);
//      tempo1 = millis();
//    }

  // Serial.print("Posicao: ");
  // Serial.println(position);

  // Serial.print("Erro: ");
  // Serial.println(erro);

  // Serial.print("ultimoErro: ");
  // Serial.println(ultimoErro);

  double d = erro - ultimoErro;
  // Serial.print("d: ");
  // Serial.println(d);

  // Serial.print("PID: ");
  // Serial.println(correcao);

  int m2 = constrain(velocidadeMaxima + correcao, 0, velocidadeMaxima);

  int m1 = constrain(velocidadeMaxima - correcao, 0, velocidadeMaxima);
  motor2.drive(m2);
  motor1.drive(m1);

  Serial.print(m1);
  Serial.print(" | ");
  Serial.println(m2);

  verifica_chegada();

}

void aguardaComandoIR(){
    Serial.println("fijfaoifj");
    while(true){
      if(irrecv.decode()){
        
        int data = irrecv.decodedIRData.decodedRawData;
        Serial.println(data);

        irrecv.resume();

        if(data == 128){
          Serial.print("Comando aceito, iniciando o robo: ");
          Serial.println(data);
          break;

        }
    }
    }
  
}