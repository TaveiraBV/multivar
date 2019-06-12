/*
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Código de controle sem observador / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
*/

#include <Mpu6050.h>
#include <Servo.h>

// ======== CONFIGURAÇÕES =========
// Altere os valores conforme a montagem do seu projeto

// Pinos
const int pino_servo = 4;
const int pino_giroscopio = 9;

// Parâmetros servo
const int offset_servo = 38; /* Valor, em graus, que deixa o
                                giroscópio paralelo a moto.
                                Utilizar o programa teste_servo
                                para determinar o valor.
                             */
const int servo_maximo = 70; //  Limite superior da atuação do servo
const int servo_minimo = 5;  //  Limite inferior da atuação do servo
                              //  Novamente, utilize o arquivo teste_servo
                              //  para determinar o menor e maior valor para atuar

const bool inverter_direcao_servo = false; //  Modifique este parâmetro caso
                                           //  a direção da rotação do servo precise
                                           //  ser invertida (de modo que um aumento de u
                                           //  provoque um aumento de theta).

// Parâmetros IMU
const bool EIXO_X = true; /* A IMU pode ser montada de duas formas,
                             ou seja, de modo que a posição angular
                             da moto corresponda ao eixo X da IMU ou
                             ao eixo Y.
                             Valores:
                             true = eixo de rotação da moto corresponde
                                    ao eixo X da IMU
                             false = eixo de rotação da moto corresponde
                                    ao eixo Y da IMU
                           */
                         
const float offset_posicao_x = -3.3;    //  Valores de offset de posição e velocidade
const float offset_velocidade_x = 4.45; //  dos eixos X e Y. Apenas um eixo será utilizado,
const float offset_posicao_y = 6.16;   //  dependendo da configuração EIXO_X acima.
const float offset_velocidade_y = -1.94;  //  Para determinar os valores utilize o programa calibracao_imu

const bool inverter_imu = true; //  Análogo ao inverter_direcao_servo. Ajustar de modo que o sentido de rotação
                                 //  positivo da IMU seja o sentido positivo de rho
                                 
// Parâmetros de Controle
// Os estados do sistema, x1, x2 e x3 são realimentados
// através da lei de controle u = -K1*x1 + -K2*x2 + -K3*x3
// x1 -> Posição angular 'rho'
// x2 -> Posição angular do giroscópio 'theta'
// x3 -> Velocidade angular 'rho_ponto'

//const float K1 = -3.8843;
//const float K2 = -1.1952;
//const float K3 = -1.2513;
//
////
const float K1 = -4.0430;
const float K2 = -1.2910;
const float K3 = -1.3469;


//const float K1 = -3.9587;
//const float K2 = -1.8257;
//const float K3 = -1.8836;


const int Ts = 5;  // Período de amostragem, em ms.
                   // Menor valor aconselhado 5ms

// Opções de visualização
const bool plotar_referencia = false;
const bool plotar_x1 = true;
const bool plotar_x2 = true;
const bool plotar_x3 = true;
const bool plotar_atuacao_min_max = false;

//  ======= FIM DAS CONFIGURAÇÕES =========
//  Não é necessário editar nada abaixo desta linha

Mpu6050 imu;
Servo atuador;
Servo giroscopio;
unsigned long tempo_ultimo_controle;
float x1, x2, x3;
float du, dt;
float theta = 0;
float u;
int atuacao;

const float kGrausParaRadianos = M_PI/180.0;
const float kRadianosParaGraus = 180.0/M_PI;

void setup() {
 
  
  Serial.begin(115200);
  if(!imu.Begin()) {
    Serial.println("Erro ao configurar IMU");
    Serial.end();
    while(true);
  }

  imu.SetXPosOffset(offset_posicao_x);
  imu.SetYPosOffset(offset_posicao_y); 
  imu.SetXSpeedOffset(offset_velocidade_x);
  imu.SetYSpeedOffset(offset_velocidade_y);

  atuador.attach(pino_servo);
  atuador.write(offset_servo); //  Vai para a posição inicial
  
  giroscopio.attach(pino_giroscopio,1000,2000); //  Sequência de inicializaçao do giro

  giroscopio.writeMicroseconds(2000);
  delay(2000);
  giroscopio.writeMicroseconds(1000);
 
  delay(3000);
  
  giroscopio.writeMicroseconds(2000);
delay(6000);
  //giroscopio.writeMicroseconds(000);

 // giroscopio.writeMicroseconds(50000);
  //delay(12000); //  Espera o giroscópio acelerar para iniciar o controle

 

  tempo_ultimo_controle = millis();
}

void loop() {
  //  IMU deve ser atualizada o mais frequente possível devido aos filtros.
  imu.Update();
  dt = millis() - tempo_ultimo_controle;
  
  if (dt >= Ts) { // Loop de controle
    Rotation rot = imu.GetRotation();
    Velocity vel = imu.GetVelocity();
    
    x1 = EIXO_X ? rot.x : rot.y;
    x1 *= (inverter_imu ? -1 : 1);
    x1 *= kGrausParaRadianos;
    
    x2 = theta;
  
    x3 = EIXO_X ? vel.x : vel.y;
    x3 *= (inverter_imu ? -1 : 1);
    x3 *= kGrausParaRadianos;

    u = -K1*x1 -K2*x2 -K3*x3;

    theta += (u*dt)/1000.0; //  Converter dt para segundos
    // Atua
    atuacao = static_cast<int>((inverter_direcao_servo ? -1 : 1)*kRadianosParaGraus*theta + offset_servo);
    if (atuacao > servo_maximo) {
      atuacao = servo_maximo;
      theta = kGrausParaRadianos*(servo_maximo-offset_servo); //  Não pode deixar theta ficar crescendo!
    } else if (atuacao < servo_minimo) {
      atuacao = servo_minimo;
      theta = kGrausParaRadianos*(servo_minimo-offset_servo);
    }
    
    atuador.write(atuacao);
    tempo_ultimo_controle = millis();
    
    // Plots
    if (plotar_referencia) {
      Serial.print(0);
      Serial.print('\t');
    }
    if (plotar_x1) {
      Serial.print(x1*kRadianosParaGraus);
      Serial.print(',');
    }
    if (plotar_x2) {
      Serial.print(x2*kRadianosParaGraus);
      Serial.print(',');
    }
    if (plotar_x3) {
      Serial.print(x3*kRadianosParaGraus);
      Serial.print('\t');
    }
    //if (plotar_atuacao_min_max) {
      //Serial.print(kGrausParaRadianos*servo_maximo);
      //Serial.print('\t');
      //Serial.print(kGrausParaRadianos*servo_minimo);
      //Serial.print('\t');
    //}
    Serial.println();
  }
}
