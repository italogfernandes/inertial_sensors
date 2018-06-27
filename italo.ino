/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   BIOLAB - Biomedical Engineering Lab

   Autor: Ítalo G S Fernandes
   contact: italogsfernandes@gmail.com
   URLs: www.biolab.eletrica.ufu.br
          https://github.com/BIOLAB-UFU-BRAZIL
   Este códido faz parte do projeto da competição do cobec.
   O que faz:
      Realiza leitura de 1 sensor inercial e envia para pc
      Realiza leitura de um emg, filtra e envia para pc

   TODO:
     Adicionar Timer de aquisicao confiavel
     Obter offsets do sensor inercial
     Verificar "NOTE" espalhados no codigo.
     Verificar "TODO" espalhados no codigo.

   Pacotes:
   Inercial(Quaternion): (11 bytes)
   ['$'] ['Q'] [WH] [WL] [XH] [XL] [YH] [YL] [ZH] [ZL] ['\n']
   EMG(Valores do ADC passados após Media Móvel): (5 bytes)
   ['$'] ['E'] [EMGH] [EMGL] ['\n']

    Esquema de montagem:
    Arduino - Dispositivo
    13      - LED
    A0      - EMG-sinal (0 a 3.3V)
    GND     - EMG-GND
    A4      - SDA do GY-521
    A5      - SCL do GY-521
    5V      - VCC do GY-521
    GND     - GND do GY-521

   Para visualizar de forma visivel ao ser humano
   Altere o comentario em: #define DEBUG_MODE
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

Servo sg90;

int servo_pin = 3;
MPU6050 sensor;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 40 //Hz
#define mpu_interval 25 //Each 10ms

#define PSDMP 42 //Packet Size DMP - tam do pacote interno da mpu-6050

//Variaveis Gerais
//TODO: trocar esses millis por timer
unsigned long currentMillis = 0;
unsigned long previousMPUMillis = 0;

//Variaveis Inercial
MPU6050 mpu(0x68);
uint8_t fifoBuffer[42]; // FIFO storage fifoBuffer of mpu
int numbPackets;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {
  sg90.attach (servo_pin);
  //GPIO:
  pinMode(LED_PIN, OUTPUT);

  //Sensor Inercial
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(200000); //NOTE: Ajustar de acordo com arduino utilizado
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  iniciar_sensor_inercial();

  //Serial:
  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMPUMillis >= mpu_interval) {
    previousMPUMillis = currentMillis;
    ler_sensor_inercial(); //Realiza leitura e envia pacote(ou mostra) dados
    sg90.write(ypr[2] * 180 / M_PI);
  }
  

}


////////////////////
//Sensor Inercial //
////////////////////

void iniciar_sensor_inercial() {
  if (mpu.testConnection()) {
    mpu.initialize(); //Initializes the IMU
    uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true);
      //trocar
      mpu.setXAccelOffset(-4062);
      mpu.setYAccelOffset(-242);
      mpu.setZAccelOffset(1078);
      mpu.setXGyroOffset(207);
      mpu.setYGyroOffset(-124);
      mpu.setZGyroOffset(33);
      Serial.println("Sensor Inercial configurado com sucesso.\n");
    } else {
      //TODO: adicionar uma forma melhor de aviso. outro led?
      Serial.println("Erro na inicializacao do sensor Inercial!\n");
    }
  } else {
    Serial.println("Erro na conexao do sensor Inercial.\n");
  }
}

void ler_sensor_inercial() {
  numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  if (numbPackets >= 24) {
    mpu.resetFIFO();
    //  DEBUG_PRINT_("FIFO sensor 1 overflow!\n"); //TODO: mostrar isso de alguma forma. outro led?
  } else {
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      numbPackets--;
    }
    enviar_pacote_inercial();
  }
}

void enviar_pacote_inercial() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI); //USAR ESSE
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
}


