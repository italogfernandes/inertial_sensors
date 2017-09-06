/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   BIOLAB - Biomedical Engineering Lab

   Autor: Ítalo G S Fernandes
   contact: italogsfernandes@gmail.com
   URLs: www.biolab.eletrica.ufu.br
          https://github.com/BIOLAB-UFU-BRAZIL
   O que faz:
      Realiza leitura de 1 sensor inercial e envia para pc

   TODO:
     Obter offsets do sensor inercial
     Verificar "NOTE" espalhados no codigo.
     Verificar "TODO" espalhados no codigo.

   Pacotes:
   Inercial(Quaternion-Accel-Gyro-Mag): (11 bytes)
   ['$']
   [QWH] [QWL] [QXH] [QXL] [QYH] [QYL] [QZH] [QZL]
               [AXH] [AXL] [AYH] [AYL] [AZH] [AZL]
               [GXH] [GXL] [GYH] [GYL] [GZH] [GZL]
   ['\n']

    Esquema de montagem:
    Arduino - Dispositivo
    13      - LED
    A4      - SDA do GY-521
    A5      - SCL do GY-521
    5V      - VCC do GY-521
    GND     - GND do GY-521

   Para visualizar de forma legivel ao ser humano
   Altere o comentario em: #define DEBUG_MODE
*/
//#define DEBUG_MODE

//Se estiver no modo debug printa as msg debug, se nao estiver nao printa
#ifdef DEBUG_MODE
#define DEBUG_PRINT_(x) Serial.print(x)
#endif
#ifndef DEBUG_MODE
#define DEBUG_PRINT_(x)
#endif

#include "Timer.h"                     //http://github.com/JChristensen/Timer

#define LED_PIN 13

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 100
#define mpu_interval 10 //Each 10ms

#define UART_BAUDRATE 115200
#define UART_START '$' //Inicio do pacote
#define UART_END '\n' //Fim do pacote

//Variaveis Gerais
Timer t;
uint8_t timer_id;
uint8_t serial_buffer_out[22];
char serialOp;
bool aquisition_running = false;

float qf[4], af[3], gf[3];
uint16_t q[4], a[3], g[3];

float QUAT_IDENTITY[] = {1.0, 0.0, 0.0, 0.0};
float QUAT_1[] = { 0.7071067811865476, 0.0, 0.0, 0.7071067811865476};
float QUAT_2[] = { 0.7071067811865476, 0.0, 0.0, -0.7071067811865476};

uint32_t value_idx = 0;

uint8_t fifoBuffer[42];
void setup() {
  //GPIO:
  pinMode(LED_PIN, OUTPUT);

  //Serial:
  Serial.begin(UART_BAUDRATE);
  DEBUG_PRINT_("Insira um comando: '1' - Start, '2' - Stop.\n");
}

void loop() {
  t.update();
  //Menu
  if (Serial.available() > 0)
  {
    serialOp = Serial.read();
    if (serialOp == '1')
    {
      digitalWrite(LED_PIN, HIGH);
      delay(5);
      timer_id = t.every(mpu_interval, ler_sensor_inercial); //Realiza leitura e envia pacote(ou mostra) dados a cada mpu_interval
      aquisition_running = true;
    }
    else if (serialOp == '2')
    {
      digitalWrite(LED_PIN, LOW);
      aquisition_running = false;
      t.stop(timer_id);
    }
  }
}


////////////////////
//Sensor Inercial //
////////////////////

void ler_sensor_inercial() {
  value_idx = (value_idx + 1) % 400;
  for (int i = 0; i < 4; i++) {
    qf[i] = 0.0;
    if (i < 3) {
      af[i] = i == 2 ? 1.0 : 0.0;
      gf[i] = 0.0;
    }
    if (value_idx < 100) {
      qf[i] = ((100.0 - value_idx) * QUAT_IDENTITY[i] + value_idx * QUAT_1[i]) / 100;
      if (i < 3) {
        gf[i] = i == 2 ? 90.0 : 0.0;
      }
    } else if (value_idx < 200) {
      qf[i] =  ((100.0 - value_idx % 100) * QUAT_1[i] + (value_idx % 100) * QUAT_IDENTITY[i]) / 100;
      if (i < 3) {
        gf[i] = i == 2 ? -90.0 : 0.0;
      }
    } else if (value_idx < 300) {
      qf[i] = ((100.0 - value_idx % 100) * QUAT_IDENTITY[i] + (value_idx % 100) * QUAT_2[i]) / 100;
      if (i < 3) {
        gf[i] = i == 2 ? -90.0 : 0.0;
      }
    } else if (value_idx < 400) {
      qf[i] = ((100.0 - value_idx % 100) * QUAT_2[i] + (value_idx % 100) * QUAT_IDENTITY[i]) / 100;
      if (i < 3) {
        gf[i] = i == 2 ? 90.0 : 0.0;
      }
    }
    q[i] = (uint16_t) (qf[i] * 16384);
    if (i < 3) {
      a[i] = (uint16_t) (af[i] * 8192);
      g[i] = (uint16_t) (gf[i] * 131);
    }
  }
  enviar_pacote_inercial();
}

void enviar_pacote_inercial() {
#ifndef DEBUG_MODE
  //Assembling packet and sending
  serial_buffer_out[0] = UART_START; //['$']
  //Quaternion
  serial_buffer_out[1] = (uint8_t) (q[0] >> 8);  serial_buffer_out[2] = (uint8_t) q[0];    //[QWH] [QWL]
  serial_buffer_out[3] = (uint8_t) (q[1] >> 8);  serial_buffer_out[4] = (uint8_t) q[1];    //[QXH] [QXL]
  serial_buffer_out[5] = (uint8_t) (q[2] >> 8);  serial_buffer_out[6] = (uint8_t) q[2];    //[QYH] [QYL]
  serial_buffer_out[7] = (uint8_t) (q[3] >> 8);  serial_buffer_out[8] = (uint8_t) q[3];   //[QZH] [QZL]
  //Aceleracao
  serial_buffer_out[9] = (uint8_t) (a[0] >> 8);  serial_buffer_out[10] = (uint8_t) a[0];  //[AXH] [AXL]
  serial_buffer_out[11] = (uint8_t) (a[1] >> 8);  serial_buffer_out[12] = (uint8_t) a[1]; //[AYH] [AYL]
  serial_buffer_out[13] = (uint8_t) (a[2] >> 8);  serial_buffer_out[14] = (uint8_t) a[2]; //[AZH] [AZL]
  //Giroscopio
  serial_buffer_out[15] = (uint8_t) (g[0] >> 8);  serial_buffer_out[16] = (uint8_t) g[0]; //[GXH] [GXL]
  serial_buffer_out[17] = (uint8_t) (g[1] >> 8);  serial_buffer_out[18] = (uint8_t) g[1]; //[GYH] [GYL]
  serial_buffer_out[19] = (uint8_t) (g[2] >> 8);  serial_buffer_out[20] = (uint8_t) g[2]; //[GZH] [GZL]

  serial_buffer_out[21] = UART_END; //['\n']
  Serial.write(serial_buffer_out, 22);
#endif /*DEBUG_MODE*/
#ifdef DEBUG_MODE

  /*
    //Quaternion
    q[0] = (float) ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0f;
    q[1] = (float) ((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0f;
    q[2] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
    q[3] = (float) ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0f;
    //Aceleracao
    a[0] = (float) ((fifoBuffer[28] << 8) | fifoBuffer[29]) / 8192.0f;
    a[1] = (float) ((fifoBuffer[32] << 8) | fifoBuffer[33]) / 8192.0f;
    a[2] = (float) ((fifoBuffer[36] << 8) | fifoBuffer[37]) / 8192.0f;
    //Giroscopio
    g[0] = (float) ((fifoBuffer[16] << 8) | fifoBuffer[17]) / 131.0f;
    g[1] = (float) ((fifoBuffer[20] << 8) | fifoBuffer[21]) / 131.0f;
    g[2] = (float) ((fifoBuffer[24] << 8) | fifoBuffer[25]) / 131.0f;
    DEBUG_PRINT_("Quat-Accel-Gyro:\t");
    //Quaternions
    DEBUG_PRINT_(q[0]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(q[1]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(q[2]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(q[3]);
    DEBUG_PRINT_("\t-\t");
    //accel in G
    DEBUG_PRINT_(a[0]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(a[1]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(a[2]);
    DEBUG_PRINT_("\t-\t");
    //gyro in degrees/s
    DEBUG_PRINT_(g[0]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(g[1]);
    DEBUG_PRINT_("\t");
    DEBUG_PRINT_(g[2]);
    DEBUG_PRINT_("\n");*/
  DEBUG_PRINT_("Quat-Accel-Gyro:\t");
  //Quaternions
  DEBUG_PRINT_(q[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[2]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[3]);
  DEBUG_PRINT_("\t-\t");
  //accel in G
  DEBUG_PRINT_(a[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[2]);
  DEBUG_PRINT_("\t-\t");
  //gyro in degrees/s
  DEBUG_PRINT_(g[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[2]);
  DEBUG_PRINT_("\n");
#endif /*DEBUG_MODE*/
}

