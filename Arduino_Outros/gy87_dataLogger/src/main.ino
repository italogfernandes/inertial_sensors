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
               [MXH] [MXL] [MYH] [MYL] [MZH] [MZL]
   ['\n']

    Esquema de montagem:
    Arduino - Dispositivo
    13      - LED
    A4      - SDA do GY-87
    A5      - SCL do GY-87
    3.3     - VCC_IN do GY-87
    GND     - GND do GY-87

   Para visualizar de forma legivel ao ser humano
   Altere o comentario em: #define DEBUG_MODE
*/
#define ARDUINO_DUE
//#define ARDUINO_UNO

#define DEBUG_MODE

//Se estiver no modo debug printa as msg debug, se nao estiver nao printa
#ifdef DEBUG_MODE
#define DEBUG_PRINT_(x) Serial.print(x)
#endif
#ifndef DEBUG_MODE
//#define DEBUG_PRINT_(x)
#endif

#ifdef ARDUINO_DUE
#include "DueTimer.h"
#endif
#ifdef ARDUINO_UNO
#include "Timer.h" //http://github.com/JChristensen/Timer
#endif

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13

//NOTE: Antes de usar vc deve alterar a frequenciana biblioteca mpu6050
//CASO ISSO NAO SEJA FEITO CORRE PERIGO DA FIFO ESTOURAR
#define MPUsampFreq 100
#define mpu_interval 10 //Each 10ms

#define PSDMP 42 //Packet Size DMP - tam do pacote interno da mpu-6050

#define UART_BAUDRATE 115200
#define UART_START '$' //Inicio do pacote
#define UART_END '\n' //Fim do pacote

//Variaveis Gerais
#ifdef ARDUINO_UNO
Timer t;
uint8_t timer_id;
#endif

uint8_t serial_buffer_out[22];
char serialOp;
bool aquisition_running = false;

//Variaveis Inercial
MPU6050 mpu(0x68);
const int offsets[6] = {  673, 1641, 1956, 36, -40, 6};
uint8_t fifoBuffer[42]; // FIFO storage fifoBuffer of mpu
int numbPackets;

void setup() {
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
  Serial.begin(UART_BAUDRATE);
  DEBUG_PRINT_("Insira um comando: '1' - Start, '2' - Stop.\n");
}

void loop() {
  #ifdef ARDUINO_UNO
  t.update();
  #endif
  //Menu
  if (Serial.available() > 0)
  {
    serialOp = Serial.read();
    if (serialOp == '1')
    {
      DEBUG_PRINT_("Comando Start Recebido.=\n");
      digitalWrite(LED_PIN, HIGH);
      mpu.resetFIFO();
      delay(5);
      #ifdef ARDUINO_DUE
      Timer3.attachInterrupt(ler_sensor_inercial).setFrequency(MPUsampFreq).start();
      #endif
      #ifdef ARDUINO_UNO
      timer_id = t.every(mpu_interval, ler_sensor_inercial); //Realiza leitura e envia pacote(ou mostra) dados a cada mpu_interval
      #endif
      aquisition_running = true;
    }
    else if (serialOp == '2')
    {
      digitalWrite(LED_PIN, LOW);
      aquisition_running = false;

      #ifdef ARDUINO_DUE
      Timer3.stop();
      #endif
      #ifdef ARDUINO_UNO
      t.stop(timer_id);
      #endif
    }
  }
}


////////////////////
//Sensor Inercial //
////////////////////

/***
New functions:

bool getTempFIFOEnabled();
void setTempFIFOEnabled(bool enabled);

bool getSlave0FIFOEnabled();
void setSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool getMultiMasterEnabled();
void setMultiMasterEnabled(bool enabled);
bool getWaitForExternalSensorEnabled();
void setWaitForExternalSensorEnabled(bool enabled);
bool getSlave3FIFOEnabled();
void setSlave3FIFOEnabled(bool enabled);
bool getSlaveReadWriteTransitionEnabled();
void setSlaveReadWriteTransitionEnabled(bool enabled);
uint8_t getMasterClockSpeed();
void setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t getSlaveAddress(uint8_t num);
void setSlaveAddress(uint8_t num, uint8_t address);
uint8_t getSlaveRegister(uint8_t num);
void setSlaveRegister(uint8_t num, uint8_t reg);
bool getSlaveEnabled(uint8_t num);
void setSlaveEnabled(uint8_t num, bool enabled);
bool getSlaveWordByteSwap(uint8_t num);
void setSlaveWordByteSwap(uint8_t num, bool enabled);
bool getSlaveWriteMode(uint8_t num);
void setSlaveWriteMode(uint8_t num, bool mode);
bool getSlaveWordGroupOffset(uint8_t num);
void setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t getSlaveDataLength(uint8_t num);
void setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_MST_STATUS register
bool getPassthroughStatus();
bool getSlave4IsDone();
bool getLostArbitration();
bool getSlave4Nack();
bool getSlave3Nack();
bool getSlave2Nack();
bool getSlave1Nack();
bool getSlave0Nack();


// setup AK8975 (0x0E) as Slave 0 in read mode
DEBUG_PRINTLN(F("Setting up AK8975 read slave 0..."));
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_ADDR, 0x8E);
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_REG,  0x01);
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_CTRL, 0xDA);

// setup AK8975 (0x0E) as Slave 2 in write mode
DEBUG_PRINTLN(F("Setting up AK8975 write slave 2..."));
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_ADDR, 0x0E);
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_REG,  0x0A);
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_CTRL, 0x81);
I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_DO,   0x01);


**/
void iniciar_sensor_inercial() {
  if (mpu.testConnection()) {
    mpu.initialize(); //Initializes the IMU
    uint8_t ret = mpu.dmpInitialize(); //Initializes the DMP
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true);
      mpu.setXAccelOffset(offsets[0]); mpu.setYAccelOffset(offsets[1]); mpu.setZAccelOffset(offsets[2]);
      mpu.setXGyroOffset(offsets[3]); mpu.setYGyroOffset(offsets[4]); mpu.setZGyroOffset(offsets[5]);
      DEBUG_PRINT_("Sensor Inercial configurado com sucesso.\n");
    } else {
      //TODO: adicionar uma forma melhor de aviso. outro led?
      DEBUG_PRINT_("Erro na inicializacao do sensor Inercial!\n");
    }
  } else {
    DEBUG_PRINT_("Erro na conexao do sensor Inercial.\n");
  }
}

void ler_sensor_inercial() {
  numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
  if (numbPackets >= 24) {
    mpu.resetFIFO();
    DEBUG_PRINT_("FIFO sensor 1 overflow!\n"); //TODO: mostrar isso de alguma forma. outro led?
  } else if (numbPackets > 0) {
    while (numbPackets > 0) {
      mpu.getFIFOBytes(fifoBuffer, PSDMP);
      numbPackets--;
    }
    enviar_pacote_inercial();
  }
}

void enviar_pacote_inercial() {
#ifndef DEBUG_MODE
  //Assembling packet and sending
  serial_buffer_out[0] = UART_START; //['$']
  //Quaternion
  serial_buffer_out[1] = fifoBuffer[0];  serial_buffer_out[2] = fifoBuffer[1];    //[QWH] [QWL]
  serial_buffer_out[3] = fifoBuffer[4];  serial_buffer_out[4] = fifoBuffer[5];    //[QXH] [QXL]
  serial_buffer_out[5] = fifoBuffer[8];  serial_buffer_out[6] = fifoBuffer[9];    //[QYH] [QYL]
  serial_buffer_out[7] = fifoBuffer[12]; serial_buffer_out[8] = fifoBuffer[13];   //[QZH] [QZL]
  //Aceleracao
  serial_buffer_out[9] = fifoBuffer[28]; serial_buffer_out[10] = fifoBuffer[29];  //[AXH] [AXL]
  serial_buffer_out[11] = fifoBuffer[32]; serial_buffer_out[12] = fifoBuffer[33]; //[AYH] [AYL]
  serial_buffer_out[13] = fifoBuffer[36]; serial_buffer_out[14] = fifoBuffer[37]; //[AZH] [AZL]
  //Giroscopio
  serial_buffer_out[15] = fifoBuffer[16]; serial_buffer_out[16] = fifoBuffer[17]; //[GXH] [GXL]
  serial_buffer_out[17] = fifoBuffer[20]; serial_buffer_out[18] = fifoBuffer[21]; //[GYH] [GYL]
  serial_buffer_out[19] = fifoBuffer[24]; serial_buffer_out[20] = fifoBuffer[25]; //[GZH] [GZL]

  serial_buffer_out[21] = UART_END; //['\n']
  Serial.write(serial_buffer_out, 22);
#endif /*DEBUG_MODE*/
#ifdef DEBUG_MODE
  float q[4], a[3], g[3];
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
  DEBUG_PRINT_("\n");
#endif /*DEBUG_MODE*/
}
