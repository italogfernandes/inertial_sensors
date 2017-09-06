#include "nrf24le1.h"
#include "reg24le1.h"
#include "nRF-SPIComands.h"
#include "hal_w2_isr.h" 
#include "hal_delay.h"
#include "timer0.h"

//Subenderecos usados no sistema
#define HMC_endereco			0x1E //Endereco I2C do HMC5883(Magnetometro):
#define MPU_endereco			0x68 //Endereco I2C do MPU6050

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_PWR_MGMT_1		0x6B
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_INT_PIN_CFG 0x37
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1

#define STATUS_LED P02

uint8_t mpu6050_buffer[14];
uint8_t hmc_buffer[14];
uint8_t hmc_start_byte = 0x03;
////////////////////////
//Functions in Sensor //
////////////////////////

void configurarSensores();
void DataAcq();
void send_dada_read();
void sendString(const char* msg, uint8_t msglen);
///////////////////
//Implementation //
///////////////////

/**
* Seta os pinos do nrf como saidas e entradas de acordo com as funcoes desejadas
*/
void iniciarIO(void){
		P0DIR = 0x00; P1DIR = 0x00; // Tudo input
    P0CON = 0x00; P1CON = 0x00; //Reseting PxCON registers
    P0DIR &= ~(1<<4);//P04 = w2scl = output
    P0DIR &= ~(1<<2);//P02 = Status led = output
    P1CON |= 0x53; // All general I/O 0101 0011
}


void setup(){
    iniciarIO(); //Entradas e Saidas
    rf_init(ADDR_HOST,ADDR_HOST,30,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm); //RF Channel
    hal_w2_configure_master(HAL_W2_100KHZ); //I2C
    configurarSensores();//MPU_6050 and HMC5883
		setup_T0_freq(200,1);
		start_T0();
		STATUS_LED = 1; //Acende o Led
}

void main(void) {
    setup();
    while(1){ //Loop
			if(timer_elapsed){
				timer_elapsed = 0;
				DataAcq();
				send_dada_read();
			}
			//delay_ms(500);
    }/*END LOOP*/
}/*END MAIN*/

///////////////////////
//FUNCIONS in Sensor //
///////////////////////

void configurarSensores(){
	//Starting MPU6050
	i2c_mpu_writeByte(MPU_endereco, MPU6050_RA_PWR_MGMT_1, 0x00);
	sendString("MPU6050 Iniciada", 16); delay_ms(100);
	//Offsets
	i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_XA_OFFS_H, 	-257);
	i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_YA_OFFS_H,	 -1477);
	i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_ZA_OFFS_H,	 	2176);
	i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_XG_OFFS_USRH, 	59);
	i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_YG_OFFS_USRH, 	15);
  i2c_mpu_writeWord(MPU_endereco, MPU6050_RA_ZG_OFFS_USRH, 	-9);
	sendString("Offsets Setados", 15); delay_ms(100);
	//Set I2C ByPass na mpu
	i2c_mpu_writeBit(MPU_endereco, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1);
	sendString("ByPass Setado", 13); delay_ms(100);
	//Starting HMC5883
	i2c_mpu_writeByte(HMC_endereco, 0x02, 0x00);
	sendString("HMC5883 Iniciado", 16); delay_ms(100);
}


void DataAcq(){
		i2c_mpu_readBytes(MPU_endereco, MPU6050_RA_ACCEL_XOUT_H, 14, mpu6050_buffer);
		hal_w2_write(HMC_endereco,&hmc_start_byte,1);hal_w2_read(HMC_endereco,hmc_buffer,6);
}


void send_dada_read(){
		tx_buf[ 0] = mpu6050_buffer[0];
		tx_buf[ 1] = mpu6050_buffer[1];
		tx_buf[ 2] = mpu6050_buffer[2];
		tx_buf[ 3] = mpu6050_buffer[3];
		tx_buf[ 4] = mpu6050_buffer[4];
		tx_buf[ 5] = mpu6050_buffer[5];
		
		tx_buf[ 6] = mpu6050_buffer[8];
		tx_buf[ 7] = mpu6050_buffer[9];
		tx_buf[ 8] = mpu6050_buffer[10];
		tx_buf[ 9] = mpu6050_buffer[11];
		tx_buf[10] = mpu6050_buffer[12];
		tx_buf[11] = mpu6050_buffer[13];

		tx_buf[12] = hmc_buffer[0];
		tx_buf[13] = hmc_buffer[1];
		tx_buf[14] = hmc_buffer[4];
		tx_buf[15] = hmc_buffer[5];
		tx_buf[16] = hmc_buffer[2];
		tx_buf[17] = hmc_buffer[3];
		
		tx_buf[18] = mpu6050_buffer[6];
		tx_buf[19] = mpu6050_buffer[7];
		
		TX_Mode_NOACK(20);
		RX_Mode();
}

void sendString(const char* msg, uint8_t msglen){
	uint8_t i = 0;
	for(i = 0; i < msglen; i++){
		tx_buf[i] = msg[i];
	}
	TX_Mode_NOACK(msglen);
	RX_Mode();
}
//interrupção do I2C - ?
void I2C_IRQ (void) interrupt INTERRUPT_SERIAL {
    I2C_IRQ_handler();
}
