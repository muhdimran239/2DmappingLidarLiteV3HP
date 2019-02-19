#include "stm32f4xx.h" 
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "EXTI_STM32F4xx.h"

#define LIDAR_ADD 0x62

__IO uint32_t usTick=2500;

char str[10];
uint32_t i,j;
int distance =0;

//PA2 TX 
//PA3 RX
void USART_Initialize(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
}

void USART_Puts(USART_TypeDef* USARTx,volatile char *s) {
 while(*s)
 {
  while(!(USARTx ->SR & 0x00000040));
	 USART_SendData(USARTx,*s);
	 *s++;
 }	 
}


//MOTEUR NEMA17 200pas/tour 
// PB9 STEP orange
// PB8 DIR  green
void Moteur_PWM_Initialize(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; 
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
		
		
		GPIO_Init(GPIOB,&GPIO_InitStructure);
}


//red - 5V
//black - gnd
//blue - PB7 (SDA)
//green - PB6 (SCL)
void I2C1_Initialize(void){
	
	I2C_InitTypeDef I2C_init;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE); 	// active l�horloge sur lequel est connect� l'I2C1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	// active l�horloge p�riph�rique du portB
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);
	
	I2C_init.I2C_ClockSpeed=100000;
	I2C_init.I2C_DutyCycle=50;
	I2C_init.I2C_Mode=I2C_Mode_I2C;
	I2C_init.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_init.I2C_Ack=I2C_Ack_Disable;
	I2C_init.I2C_OwnAddress1=0;
	
	I2C_Init(I2C1,&I2C_init);			// configuration des param�tres de transmission
	I2C_Cmd(I2C1,ENABLE);				// active le p�riph�rique I2C1.
	
}


void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	while(I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY)); // attend que l'i2c ne soit plus occup�
	//I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY);
	I2C_GenerateSTART(I2Cx,ENABLE);
	
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT));	// attend la fin de la condidion start
	//I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT);
	I2C_Send7bitAddress(I2Cx,address,direction);
	if(direction == I2C_Direction_Transmitter)
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	// attendre l�acquittement du mode �metteur 
	if(direction == I2C_Direction_Receiver)
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	// attendre l�acquittement du mode r�cepteur
	
}


int get_distance(void){
	int dist=0;
	//Distance first 2 bytes 
	int distanceA=0;
	//Distance last 2 bytes
	int distanceB=0;
	//pooling data
	uint8_t data=0;
	
		// envoi de 0x00
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);	
	I2C_SendData(I2C1,0x00);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	// envoi de 0x01
	I2C_SendData(I2C1,0x01);
	I2C_GenerateSTOP(I2C1, ENABLE);	
	
	// envoi de 0x04
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);
	I2C_SendData(I2C1,0x04);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	// envoi de 0x08
	I2C_SendData(I2C1,0x08);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	// envoi de 0x02 -- config max acq
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);
	I2C_SendData(I2C1,0x02);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	// envoi de 0x1d -- max speed acq
	I2C_SendData(I2C1,0x1d);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	
	//	Pooling sur le bit 0 du registre d'etat (0x01)
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);
	I2C_SendData(I2C1,0x01);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));	
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	I2C_start(I2C1,LIDAR_ADD << 1  ,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));	
	data=I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	while((data&0x01)!=0x00){
		I2C_start(I2C1,LIDAR_ADD << 1  ,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));	
	data=I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);
	}
	
	
	// lecture de la donn�e en 0x10 
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);
	I2C_SendData(I2C1,0x10);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));	// Attends que l�esclave I2C3 ait re�u la donn�e
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	I2C_start(I2C1,LIDAR_ADD << 1  ,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));	// Attends que l�esclave I2C3 ait re�u la donn�e
	distanceA=I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	
	// lecture de la donn�e en 0x0f
	I2C_start(I2C1,LIDAR_ADD <<1 ,I2C_Direction_Transmitter);
	I2C_SendData(I2C1,0x0f);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));	// Attends que l�esclave I2C3 ait re�u la donn�e
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	I2C_start(I2C1,LIDAR_ADD << 1  ,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));	// Attends que l�esclave I2C3 ait re�u la donn�e
	distanceB=I2C_ReceiveData(I2C1);
	I2C_GenerateSTOP(I2C1, ENABLE);

	// concatenation des 2 donnees pour obtenir la distance
	distanceB=distanceB<<8;
	dist=distanceB+distanceA;
		
	return dist;
	
}

//Interruption for motor
void SysTick_Handler(void){ 	
  usTick--;
	GPIO_WriteBit(GPIOB,GPIO_Pin_9,1); 
	if(usTick==0){
		GPIO_WriteBit(GPIOB,GPIO_Pin_9,0);
    usTick=2500;	//2.5 ms 1 pas	
	}
    
}

int main(){
	
	USART_Initialize(); //Initialize USART 
	I2C1_Initialize(); //Initialize I2C1
	Moteur_PWM_Initialize(); //Initialize motor
	SystemCoreClockUpdate();
	//The function initializes the System Timer and its interrupt, and starts the System Tick Timer. 
	//Counter is in free running mode to generate periodic interrupts.
  	SysTick_Config(SystemCoreClock / 1000000); //uint32_t SystemCoreClock = 168000000; 
	GPIO_WriteBit(GPIOB,GPIO_Pin_8,1);//direction
	

	//USART_Puts(USART2,"Program Lidar Start\n");
	while(1){
			
      distance = get_distance(); //Get the mesure from LIDAR
			sprintf(str,"S%d\n",distance);
			USART_Puts(USART2,str); 
	
	}
	
	return 0;
}

