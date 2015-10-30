/*
版 本 号：v1.8
创 建 者：纸飞机工作室
修 改 者：
功能描述：纸飞机四轴程序，勿用于商业用途

 */
#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"
#include "Nrf24l01.h"
#include "IOI2C.h"
#include "MPU6050.h"
#include "spi.h"
#include "func.h"
#include "imu.h"
#include "USART.h"
#include "eeprom.h"
#include "motor.h"
#include "adc_dma.h"
#include "filter.h"
#include "controller.h"

#define		RESEND	3

static volatile ErrorStatus HSEStartUpStatus = SUCCESS;

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		 //LED1 LED2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;		 //LED1 LED2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

}


//系统中断管理
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	  /* Enable the USART1 Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
		
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
						   |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1, ENABLE );
   	
}


int main(void)
{  	
	u8 i;
	u8 RecvBuf[32];
	u8 SendBuf[32];
	u8 offline = 0;
	u8 recv_flag = 0;
	u32 pd2ms=0,pd20ams=0,pd20bms=0,pd100ms=0;

	SystemInit();
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	
	USART1_Configuration();
	dbgPrintf(" Init Ticktack !\r\n");
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	
	for(i=0;i<2;i++)
	{
		OP_LED1;OP_LED2;OP_LED3;OP_LED4;
		delay_ms(500);
		OP_LED1;OP_LED2;OP_LED3;OP_LED4;
		delay_ms(500);
	}
	FilterInit();
	controllerInit();

	dbgPrintf(" Init eeprom!\r\n");
	FLASH_Unlock();	
	EE_Init();
	EE_Read_ACC_GYRO_Offset();
	/* 如果PID丢失或者错误，将下面两行注释去掉，重新编译烧写，运行一遍，可将PID还原，然后重新注释，再烧写一遍 */
	//EE_Write_PID();
	//EE_Write_Rate_PID();
	EE_Read_PID();
	EE_Read_Rate_PID();

	dbgPrintf(" Init adc!\r\n");
	ADC_DMA_Init();

	dbgPrintf(" Init NRF24L01 !\r\n");
	SPI_NRF_Init();	
	Nrf24l01_Init();
	NRF24l01_SetRxMode();
	
	while(Nrf24l01_Check())	
	{
		dbgPrintf("NRF24L01 Fault !\r\n");
		delay_ms(500);
	}
	dbgPrintf("NRF24L01 Is Detected !\r\n");

	dbgPrintf("Init MPU6050...\r\n");
	IIC_Init();
	MPU6050_initialize();


	dbgPrintf("Init Motor...\r\n");
	Motor_Init();
	Motor_SetPwm(0,0,0,0);


	pd20bms = TIMIRQCNT + 10*ITS_PER_MS;
	while(1)
	{
		if(TIMIRQCNT>pd2ms + 2*ITS_PER_MS-1)	// every 4ms
		{
			GetEulerAngle();
			if(lock_flag==UNLOCK) 
				AttitudeToMotors(angle.y,angle.x,angle.z);
			else
			{
				MOTOR1=0;	 			
				MOTOR2=0;				
				MOTOR3=0;				
				MOTOR4=0;				
			}
			Motor_SetPwm(MOTOR1,MOTOR2,MOTOR3,MOTOR4);
			pd2ms = TIMIRQCNT;
		}
		
		if(TIMIRQCNT>pd20ams + 20*ITS_PER_MS-1)	// every 20ms
		{
				if(NRF24l01_Recv(RecvBuf)>10)
				{
						if((RecvBuf[RecvBuf[2]+3]==CheckSum(RecvBuf, RecvBuf[2]+3))&&(RecvBuf[0]==0xAA))
						{
								if(RecvBuf[1]!=0xC0)		
									OP_LED1;		
								offline=0;
								switch(RecvBuf[1])
								{
										case 0xC0:  //control
												Getdesireddata(RecvBuf);
												OP_LED2;
												break;
										case 0x10:  //W PID
												SetPID(RecvBuf);
												break;
										case 0x11:  //W Attitude
												SetAccGyroOffset(RecvBuf);
												break;
										case 0x12:  //W Control offset
												break;
										case 0x14:  //W Rate PID
												SetRatePID(RecvBuf);
												break;
										case 0x20:  //R PID
												recv_flag = RESEND;
												GetPID(SendBuf);
												break;
										case 0x21:  //R Attitude
												recv_flag = RESEND;
												GetAccGyroOffset(SendBuf);
												break;
										case 0x22:  //R Control offset
												break;
										case 0x24:  //R Rate PID
												recv_flag = RESEND;
												GetRatePID(SendBuf);
												break;
										case 0x40:	//校准姿态
												EnableCalibration();
												break;
										case 0x41:	//校准遥控器零点												
												break;
									 default:
												break;
								}
						}
				}
				pd20ams = TIMIRQCNT;
		}
		if(TIMIRQCNT>pd20bms + 20*ITS_PER_MS-1)	// every 20ms
		{
			if(recv_flag==0)
         		GetState(SendBuf);
      		else
         		recv_flag--;
			NRF_SendData(SendBuf);
			OP_LED3;
			pd20bms = TIMIRQCNT;
		}
		
		if(TIMIRQCNT>pd100ms + 100*ITS_PER_MS-1)	// every 100ms
		{
			if(offline>20)
				lock_flag = LOCK;
			offline++;
//			OP_LED4;
			pd100ms = TIMIRQCNT;
		}
	}
}

