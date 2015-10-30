#include "nrf24l01.h"

u8 mode = 0;
uint8_t  TX_ADDRESS[TX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	
uint8_t  RX_ADDRESS[RX_ADDR_LEN]= {0xE1,0xE1,0xE1,0xE1,0xE1};	

uint8_t NRF_Write_Reg(uint8_t reg, uint8_t data)
{
	uint8_t status;
	NRF_CS(0);					  
	status = SPI_RW(reg);  			
	SPI_RW(data);		  		
	NRF_CS(1);					
  	return 	status;
}

uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t date;
	NRF_CS(0);					
	SPI_RW(reg);		
	date = SPI_RW(0xff);	 
	NRF_CS(1);					
    	return date;
}

uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t i;
	uint8_t status;
	NRF_CS(0);				    
	status = SPI_RW(reg);	
	for(i=0; i<len; i++)
	{
		SPI_RW(pBuf[i]);		
	}
	NRF_CS(1);						
    return status;	
}

uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t i;
	uint8_t status;
	NRF_CS(0);						
	status = SPI_RW(reg);	
	for(i=0; i<len; i++)
	{
		pBuf[i] = SPI_RW(0xff); 
	}
	NRF_CS(1);						
    return status;
}

void NRF_SendPacket(uint8_t * tx_buf, uint8_t len)
{	
	NRF_CE(0);		
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADDR_LEN); // װ�ؽ��ն˵�ַ
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // װ������	
	NRF_CE(1);		 
}
void NRF_BackPacket(uint8_t * tx_buf, uint8_t len)
{	
	NRF_CE(0);		 
	NRF_Write_Buf(0xa8, tx_buf, len); 			 // װ������
	NRF_CE(1);		
}
u8 Nrf24l01_Check(void)
{ 
	u8 buf[5]; 
	u8 i; 
	/*д��5���ֽڵĵ�ַ. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*����д��ĵ�ַ */ 
	NRF_Read_Buf(TX_ADDR,buf,5); 
	
	for(i=0;i<5;i++) 
	{ 
		if(buf[i]!=TX_ADDRESS[i]) 
		    break; 
	} 
	if(i==5)
		return 0 ; 
	else
		return 1 ; 
}

void Nrf24l01_Init(void)
{
	NRF_CE(0);
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADDR_LEN);		//дRX�ڵ��ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADDR_LEN); 		//дTX�ڵ��ַ  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);														//����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 											//����TX�������,0db����,2Mbps,���������濪��
}

void NRF24l01_SetRxMode(void)
{
	NRF_Write_Reg(FLUSH_TX,0xff);
	NRF_Write_Reg(FLUSH_RX,0xff);
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 
	
	SPI_RW(0x50);
	SPI_RW(0x73);
	NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	NRF_CE(1);
	mode=2;
}

void NRF24l01_SetTxMode(void)
{
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		
	NRF_Write_Reg(FLUSH_TX,0xff);
	NRF_Write_Reg(FLUSH_RX,0xff);
	
	SPI_RW(0x50);
	SPI_RW(0x73);
	NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	NRF_CE(1);
	mode=1;
}

u8 NRF24l01_Recv(u8 *buf)	
{
	u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	u8 rx_len = 0;

	if(sta & (1<<RX_DR)) 
	{
		rx_len = NRF_Read_Reg(R_RX_PL_WID);	
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,buf,rx_len);
		}
		else 
		{
			NRF_Write_Reg(FLUSH_RX,0xff);
		}
	}
	if(sta & (1<<MAX_RT))
	{		
		if(sta & 0x01)	
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
	return rx_len;
}

void NRF_SendData(u8 *buf)
{
	if(mode==1)
		NRF_SendPacket(buf,32);
	else if(mode==2)
		NRF_BackPacket(buf,32);
}

////////////////////////////////////////////////////////////////////////////////
