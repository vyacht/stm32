#include <string.h>
#include <stdio.h>

#include "spi.h"
#include "nmea2000.h"
#include "nmea0183.h"
#include "fifo.h"
#include "uart.h"
 
#define RX_BUFFER_SIZE 5

#define READ   0x01
#define WRITE  0x02
#define RESET  0x04

#define HEADER 0x10
#define DATA   0x20
#define STATS  0x40

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02

SPI_TypeDef * SPI_Module;
GPIO_TypeDef * CS_GPIO;
uint16_t CS_GPIO_Pin;
 
volatile uint8_t spiRxCounter;

/* this keeps tracks of bytes actually delivered
 */
volatile uint32_t spiTxCounter;

/* this keeps track of no of elements in the fifo buffer
   unlike the in of the actual fifo implementation
   this in is at packets limits
 */
volatile uint32_t spiIn;

// volatile uint32_t spiOut;

volatile uint32_t spiMaxOut;

volatile uint32_t spiOldOut;

/* used during transmission to keep track of the current length
   of buffer to transfer to the spi counterpart (master)
   will be spiIn - spiOut
 */
volatile uint32_t spiLength;

volatile uint8_t resetPending;


uint32_t resetCounter;
uint32_t packetFastCounter;
uint32_t packetErrorCounter;

/*
   header PGN L
          01234
   PGN will be stored in hi byte first
*/
fifo_t spiFifo;
volatile uint8_t rxCmd;
uint8_t spiTxBuffer[12];

//                       meta      pgn                 pid
/*
uint8_t trxBuffer1[] = {  0x02,  43,0x00,0x01,0xf8,0x06,0x00,0x00,0x00,0xff,
                      0xb0,0xff,0xff,0x70,0x59,0x00,
		      0x13,0x00,0xba,0xe4,0x59,0x2a,0x9b,
		      0x7b,0x07,0x00,0xd2,0x2d,0x21,0x57,
		      0x03,0xd1,0x03,0xff,0xff,0xff,0xff,
		      0xff,0xff,0xff,0x7f,0x10,0xfc,0xff,
		      0xff,0x7f,0xff,0x7f,0xff,0xff,0xff,
			 0x7f,0xff};
*/

volatile uint8_t trxBuffer[]  =  "__abcdefghijklmnopqrstuvwxyz";
volatile uint8_t trxBuffer1[]  =  "__ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/*
volatile uint8_t * trxBuffer[]  = { 
  "__ABCDEFGHIJKLMNOPQRSTUVWXYZ"  
};
*/

uint8_t buffercnt = 0;

void spi_dataUnsignal(void);
 
void spi_configuration() {

  SPI_InitTypeDef SPI_InitStructure;

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_CalculateCRC(SPI1, DISABLE);
 
  
  SPI_Cmd(SPI1, ENABLE);

  // DMA Channel 3 - SPI TX
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_BufferSize = sizeof(trxBuffer);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(trxBuffer[0]);
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
 
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
  //  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

  DMA_Cmd(DMA1_Channel3, ENABLE);

}

void spi_create(SPI_TypeDef * SPIx, GPIO_TypeDef * CS_GPIOx, uint16_t CS_GPIO_Pin_x){
  SPI_Module = SPIx;
  CS_GPIO = CS_GPIOx;
  CS_GPIO_Pin = CS_GPIO_Pin_x;

  fifo_init(&spiFifo);

  trxBuffer[0] = 10;

  spiRxCounter = 0;

  spiTxCounter = 0;

  resetPending = 0;

  //spiOut = 0;
  spiOldOut = 0;
  spiIn = 0;

  resetCounter = 0;
  packetFastCounter = 0;
  packetErrorCounter = 0;
  spi_dataUnsignal();
}
 
void spi_enableTxInterrupt(void){
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}
 
void spi_disableTxInterrupt(void){
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
}
 
void spi_dataSignal(void){
  GPIO_SetBits(GPIOC, GPIO_Pin_12 );
  GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
  //  ITM_SendChar('S'); ITM_SendChar('\n');
}

uint8_t spi_dataSignaled(void){
  // mind the RESET - low is signaled
  return (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET);
}
 
void spi_dataUnsignal(void){
  GPIO_ResetBits(GPIOC, GPIO_Pin_12 );
  GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
  //  ITM_SendChar('U'); ITM_SendChar('\n');
}
 
void spi_writeData(uint32_t val) {

  fifo_put(&spiFifo, val >> 24);    
  fifo_put(&spiFifo, val >> 16);    
  fifo_put(&spiFifo, val >> 8);    
  fifo_put(&spiFifo, val);    

}

void spi_writeBuffer(uint8_t * buf, uint8_t off, volatile uint32_t val) {

  buf[off] = val >> 24;    
  buf[off + 1] = val >> 16;    
  buf[off + 2] = val >> 8;    
  buf[off + 3] = val;    
}

void spi_writeNmea0183(struct gps_packet_t *lexer){

  uint8_t len = (uint32_t)(lexer->inbufptr - lexer->inbuffer);

  if(resetPending) 
    spiFifo.in = spiFifo.out;
  resetPending = 0;

  if((fifo_free(&spiFifo) < len + 2)) {
    UART_printf(0, "NMEA 0183 REJ len = %u, org = %d, spiIn = %lu, spiOut= %lu\n", 
		len,
		lexer->origin,
		spiIn, spiFifo.out);
    // wake-up call if queue is full doesn't do any harm
    if(!spi_dataSignaled()) {
      spi_dataSignal();
    }
    return;
  }


  unsigned long l = spiFifo.in;

  // package type and origin
  fifo_put(&spiFifo, PKG_TYPE_NMEA0183);    

  fifo_put(&spiFifo, (uint8_t)len);         // package len

  fifo_in(&spiFifo, (uint8_t *)lexer->inbuffer, len);    

  spiIn = spiFifo.in; 

  UART_printf(0, "NMEA 0183 ADD len = %u + 2, org = %d, spiIn = %lu, spiOut= %lu\n", 
	      len,
	      lexer->origin,
	      spiIn, spiFifo.out);

  assert_param(l + len + 2 == spiFifo.in);

  spi_dataSignal();
}

void spi_writeNmea2000(nmea2000_packet * packet){

  if(resetPending) 
    spiFifo.in = spiFifo.out;
  resetPending = 0;

  if(fifo_free(&spiFifo) < packet->outbuflen + 8 + 2)  {
    UART_printf(0, "NMEA 2000 REJ #= %lu, pgn = %lu with len= %u + 8 + 2, spiIn = %lu, spiOut= %lu\n", 
	      packet->packet_transfer_count,
	      packet->pgn,
	      packet->outbuflen,
	      spiIn, spiFifo.out);

    // wake-up call if queue is full doesn't do any harm
    if(!spi_dataSignaled()) {
      spi_dataSignal();
    }
    return;
  }

  unsigned long l = spiFifo.in;

  // package type and origin
  fifo_put(&spiFifo, PKG_TYPE_NMEA2000 | (0 << 5));    

  //  fifo_put(&spiFifo, (uint8_t)packet->outbuflen);         // package len
  fifo_put(&spiFifo, (uint8_t)43);         // package len

  fifo_put(&spiFifo, packet->pgn >> 24);    
  fifo_put(&spiFifo, packet->pgn >> 16);    
  fifo_put(&spiFifo, packet->pgn >>  8);    
  fifo_put(&spiFifo, packet->pgn);    

  fifo_put(&spiFifo, packet->packet_transfer_count >> 24);    
  fifo_put(&spiFifo, packet->packet_transfer_count >> 16);    
  fifo_put(&spiFifo, packet->packet_transfer_count >>  8);    
  fifo_put(&spiFifo, packet->packet_transfer_count);    

  fifo_in(&spiFifo, packet->outbuffer, packet->outbuflen);    
  
  spiIn = spiFifo.in; 

  UART_printf(0, "NMEA 2000 ADD #= %lu, pgn = %lu with len= %u + 8, spiIn = %lu, spiOut= %lu\n", 
	      packet->packet_transfer_count,
	      packet->pgn,
	      packet->outbuflen,
	      spiIn, spiFifo.out);

  assert_param(l + packet->outbuflen + 8 + 2 == spiFifo.in);

  spi_dataSignal();
}

void spi_reset() {

  UART_printf(0, "SPI reset\n");

  //  spi_disableTxInterrupt();
  spi_dataUnsignal();

  rxCmd = 0x00;

  spiTxCounter = 0; // reset tx write counter
  spiRxCounter = 0; // reset tx write counter

  assert_param(spiFifo.out <= spiFifo.in);
  assert_param(spiIn <= spiFifo.in);

  // dirty hack to consume all bytes up to next packet start
  spiLength = 0;
  spiFifo.out = spiOldOut = spiIn;


  resetPending = 1;
}

void spi_handleDMA1Ch3Interrupt(void){

  if(DMA_GetFlagStatus(DMA1_FLAG_TC3) != RESET)
    {
      //Clear the interrupt flag
      DMA_ClearFlag(DMA1_FLAG_TC3);
  
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //wait until TXE=1
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET);  //wait until BSY=0
      //while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET); //also wait until the DMA transfer for reception is finished
  
      //Disable the interrupt when the transfer is complete
            DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
      //Disable the DMA transfers
      //SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,DISABLE);
            SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,DISABLE);
  
	    //DMA_Cmd(DMA1_Channel3, DISABLE);
      rxCmd = 0x00;

    }
}

void spi_handleSPI1Interrupt(void){

  uint8_t c = 'x';

  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {

    /* TODO - if READ/WRITE was sent: 
       how to handle if there is second coming before 
       all data was read/written */

    // Receive Buffer Not Empty
    c = SPI_I2S_ReceiveData(SPI1);

    if(c == RESET){

      spi_reset();

    } else if((c & READ) || (c & WRITE)) {

      rxCmd = c;

      if(rxCmd & HEADER) {

        spiTxCounter = 0; // reset tx read counter

        //spiLength = spiMaxOut - spiFifo.out;
	spiLength = sizeof(trxBuffer);

	trxBuffer[0] = spiLength - 2;
	trxBuffer1[0] = spiLength - 2;

	DMA1_Channel3->CNDTR = sizeof(trxBuffer1);
	DMA1_Channel3->CMAR = (uint32_t)&trxBuffer1[0];
        buffercnt++;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	//Configure the tx DMA to interrupt when the transfer is complete
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA1_Channel3, ENABLE);

	ITM_SendChar('h');
	ITM_SendChar('\n');

      } else if(rxCmd & DATA) {

	//	spi_disableTxInterrupt();
      }
    }
  } 

  /*
  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET) {

    // TODO: timeout to ensure that we do not hang in an TX loop forever 
    // if client doesn't complete its read (which is our transfer)

    if(rxCmd & READ) {

      if(rxCmd & DATA) {
	ITM_SendChar(rxCmd); ITM_SendChar('\n');
      }
      assert_param((rxCmd & DATA) == 0);

      if(rxCmd & HEADER) {

	if(spiTxCounter < 1) {
	  SPI_I2S_SendData(SPI1, (uint16_t)spiLength);
	  spi_disableTxInterrupt();
	  spiTxCounter++;

	} else {
  */
	  /* disable interrupt even after header read
	   *
	   * if we get an accidental zero length read w/o data, "client" may exit
	   * leaving us in a state with enabled interrupt which loops and then starvs
	   * potentially all other interrupts
	   */
  /*	  spiTxCounter++;
	  spi_disableTxInterrupt();
	  spi_dataUnsignal();
	  rxCmd = 0x00;

	}

      } 
    }
  }
  */
}




