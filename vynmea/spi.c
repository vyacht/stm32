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
#define PKG_TYPE_BAD      0x08 // mark bad packets

SPI_TypeDef * SPI_Module;
GPIO_TypeDef * CS_GPIO;
uint16_t CS_GPIO_Pin;
 
volatile uint8_t spiRxCounter;

/* this keeps tracks of bytes actually delivered
 */
volatile uint32_t spiTxCounter;

volatile uint8_t resetPending;

uint32_t resetCounter;
uint32_t packetFastCounter;
uint32_t packetErrorCounter;

/*
   header PGN L
          01234
   PGN will be stored in hi byte first
*/

volatile uint8_t rxCmd;

#define MAX_SPI_BUFFER_SIZE 256
#define MAX_SPI_FIFO_SIZE 2

typedef struct __spi_buffer {
        uint16_t        len;
        uint8_t         data[MAX_SPI_BUFFER_SIZE];
} spi_buffer_t;


uint8_t spi_fifo_in;
uint8_t spi_fifo_out;
uint8_t spi_fifo_mask;
spi_buffer_t spi_fifo_data[MAX_SPI_FIFO_SIZE];

void spi_fifo_init() {
  spi_fifo_in = spi_fifo_out = 0;
  spi_fifo_mask = MAX_SPI_FIFO_SIZE - 1;
}

int spi_fifo_is_full()
{
  return (spi_fifo_in - spi_fifo_out) > spi_fifo_mask;
}

int spi_fifo_is_empty()
{
  return spi_fifo_in == spi_fifo_out;
}

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
  DMA_InitStructure.DMA_BufferSize = sizeof(spi_fifo_data[0].len);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(spi_fifo_data[0].data);
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
 
  //  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

  //Configure the tx DMA to interrupt when the transfer is complete
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

  DMA_Cmd(DMA1_Channel3, DISABLE);

}

void spi_create(SPI_TypeDef * SPIx, GPIO_TypeDef * CS_GPIOx, uint16_t CS_GPIO_Pin_x){

  SPI_Module = SPIx;
  CS_GPIO = CS_GPIOx;
  CS_GPIO_Pin = CS_GPIO_Pin_x;

  spi_fifo_init();

  spiRxCounter = 0;

  spiTxCounter = 0;

  resetPending = 0;

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
 
void spi_writeNmea0183(struct gps_packet_t *lexer){

  uint8_t len = (uint32_t)(lexer->inbufptr - lexer->inbuffer);

  if(resetPending)  {}
  resetPending = 0;

  if(spi_fifo_is_full()) {
    UART_printf(0, "NMEA 0183 REJ len = %u, org = %d\n", 
		len,
		lexer->origin);
    // wake-up call if queue is full doesn't do any harm
    if(!spi_dataSignaled()) {
      spi_dataSignal();
    }
    return;
  }

  spi_buffer_t * s = &spi_fifo_data[spi_fifo_in & spi_fifo_mask];

  s->data[0] = PKG_TYPE_NMEA0183 | (lexer->origin << 5);    

  if(lexer->type == BAD_PACKET)
    s->data[0] |= PKG_TYPE_BAD;    

  s->data[1] = (uint8_t)len;
  memcpy(s->data + 2, (uint8_t *)lexer->inbuffer, len);

  s->len = len + 2;

  spi_fifo_in++;

  UART_printf(0, "NMEA 0183 ADD len = %u + 2, org = %d\n", 
	      len, lexer->origin);

  spi_dataSignal();
}

void spi_writeNmea2000(nmea2000_packet * packet){

  if(resetPending) {}
  resetPending = 0;

  if(spi_fifo_is_full())  {
    UART_printf(0, "NMEA 2000 REJ #= %lu, pgn = %lu with len= %u + 8 + 2\n", 
	      packet->packet_transfer_count,
	      packet->pgn,
	      packet->outbuflen);

    // wake-up call if queue is full doesn't do any harm
    if(!spi_dataSignaled()) {
      spi_dataSignal();
    }
    return;
  }

  spi_buffer_t * s = &spi_fifo_data[spi_fifo_in & spi_fifo_mask];

  // package type and origin
  s->data[0] = PKG_TYPE_NMEA2000 | (0 << 5);    

  //  fifo_put(&spiFifo, (uint8_t)packet->outbuflen);         // package len
  s->data[1] = (uint8_t)43;         // package len

  s->data[2] = packet->pgn >> 24;    
  s->data[3] = packet->pgn >> 16;    
  s->data[4] = packet->pgn >>  8;    
  s->data[5] = packet->pgn;    

  s->data[6] = packet->packet_transfer_count >> 24;    
  s->data[7] = packet->packet_transfer_count >> 16;    
  s->data[8] = packet->packet_transfer_count >>  8;    
  s->data[9] = packet->packet_transfer_count;    

  memcpy(&(s->data[10]), packet->outbuffer, packet->outbuflen);    
  
  s->len = packet->outbuflen + 2 + 8;
  spi_fifo_in++;

  UART_printf(0, "NMEA 2000 ADD #= %lu, pgn = %lu with len= %u + 8\n", 
	      packet->packet_transfer_count,
	      packet->pgn,
	      packet->outbuflen);

  spi_dataSignal();
}

void spi_reset() {

  UART_printf(0, "SPI reset\n");

  spi_disableTxInterrupt();
  spi_dataUnsignal();

  rxCmd = 0x00;

  spiTxCounter = 0; // reset tx write counter
  spiRxCounter = 0; // reset tx write counter

  resetPending = 1;
}

void spi_handleDMA1Ch3Interrupt(void){

  if(DMA_GetFlagStatus(DMA1_FLAG_TC3) != RESET)
    {
      //Clear the interrupt flag
      DMA_ClearFlag(DMA1_FLAG_TC3);
  
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //wait until TXE=1
      //while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET);  //wait until BSY=0

      //also wait until the DMA transfer for reception is finished
      //while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET); 
  
      DMA_Cmd(DMA1_Channel3, DISABLE);
      spi_dataUnsignal();

      /*
	Transfer is finished. We jump to the next buffer.
       */
      spi_fifo_out++;
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

	// move to other send buffer
        spiTxCounter = 0; // reset tx read counter
	spi_enableTxInterrupt();

      } else if(rxCmd & DATA) {

	DMA1_Channel3->CNDTR = spi_fifo_data[spi_fifo_out && spi_fifo_mask].len;
	DMA1_Channel3->CMAR = (uint32_t)&(spi_fifo_data[spi_fifo_out && spi_fifo_mask].data[0]);

	DMA_Cmd(DMA1_Channel3, ENABLE);

      }
    }
  } 

  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET) {

    // TODO: timeout to ensure that we do not hang in an TX loop forever 
    // if client doesn't complete its read (which is our transfer)

    if(rxCmd & HEADER) {

	if(spiTxCounter < 1) {
	  /* 
	     We are making an active decision here and once
	     that there is data to deliver by sending the 
	     packet len.

	     spi_fifo_out needs to be increased once transfer 
	     stopped.
	  */

	  // buffer not empty
	  if(spi_fifo_in != spi_fifo_out) {
	    SPI_I2S_SendData(SPI1, spi_fifo_data[spi_fifo_out && spi_fifo_mask].len);
	  } else {
	    SPI_I2S_SendData(SPI1, 0);
	  }
	  spi_disableTxInterrupt();
	  spiTxCounter++;

	} 

	/* 
	   else {
	  // disable interrupt even after header read
	   //
	   // if we get an accidental zero length read w/o data, "client" may exit
	   // leaving us in a state with enabled interrupt which loops and then starvs
	   // potentially all other interrupts
	   //
  	  spiTxCounter++;
	  spi_disableTxInterrupt();
	  spi_dataUnsignal();
	  rxCmd = 0x00;

	}
      */

    }
  }
}




