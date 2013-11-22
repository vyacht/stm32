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

void spi_dataUnsignal(void);
 
void spi_create(SPI_TypeDef * SPIx, GPIO_TypeDef * CS_GPIOx, uint16_t CS_GPIO_Pin_x){
  SPI_Module = SPIx;
  CS_GPIO = CS_GPIOx;
  CS_GPIO_Pin = CS_GPIO_Pin_x;

  fifo_init(&spiFifo);

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

  spi_disableTxInterrupt();
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
      spi_enableTxInterrupt();
      
      if(rxCmd & HEADER) {

        spiTxCounter = 0; // reset tx read counter

	spiMaxOut = spiIn;

	//        if(spiOldOut + spiLength != spiFifo.out) {
	//  ITM_SendChar('!');
	//  ITM_SendChar('\n');
	  /*	  UART_printf(0, "! oo = %lu, l= %lu, o= %lu, i= %lu\n", 
		      spiOldOut, spiLength, spiFifo.out, spiMaxOut);
	  */
	//}

        spiLength = spiMaxOut - spiFifo.out;
        //spiOldOut = spiFifo.out;

      }
    }
  } 

  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET) {

    // TODO: timeout to ensure that we do not hang in an TX loop forever 
    // if client doesn't complete its read (which is our transfer)

    if(rxCmd & READ) {

      if(rxCmd & DATA) {

	if(spiFifo.out < spiMaxOut) {

	  fifo_get(&spiFifo, &c);
	  SPI_I2S_SendData(SPI1, c);

	} else {

	  spi_disableTxInterrupt();
	  rxCmd = 0x00;

	}
      }

      if(rxCmd & HEADER) {

	if(spiTxCounter < 1) {
	  SPI_I2S_SendData(SPI1, (uint16_t)spiLength);
	  spiTxCounter++;

	} else {

	  /* disable interrupt even after header read
	   *
	   * if we get an accidental zero length read w/o data, "client" may exit
	   * leaving us in a state with enabled interrupt which loops and then starvs
	   * potentially all other interrupts
	   */
	  spiTxCounter++;
	  spi_disableTxInterrupt();
	  spi_dataUnsignal();
	  rxCmd = 0x00;

	}

      } 
    }
  }
}




