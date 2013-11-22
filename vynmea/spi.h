#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f10x_conf.h"
#include "nmea2000.h"
#include "nmea0183.h"
 
void spi_create(SPI_TypeDef * SPIx, GPIO_TypeDef * CS_GPIOx, uint16_t CS_GPIO_Pin);
void spi_handleSPI1Interrupt(void);

void spi_writeNmea0183(struct gps_packet_t *lexer);
void spi_writeNmea2000(nmea2000_packet * packet);

#endif // _SPI_H_
