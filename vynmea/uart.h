#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f10x.h"
#include "fifo.h"

void UART_create(USART_TypeDef* USARTx);
void UART_handleUARTInterrupt(USART_TypeDef* USARTx, fifo_t * fifo);
extern uint8_t UART_printf(USART_TypeDef* USARTx, const char *format, ...); 

#endif
