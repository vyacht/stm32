/* Martin Thomas 4/2009, 3/2010 */
#include "stm32f10x.h"
#include "comm.h"

int comm_test(USART_TypeDef* USARTx)
{
	return ( USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET ) ? 0 : 1;
}

char comm_get(USART_TypeDef* USARTx)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET) { ; }
	return (char)USART_ReceiveData(USARTx);
}

void comm_put(USART_TypeDef* USARTx, char d)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET) { ; }
	USART_SendData(USARTx, (uint16_t)d);
}

void comm_puts(USART_TypeDef* USARTx, const char* s)
{
	char c;
	while ( ( c = *s++) != '\0' ) {
	  comm_put(USARTx, c);
	}
}

void comm_init (void)
{
	// already done in main.c
}


