#ifndef COMM_H_
#define COMM_H_

void comm_init(void);
int  comm_test(USART_TypeDef* USARTx);
void comm_put(USART_TypeDef* USARTx, char);
void comm_puts(USART_TypeDef* USARTx, const char*);
char comm_get(USART_TypeDef* USARTx);

#endif

