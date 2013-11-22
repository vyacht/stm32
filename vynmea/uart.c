#include <stdint.h>
#include "stm32f10x.h"

#include <stdarg.h>

#include "comm.h"
#include "fifo.h"

#define DATA_BUF_LEN   80       // Used in uart_mini_printf
#define UART_RX_BUFFER_SIZE 255
#define UART_TX_BUFFER_SIZE 255

void UART_create(USART_TypeDef* USARTx) 
{
}

void UART_handleUARTInterrupt(USART_TypeDef* USARTx, fifo_t * fifo)
{
  if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
  {
     uint8_t recdata;

     // Clear the USART1 Receive interrupt 
     USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
     recdata = USART_ReceiveData(USARTx)&0xFF;
     fifo_put(fifo, recdata);
  }
  if(USART_GetITStatus(USARTx, USART_IT_TXE) != RESET)
  {
    unsigned char temptail;
    USART_ClearITPendingBit(USARTx, USART_IT_TXE);
    USART_SendData(USARTx, 'x');
  }
}

void UART_Transmit(USART_TypeDef* USARTx, unsigned char Data)
{
  //  comm_put(USARTx, Data);
  ITM_SendChar(Data);
}

void UART_TransmitString(USART_TypeDef* USARTx, uint8_t *s)
{
  char c;
  while ( ( c = *s++) != '\0' ) {
    UART_Transmit(USARTx, c);
  }
}

uint8_t UART_printf(USART_TypeDef* USARTx, const char *format, ...) {

    va_list arg_ptr;
    uint8_t      *p,*sval;
    uint8_t      u8_temp, n_sign, data_idx, min_size;
    uint8_t      data_buf[DATA_BUF_LEN];
    int8_t      long_flag, alt_p_c;
    int8_t      s8_val;
    int16_t     s16_val;
    int32_t     s32_val;
    uint16_t     u16_val;
    uint32_t     u32_val;

    long_flag = 0;
    alt_p_c = 0;
    min_size = DATA_BUF_LEN-1;

    va_start(arg_ptr, format);   // make arg_ptr point to the first unnamed arg
    for (p = (uint8_t *) format; *p; p++)
    {
        if ((*p == '%') || (alt_p_c == 1))
        {
            p++;
        }
        else
        {
	  UART_Transmit(USARTx, *p);
            alt_p_c = 0;
            long_flag = 0;
            continue;   // "switch (*p)" section skipped
        }

        switch (*p)
        {
            case 'c':
                if (long_flag == 1)      // ERROR: 'l' before any 'c'
                {
		  UART_Transmit(USARTx, 'l');
		  UART_Transmit(USARTx, 'c');
                }
                else
                {
                    s8_val = (int8_t)(va_arg(arg_ptr, int));    // s8_val = (int8_t)(va_arg(arg_ptr, int16_t));
                    UART_Transmit(USARTx, (uint8_t)(s8_val));
                }
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break; // case 'c'
                
            case 's':
                if (long_flag == 1)      // ERROR: 'l' before any 's'
                {
		  UART_Transmit(USARTx, 'l');
		  UART_Transmit(USARTx, 's');
                }
                else
                {
                    for (sval = va_arg(arg_ptr, uint8_t *); *sval; sval++)
                    {
		      UART_Transmit(USARTx, *sval);
                    }
                }
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break;  // case 's'
                
            case 'l':  // It is not the number "ONE" but the lower case of "L" character
                if (long_flag == 1)      // ERROR: two consecutive 'l'
                {
		  UART_Transmit(USARTx, 'l');
                    alt_p_c = 0;
                    long_flag = 0;
                }
                else
                {
                    alt_p_c = 1;
                    long_flag = 1;
                }
                p--;
                break;  // case 'l'
                
            case 'd':
                n_sign  = 0;               
                for(data_idx = 0; data_idx < (DATA_BUF_LEN-1); data_idx++)
                {
                    data_buf[data_idx] = '0';
                }
                data_buf[DATA_BUF_LEN-1] = 0;
                data_idx = DATA_BUF_LEN - 2;
                if (long_flag)  // 32-bit
                {
                    s32_val = va_arg(arg_ptr, int32_t);
                    if (s32_val < 0)
                    {
                        n_sign = 1;
                        s32_val  = -s32_val;
                    }
                    while (1)
                    {
                        data_buf[data_idx] = s32_val % 10 + '0';
                        s32_val /= 10;
                        data_idx--;
						if (s32_val==0) break;
                   }
                }
                else  // 16-bit
                {
                    s16_val = (int16_t)(va_arg(arg_ptr, int)); // s16_val = va_arg(arg_ptr, int16_t);
                    if (s16_val < 0)
                    {
                        n_sign = 1;
                        s16_val  = -s16_val;
                    }
                    while (1)
                    {
                        data_buf[data_idx] = s16_val % 10 + '0';
                        s16_val /= 10;
                        data_idx--;
						if (s16_val==0) break;
                    }
                }
                if (n_sign) { UART_Transmit(USARTx, '-'); }
                data_idx++;
                if (min_size < data_idx)
                {
                    data_idx = min_size;
                }
                UART_TransmitString(USARTx, data_buf + data_idx);
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break;  // case 'd'
                
            case 'u':
                for(data_idx = 0; data_idx < (DATA_BUF_LEN-1); data_idx++)
                {
                    data_buf[data_idx] = '0';
                }
                data_buf[DATA_BUF_LEN-1] = 0;
                data_idx = DATA_BUF_LEN - 2;
                if (long_flag)  // 32-bit
                {
                    u32_val = va_arg(arg_ptr, uint32_t);
                    while (1)
                    {
                        data_buf[data_idx] = u32_val % 10 + '0';
                        u32_val /= 10;
                        data_idx--;
						if (u32_val==0) break;
                    }
                }
                else  // 16-bit
                {
                    u16_val = (uint16_t)(va_arg(arg_ptr, int)); // u16_val = va_arg(arg_ptr, uint16_t);
                    while (1)
                    {
                        data_buf[data_idx] = u16_val % 10 + '0';
                        data_idx--;
                        u16_val /= 10;
						if (u16_val==0) break;
                    }
                }
                data_idx++;
                if (min_size < data_idx)
                {
                    data_idx = min_size;
                }
                UART_TransmitString (USARTx, data_buf + data_idx);
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break;  // case 'u':
                
            case 'x':
            case 'X':
                for(data_idx = 0; data_idx < (DATA_BUF_LEN-1); data_idx++)
                {
                    data_buf[data_idx] = '0';
                }
                data_buf[DATA_BUF_LEN-1] = 0;
                data_idx = DATA_BUF_LEN - 2;
                if (long_flag)  // 32-bit
                { 
                    u32_val = va_arg(arg_ptr, uint32_t);
                    while (u32_val)
                    {
                        u8_temp = (uint8_t)(u32_val & 0x0F);
                        data_buf[data_idx] = (u8_temp < 10)? u8_temp+'0':u8_temp-10+(*p=='x'?'a':'A');
                        u32_val >>= 4;
                        data_idx--;
                    }
                }
                else  // 16-bit
                {
                    u16_val = (uint16_t)(va_arg(arg_ptr, int)); // u16_val = va_arg(arg_ptr, uint16_t);
                    while (u16_val)
                    {
                        u8_temp = (uint8_t)(u16_val & 0x0F);
                        data_buf[data_idx] = (u8_temp < 10)? u8_temp+'0':u8_temp-10+(*p=='x'?'a':'A');
                        u16_val >>= 4;
                        data_idx--;
                    }
                }
                data_idx++;
                if (min_size < data_idx)
                {
                    data_idx = min_size;
                }
                UART_TransmitString (USARTx, data_buf + data_idx);
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break;  // case 'x' & 'X'
                
            case '0':   // Max allowed "min_size" 2 decimal digit, truncated to DATA_BUF_LEN-1.
                min_size = DATA_BUF_LEN-1;
                if (long_flag == 1)      // ERROR: 'l' before '0'
                {
		  UART_Transmit(USARTx, 'l');
		  UART_Transmit(USARTx, '0');
                    // Clean up
                    alt_p_c = 0;
                    long_flag = 0;
                    break;
                }
                u8_temp = *++p;
                if ((u8_temp >='0') && (u8_temp <='9'))
                {
                    min_size = u8_temp & 0x0F;
                    u8_temp = *++p;
                    if ((u8_temp >='0') && (u8_temp <='9'))
                    {
                        min_size <<= 4;
                        min_size |= (u8_temp & 0x0F);
                        p++;
                    }
                    min_size = ((min_size & 0x0F) + ((min_size >> 4) *10));  // Decimal to hexa
                    if (min_size > (DATA_BUF_LEN-1))
                    {
                        min_size = (DATA_BUF_LEN-1);
                    }  // Truncation
                    min_size = DATA_BUF_LEN-1 - min_size;  // "min_size" formatted as "data_ix"
                }
                else      // ERROR: any "char" after '0'
                {
		  UART_Transmit(USARTx, '0');
		  UART_Transmit(USARTx, *p);
                    // Clean up
                    alt_p_c = 0;
                    long_flag = 0;
                    break;
                }
                p-=2;
                alt_p_c = 1;
                // Clean up
                long_flag = 0;
                break;  // case '0'
                
            default:
                if (long_flag == 1)
                {
		  UART_Transmit(USARTx, 'l');
                }
                UART_Transmit(USARTx, *p);
                // Clean up
                min_size = DATA_BUF_LEN-1;
                alt_p_c = 0;
                long_flag = 0;
                break;  // default
                
        }   // switch (*p ...
        
    }   // for (p = ...
    
    va_end(arg_ptr);
    return 0;
}

