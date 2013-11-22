/*******************************************************************************
* File Name          : main.c
* Author             : Martin Thomas, main-skeleton based on code from the
*                      STMicroelectronics MCD Application Team
* Version            : see VERSION_STRING below
* Date               : see VERSION_STRING below
* Description        : Main program body for the SD-Card tests
********************************************************************************
* License: 3BSD
*******************************************************************************/

#define VERSION_STRING "V1.2.1 7/2010"

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_dma.h"
#include "main.h"
#include "term_io.h"
#include "comm.h"
#include "fifo.h"
#include "uart.h"
#include "spi.h"
#include "rtc.h"
#include "bootload.h"

#include "queue.h"
#include "nmea2000.h"
#include "nmea0183.h"

/* Private variables ---------------------------------------------------------*/
uint8_t     CAN_RxRdy; 
Queue queue;
fifo_t uart_fifo[2];
fifo_t uart3_fifo;

/* Private function prototypes -----------------------------------------------*/
void Periph_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void CAN_Configuration(void);
void SPI_Configuration(void);
void UART_Configuration(void);

void stdio_tests(void);
void dcc_tests(void);

/* Public functions -- -------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main_systick_action
* Description    : operations to be done every 1ms
* Input          : None
* Output         : None
* Return         : None
*  overrides weak SysTick_Handler in startup_stm32*.c
*  When a RAMFUNC every function called from inside the ISR must be
*  reachable. This can be achieved by using compiler-option -mlong-calls
*  which globally enables long-calls. Here this option has not been used
*  instead the unreachable functions GPIO_Set/ResetBits have been replaced
*  by direct register-writes and disk_timerproc has also been attributed
*  as RAMFUNC to be reachable.
*******************************************************************************/
RAMFUNC void SysTick_Handler(void)
{
}

void toggleLED() {

	static uint8_t flip=0;

		/* alive sign */
		if ( flip ) {
			// GPIO_SetBits(GPIOC, GPIO_Pin_12 );
			// GPIOC->BSRR = 0x00001000;
		} else {
			// GPIO_ResetBits(GPIOC, GPIO_Pin_12 );
			// GPIOC->BRR = 0x00001000;
		}
		flip = !flip;
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
nmea2000_packet packet;
extern uint8_t spiDataBuffer[255];

int main(void)
{
	/* System Clocks Configuration */
	Periph_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();

        CAN_Configuration();

        spi_configuration();
	
        UART_Configuration();

	/* Turn on/off LED(s) */
	// GPIO_SetBits(GPIO_LED, GPIO_Pin_LED1 /*| GPIO_Pin_LED4*/);

	/* Setup SysTick Timer for 1 millisecond interrupts, 
	   also enables Systick and Systick-Interrupt */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		// Capture error 
		while (1);
	}

        fifo_init(&uart_fifo[0]);
        fifo_init(&uart_fifo[1]);
        fifo_init(&uart3_fifo);

	// Enable the USARTx
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
       	USART_Cmd(USART3, ENABLE);


        spi_create(SPI1, GPIOA, GPIO_Pin_8);
	
    rtc_init();

    __enable_irq (); 
    
    // enable can interrupts
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    //    CAN_ITConfig(CAN1, CAN_IT_FF0,  ENABLE);
//    CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE); 
  
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
    // SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    // USART_ITConfig(USART3, USART_IT_TXE, ENABLE);    

    CanRxMsg RxMsg;
    packet.packet_count = 0;
    packet.packet_transfer_count = 0;
    packet.frame_count = 0;
    uint8_t c;
    uint8_t q;

    struct gps_packet_t lexer[2];
    packet_init(&lexer[0]); lexer[0].origin = 0;
    packet_init(&lexer[1]); lexer[1].origin = 1; 

    // super small state machine for finding btld key word
    static uint8_t btldstate[] = "xbtldx";

    uint8_t btldcnt= 0;

    while (1) {

       //  printITMState();
       if(queue_pop(&queue, &RxMsg)) {

          nmea2000_parsemsg(&RxMsg, &packet);

          if(packet.state == complete) {
            packet.packet_transfer_count++;
            spi_writeNmea2000(&packet);
          }
        }

       for (q = 0; q < 2; q++) {
        if(!fifo_is_empty(&uart_fifo[q])) {
           fifo_get(&uart_fifo[q], &c);
	   //TODO: check length of buffer
           lexer[q].inbuffer[lexer[q].inbuflen] = c;
           lexer[q].inbuffer[lexer[q].inbuflen+1] = '\0';
           lexer[q].inbuflen++;
           packet_parse(&lexer[q]);

	   /*
	   UART_printf(0, "uart= %d, c = %c @ len = %d, fifo.in= %d, fifo.out= %d \n", 
		       q, c,  
		       lexer[q].inbuflen,
		       uart_fifo[q].in, uart_fifo[q].out);
	   */
           // take copied accepted package and report to SPI
	}
       }

        if(!fifo_is_empty(&uart3_fifo)) {
           fifo_get(&uart3_fifo, &c);
           if(c == btldstate[btldcnt + 1])
             btldcnt++;
           if(btldcnt == 4) {// lifted into state 'd' 
             // bootload();
                *((unsigned long *)0x20004FF0) = 0xDEADBEEF; // 64KBSTM32F103
		NVIC_SystemReset();
           }
        }
    }
}

void UART_Config(USART_TypeDef* USARTx, uint32_t speed) {

  /* USART1 and USART2 configuration ------------------------------------------------------*/
  /* USART and USART2 configured as follow:
     - BaudRate = <speed> baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
  */
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = speed;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USARTx, &USART_InitStructure);

}

void UART_Configuration() {

  UART_Config(USART1, 38400);
  UART_Config(USART2, 38400);

  UART_Config(USART3, 115200);

}

void CAN_Configuration() {

   CAN_InitTypeDef CAN_InitStructure;

   CAN_DeInit(CAN1);

   /* Configure CAN1 **************************************************/  
   /* Struct init*/
   CAN_StructInit(&CAN_InitStructure);
   CAN_InitStructure.CAN_TTCM = DISABLE;
   CAN_InitStructure.CAN_ABOM = DISABLE;
   CAN_InitStructure.CAN_AWUM = DISABLE;
   CAN_InitStructure.CAN_NART = DISABLE;
   CAN_InitStructure.CAN_RFLM = DISABLE;
   CAN_InitStructure.CAN_TXFP = ENABLE;
   CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
   CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
   CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
   CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
   CAN_InitStructure.CAN_Prescaler = 16; //should result in ~250 kB

   /*Initializes the CAN1 */
   CAN_Init(CAN1,&CAN_InitStructure);

   /* CAN filter init */
   CAN_FilterInitTypeDef CAN_FilterInitStructure;
   CAN_FilterInitStructure.CAN_FilterNumber=0;
   CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
   CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
   CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
   CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
   CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
   CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
   CAN_FilterInit(&CAN_FilterInitStructure);

}

void USB_LP_CAN1_RX0_IRQHandler(void) {
  CanRxMsg RxMessage;
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  queue_push(&queue, &RxMessage);
}

void SPI1_IRQHandler(void){
  spi_handleSPI1Interrupt();
}

void USART1_IRQHandler(void)
{
  UART_handleUARTInterrupt(USART1, &uart_fifo[0]);
}

void USART2_IRQHandler(void)
{
  UART_handleUARTInterrupt(USART2, &uart_fifo[1]);
}

void USART3_IRQHandler(void)
{
  UART_handleUARTInterrupt(USART3, &uart3_fifo);
}

void DMA1_Channel3_IRQHandler(void){
  spi_handleDMA1Ch3Interrupt();
  DMA_ClearFlag(DMA1_FLAG_TC3);
}

/*******************************************************************************
* Function Name  : PeriphConfiguration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Periph_Configuration(void)
{
  /* Enable USART1, GPIOA, GPIOD and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); // AFIO and GPIOA needed as well

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable GPIO_LED clock */
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);

  /* DMA1 clock enable */
  // RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
//
#if defined(USE_STM3210B_EVAL) || defined(USE_EK_STM32F)
  /* Enable the USART2 Pins Software Remapping */
//  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif

  // Configure USART1 TX (PA.09) as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 RX (PA.10) as input floating 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   // Remap CAN1, probably only needed for pins other than 11,12
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);

  // Configure CAN1 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure CAN1 TX pin 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
 
  // pin 6 MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // pin 7 MOSI, pin 5 CLK
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Data signal line
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // GPIO_WriteBit(GPIOA, GPIO_Pin_8, SET);

  // Configure USART1 TX as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 RX as input floating 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART2 TX as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART2 RX as input floating 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART3 TX as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART3 RX as input floating 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure GPIO for LEDs as Output push-p GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_SET);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#ifdef VECT_TAB_RAM
/* vector-offset (TBLOFF) from bottom of SRAM. defined in linker script */
extern uint32_t _isr_vectorsram_offs;
void NVIC_Configuration(void)
{
  /* Set the Vector Table base location at 0x20000000+_isr_vectorsram_offs */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, (uint32_t)&_isr_vectorsram_offs);
}
#error // we do not implement this
#else
extern uint32_t _isr_vectorsflash_offs;
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef nvic;

  /* Set the Vector Table base location at 0x08000000+_isr_vectorsflash_offs */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, (uint32_t)&_isr_vectorsflash_offs);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  nvic.NVIC_IRQChannel = SPI1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // CAN1
  nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // Enable USART1 interrupt
  nvic.NVIC_IRQChannel = USART1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // Enable USART2 interrupt0
  nvic.NVIC_IRQChannel = USART2_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // Enable USART3 interrupt
  nvic.NVIC_IRQChannel = USART3_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  nvic.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
}
#endif /* VECT_TAB_RAM */


/*******************************************************************************
* Function Name  : assert_failed
* Description    : called on failed assertion if compiled with USE_FULL_ASSERT
* Input          : pointers to char-arrays/strings: filename, function-name,
*                  line in file
* Output         : via xprintf
* Return         : None
*******************************************************************************/
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(const uint8_t* file, const uint8_t* func, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  UART_printf(0, "Wrong parameters value: file %s on line %d in function %s\r\n", file, line, func);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
