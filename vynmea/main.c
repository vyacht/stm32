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
#include "comm.h"
#include "fifo.h"
#include "uart.h"
#include "spi.h"
#include "rtc.h"
#include "bootload.h"

#include "queue.h"
#include "nmea2000.h"
#include "nmea0183.h"
/*
 * POSSIBLE DMA channels
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 */

/* Private variables ---------------------------------------------------------*/
uint8_t     CAN_RxRdy; 
Queue queue;
fifo_t uart3_fifo;

#define BUFFER_SIZE 256
typedef struct __uartFifo {
  uint8_t data[BUFFER_SIZE];
  uint16_t in;    
  uint16_t out;    
  DMA_Channel_TypeDef* DMACh;
} uartFifo_t;

uartFifo_t uartFifo[2];


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

  //  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
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
      // read out from the DMA buffer
      uint16_t dataCounter = DMA_GetCurrDataCounter(uartFifo[q].DMACh);
      uartFifo[q].in = BUFFER_SIZE - dataCounter;

      // if we wrapped, we consume everything to the end of the buffer
      if (uartFifo[q].in < uartFifo[q].out) {
	while (uartFifo[q].out < BUFFER_SIZE) {
	  c = uartFifo[q].data[uartFifo[q].out++];
	  lexer[q].inbuffer[lexer[q].inbuflen] = c;
	  lexer[q].inbuffer[lexer[q].inbuflen+1] = '\0';
	  lexer[q].inbuflen++;
	}
	uartFifo[q].out = 0;
      }

      // consume the beginning of the buffer
      while (uartFifo[q].out < uartFifo[q].in) {
	c = uartFifo[q].data[uartFifo[q].out++];
	lexer[q].inbuffer[lexer[q].inbuflen] = c;
	lexer[q].inbuffer[lexer[q].inbuflen + 1] = '\0';
	lexer[q].inbuflen++;
      }

      packet_parse(&lexer[q]);

    }

    /*
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
    */
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
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USARTx, &USART_InitStructure);

}
uint8_t Buffer[32];
 
void UART_DMA_Configuration(DMA_Channel_TypeDef* DMAy_Channelx, 
			    USART_TypeDef* USARTx,
			    uartFifo_t * fifo) {
  fifo->in = 0;
  fifo->out = 0;
  fifo->DMACh = DMAy_Channelx;

  DMA_InitTypeDef  DMA_InitStructure;
 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // Receive
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(fifo->data);
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMAy_Channelx, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
 
  // Enable DMA Stream Half Transfer and Transfer Complete interrupt 
  // DMA_ITConfig(DMAy_Channelx, DMA_IT_TC, ENABLE);
  // DMA_ITConfig(DMAy_Channelx, DMA_IT_HT, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMAy_Channelx, ENABLE);
}

/*
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

static uint32_t exti_calls = 0;
void EXTI15_10_IRQHandler(void) {
  exti_calls++;
}


void DMA1_Channel6_IRQHandler(void){

  // Test on DMA Stream Transfer Complete interrupt 
  if (DMA_GetITStatus(DMA1_FLAG_TC6))
  {
    // Clear DMA Stream Transfer Complete interrupt pending bit 
    DMA_ClearITPendingBit(DMA1_FLAG_TC6);
  }
 
}
*/

void UART_Configuration() {

  UART_Config(USART1, 38400);
  UART_DMA_Configuration(DMA1_Channel5, USART1, &uartFifo[0]);

  UART_Config(USART2, 38400);
  UART_DMA_Configuration(DMA1_Channel6, USART2, &uartFifo[1]);

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
}

void USART2_IRQHandler(void)
{
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

  /* TIM2 clock enable */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

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

   // Remap CAN1
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);

  // Configure CAN1 RX pin PA11 -> PB8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure CAN1 TX pin PA12 -> PB9
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


  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

  // Configure USART1 TX (PA.09) -> PB6
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART1 RX (PA.10) -> PB7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

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

  // Configure GPIO for LEDs as Output push-p 
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
extern uint32_t _isr_vectorsflash_offs;
extern uint32_t _isr_vectorsram_offs;
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef nvic;

  /* Set the Vector Table base location at 0x08000000+_isr_vectorsflash_offs */
  // NVIC_SetVectorTable(NVIC_VectTab_RAM, (uint32_t)&_isr_vectorsram_offs);
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, (uint32_t)&_isr_vectorsflash_offs);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // SPI
  nvic.NVIC_IRQChannel = SPI1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // DMA for SPI
  nvic.NVIC_IRQChannel = DMA1_Channel3_IRQn;
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

  /*
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
  */

  // Enable USART3 interrupt
  nvic.NVIC_IRQChannel = USART3_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  /*
  //configure the interrupts
  //Connect EXTI Line to UART 1 RX
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);

  // Configure UART EXTI line
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
  nvic.NVIC_IRQChannelSubPriority = 0x07;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);


  // Enable the UART2 RX DMA Interrupt 
  nvic.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  */

  /*
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  // Enable the TIM2 gloabal Interrupt 
  // Time base configuration 
  nvic.NVIC_IRQChannel = TIM2_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  // VL clocks at 24 MHz, change to 72 - 1 for a 72 MHz system
  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // TIM IT enable 
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  // TIM2 enable counter 
  TIM_Cmd(TIM2, ENABLE);
  */
}


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
