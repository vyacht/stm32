/**
  ******************************************************************************
  * @file platform_config.h
  * @author  MCD Application Team, mod. Martin Thomas for Manley EK-STM32F
  * @version  V3.0.0.1
  * @date  04/17/2009
  * @brief  Evaluation board specific configuration file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Un-comment the line corresponding to evaluation board used to run the example */
#if !defined (USE_STM3210B_EVAL) && !defined (USE_STM3210E_EVAL) && \
    !defined (USE_EK_STM32F) && !defined(USE_MINI_STM32)
 //#define USE_STM3210B_EVAL
 //#define USE_STM3210E_EVAL
 //#define USE_EK_STM32F
#error "board not defined"
#endif


/* Define the STM32F10x hardware depending on the used evaluation board */
#if defined(USE_STM3210B_EVAL)
#define GPIOx                     GPIOD
#define RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOD
#define GPIO_TxPin                GPIO_Pin_5
#define GPIO_RxPin                GPIO_Pin_6
#define GPIO_LED                  GPIOC
#define RCC_APB2Periph_GPIO_LED   RCC_APB2Periph_GPIOC
#define GPIO_Pin_LED1             GPIO_Pin_6
#define GPIO_Pin_LED2             GPIO_Pin_7
#define GPIO_Pin_LED3             GPIO_Pin_8
#define GPIO_Pin_LED4             GPIO_Pin_9

#elif defined(USE_STM3210E_EVAL)
#define GPIOx                     GPIOA
#define RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOA
#define GPIO_TxPin                GPIO_Pin_2
#define GPIO_RxPin                GPIO_Pin_3
#define GPIO_LED                  GPIOF
#define RCC_APB2Periph_GPIO_LED   RCC_APB2Periph_GPIOF
#define GPIO_Pin_LED1             GPIO_Pin_6
#define GPIO_Pin_LED2             GPIO_Pin_7
#define GPIO_Pin_LED3             GPIO_Pin_8
#define GPIO_Pin_LED4             GPIO_Pin_9

#elif defined(USE_EK_STM32F)
#define GPIOx                     GPIOD
#define RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOD
#define GPIO_TxPin                GPIO_Pin_5
#define GPIO_RxPin                GPIO_Pin_6
#define GPIO_LED                  GPIOC
#define RCC_APB2Periph_GPIO_LED   RCC_APB2Periph_GPIOC
#define GPIO_Pin_LED1             GPIO_Pin_4
#define GPIO_Pin_LED2             GPIO_Pin_5
#define GPIO_Pin_LED3             GPIO_Pin_6
#define GPIO_Pin_LED4             GPIO_Pin_7

#elif defined(USE_MINI_STM32)
#define GPIOx                     GPIOD
#define RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOD
#define GPIO_TxPin                GPIO_Pin_5
#define GPIO_RxPin                GPIO_Pin_6
#define GPIO_LED                  GPIOA
#define RCC_APB2Periph_GPIO_LED   RCC_APB2Periph_GPIOA
#define GPIO_Pin_LED1             GPIO_Pin_0
#define GPIO_Pin_LED2             GPIO_Pin_1
#define GPIO_Pin_LED3             GPIO_Pin_0
#define GPIO_Pin_LED4             GPIO_Pin_1

#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
