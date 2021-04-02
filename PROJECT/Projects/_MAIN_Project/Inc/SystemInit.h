/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Templates_LL/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEMINIT_H
#define __SYSTEMINIT_H

#ifdef __cplusplus
extern "C" {
#endif


/* ----------------------------------------------------------------------------
 *                           Driver Includes
 * ----------------------------------------------------------------------------*/

		#include "stm32g0xx_ll_rcc.h"
		#include "stm32g0xx_ll_bus.h"
		#include "stm32g0xx_ll_system.h"
		#include "stm32g0xx_ll_exti.h"
		#include "stm32g0xx_ll_cortex.h"
		#include "stm32g0xx_ll_utils.h"
		#include "stm32g0xx_ll_pwr.h"
		#include "stm32g0xx_ll_dma.h"
		#include "stm32g0xx_ll_gpio.h"


		#include "stm32g0xx_ll_adc.h"
		#include "stm32g0xx_ll_comp.h"
		#include "stm32g0xx_ll_crc.h"
		#include "stm32g0xx_ll_dac.h"
		#include "stm32g0xx_ll_i2c.h"
		#include "stm32g0xx_ll_lptim.h"
		#include "stm32g0xx_ll_lpuart.h"
		#include "stm32g0xx_ll_rng.h"
		#include "stm32g0xx_ll_rtc.h"
		#include "stm32g0xx_ll_spi.h"
		#include "stm32g0xx_ll_tim.h"
		#include "stm32g0xx_ll_ucpd.h"
		#include "stm32g0xx_ll_usart.h"
		

/* ----------------------------------------------------------------------------
 *                           Standard Includes
 * ----------------------------------------------------------------------------*/
		//#include "main.h"
		#include "stdint.h"
		#include "stdio.h"
		#include "string.h"
		#include "stdlib.h"
		#include "math.h"
		#include "errno.h"
		#include "app.h"
		

/* ----------------------------------------------------------------------------
 *                           Mereged Standard peripheral library
 * ----------------------------------------------------------------------------*/


		#define _DNumber_0				(uint8_t)0
		#define _DNumber_1				(uint8_t)1
		#define _DNumber_2				(uint8_t)2
		#define _DNumber_3				(uint8_t)3
		#define _DNumber_4				(uint8_t)4
		#define _DNumber_5				(uint8_t)5
		#define _DNumber_6				(uint8_t)6
		#define _DNumber_7				(uint8_t)7
		#define _DNumber_8				(uint8_t)8
		#define _DNumber_9				(uint8_t)9		
	
		#define _DNumber_10				(uint8_t)10
		#define _DNumber_11				(uint8_t)11
		#define _DNumber_12				(uint8_t)12
		#define _DNumber_13				(uint8_t)13
		#define _DNumber_14				(uint8_t)14
		#define _DNumber_15				(uint8_t)15
		#define _DNumber_16				(uint8_t)16
		#define _DNumber_17				(uint8_t)17
		#define _DNumber_18				(uint8_t)18
		#define _DNumber_19				(uint8_t)19		
	
		#define _DNumber_20				(uint8_t)20
		#define _DNumber_21				(uint8_t)21
		#define _DNumber_22				(uint8_t)22
		#define _DNumber_23				(uint8_t)23
		#define _DNumber_24				(uint8_t)24
		#define _DNumber_25				(uint8_t)25
		#define _DNumber_26				(uint8_t)26
		#define _DNumber_27				(uint8_t)27
		#define _DNumber_28				(uint8_t)28
		#define _DNumber_29				(uint8_t)29		
	
		#define _DNumber_30				(uint8_t)30
		#define _DNumber_31				(uint8_t)31
		#define _DNumber_32				(uint8_t)32
		#define _DNumber_33				(uint8_t)33
		#define _DNumber_34				(uint8_t)34
		#define _DNumber_35				(uint8_t)35
		#define _DNumber_36				(uint8_t)36
		#define _DNumber_37				(uint8_t)37
		#define _DNumber_38				(uint8_t)38
		#define _DNumber_39				(uint8_t)39		
	
		#define _DNumber_40				(uint8_t)40
		#define _DNumber_41				(uint8_t)41
		#define _DNumber_42				(uint8_t)42
		#define _DNumber_43				(uint8_t)43
		#define _DNumber_44				(uint8_t)44
		#define _DNumber_45				(uint8_t)45
		#define _DNumber_46				(uint8_t)46
		#define _DNumber_47				(uint8_t)47
		#define _DNumber_48				(uint8_t)48
		#define _DNumber_49				(uint8_t)49				

		#define _DNumber_50				(uint8_t)50	
		#define _DNumber_51				(uint8_t)51
		#define _DNumber_52				(uint8_t)52
		#define _DNumber_53				(uint8_t)53
		#define _DNumber_54				(uint8_t)54
		#define _DNumber_55				(uint8_t)55
		#define _DNumber_56				(uint8_t)56
		#define _DNumber_57				(uint8_t)57
		#define _DNumber_58				(uint8_t)58
		#define _DNumber_59				(uint8_t)59

		#define _DNumber_60				(uint8_t)60
		#define _DNumber_61				(uint8_t)61
		#define _DNumber_62				(uint8_t)62
		#define _DNumber_63				(uint8_t)63
		#define _DNumber_64				(uint8_t)64
		#define _DNumber_65				(uint8_t)65
		#define _DNumber_66				(uint8_t)66
		#define _DNumber_67				(uint8_t)67
		#define _DNumber_68				(uint8_t)68
		#define _DNumber_69				(uint8_t)69
		
		#define _DNumber_70				(uint8_t)70
		#define _DNumber_71				(uint8_t)71
		#define _DNumber_72				(uint8_t)72
		#define _DNumber_73				(uint8_t)73
		#define _DNumber_74				(uint8_t)74
		#define _DNumber_75				(uint8_t)75
		#define _DNumber_76				(uint8_t)76
		#define _DNumber_77				(uint8_t)77
		#define _DNumber_78				(uint8_t)78
		#define _DNumber_79				(uint8_t)79

		#define _DNumber_80				(uint8_t)80
		#define _DNumber_81				(uint8_t)81
		#define _DNumber_82				(uint8_t)82
		#define _DNumber_83				(uint8_t)83
		#define _DNumber_84				(uint8_t)84
		#define _DNumber_85				(uint8_t)85
		#define _DNumber_86				(uint8_t)86
		#define _DNumber_87				(uint8_t)87
		#define _DNumber_88				(uint8_t)88
		#define _DNumber_89				(uint8_t)89
		
		#define _DNumber_90				(uint8_t)90
		#define _DNumber_91				(uint8_t)91
		#define _DNumber_92				(uint8_t)92
		#define _DNumber_93				(uint8_t)93
		#define _DNumber_94				(uint8_t)94
		#define _DNumber_95				(uint8_t)95
		#define _DNumber_96				(uint8_t)96
		#define _DNumber_97				(uint8_t)97
		#define _DNumber_98				(uint8_t)98
		#define _DNumber_99				(uint8_t)99				
		
		#define _DNumber_100			(uint8_t)100
		
 
	/************************GPIOS*****************************/
	#define GPIO_Pin_0 					LL_GPIO_PIN_0  /*!< Select pin 0 */
	#define GPIO_Pin_1 					LL_GPIO_PIN_1  /*!< Select pin 1 */
	#define GPIO_Pin_2 					LL_GPIO_PIN_2  /*!< Select pin 2 */
	#define GPIO_Pin_3 					LL_GPIO_PIN_3  /*!< Select pin 3 */
	#define GPIO_Pin_4 					LL_GPIO_PIN_4  /*!< Select pin 4 */
	#define GPIO_Pin_5 					LL_GPIO_PIN_5  /*!< Select pin 5 */
	#define GPIO_Pin_6 					LL_GPIO_PIN_6  /*!< Select pin 6 */
	#define GPIO_Pin_7 					LL_GPIO_PIN_7  /*!< Select pin 7 */
	#define GPIO_Pin_8 					LL_GPIO_PIN_8  /*!< Select pin 8 */
	#define GPIO_Pin_9 					LL_GPIO_PIN_9  /*!< Select pin 9 */
	#define GPIO_Pin_10					LL_GPIO_PIN_10 /*!< Select pin 10 */
	#define GPIO_Pin_11					LL_GPIO_PIN_11 /*!< Select pin 11 */
	#define GPIO_Pin_12					LL_GPIO_PIN_12 /*!< Select pin 12 */
	#define GPIO_Pin_13					LL_GPIO_PIN_13 /*!< Select pin 13 */
	#define GPIO_Pin_14					LL_GPIO_PIN_14 /*!< Select pin 14 */
	#define GPIO_Pin_15					LL_GPIO_PIN_15 /*!< Select pin 15 */


	#define GPIO_Mode_IN 				LL_GPIO_MODE_INPUT   		/*!< Select input mode */
	#define GPIO_Mode_OUT 			LL_GPIO_MODE_OUTPUT  		/*!< Select output mode */
	#define GPIO_Mode_AF 				LL_GPIO_MODE_ALTERNATE  /*!< Select alternate function mode */
	#define GPIO_Mode_AN 				LL_GPIO_MODE_ANALOG     /*!< Select analog mode */


	#define GPIO_Speed_2MHz 		LL_GPIO_SPEED_FREQ_LOW      /*!< Select I/O low output speed    */
	#define GPIO_Speed_25MHz 		LL_GPIO_SPEED_FREQ_MEDIUM   /*!< Select I/O medium output speed */
	#define GPIO_Speed_50MHz 		LL_GPIO_SPEED_FREQ_HIGH     /*!< Select I/O fast output speed   */
	#define GPIO_Speed_100MHz 	LL_GPIO_SPEED_FREQ_VERY_HIGH  /*!< Select I/O high output speed   */

	
	#define GPIO_OType_PP 			LL_GPIO_OUTPUT_PUSHPULL    /*!< Select push-pull as output type */
	#define GPIO_OType_OD 			LL_GPIO_OUTPUT_OPENDRAIN   /*!< Select open-drain as output type */


	#define GPIO_PuPd_NOPULL 		LL_GPIO_PULL_NO   /*!< Select I/O no pull */
	#define GPIO_PuPd_UP 				LL_GPIO_PULL_UP   /*!< Select I/O pull up */
	#define GPIO_PuPd_DOWN			LL_GPIO_PULL_DOWN /*!< Select I/O pull down */
	
	
	#define GPIO_AF_USARTx			LL_GPIO_AF_1  		/*!< Select alternate function 1 */


	#define RCC_PBxPeriph_GPIOA			LL_IOP_GRP1_PERIPH_GPIOA	
	#define RCC_PBxPeriph_GPIOB 		LL_IOP_GRP1_PERIPH_GPIOB
	#define RCC_PBxPeriph_GPIOC 		LL_IOP_GRP1_PERIPH_GPIOC
	#define RCC_PBxPeriph_GPIOD			LL_IOP_GRP1_PERIPH_GPIOD
	#if defined(GPIOE)
			#define RCC_PBxPeriph_GPIOE			LL_IOP_GRP1_PERIPH_GPIOE
	#endif
			#define RCC_PBxPeriph_GPIOF			LL_IOP_GRP1_PERIPH_GPIOF


	#define FnGpioPortXClockEnable(x)  LL_IOP_GRP1_EnableClock(x)	
	#define FnGpioPortAClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOA)	
	#define FnGpioPortBClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOB)	
	#define FnGpioPortCClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOC)	
	#define FnGpioPortDClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOD)	
	#if defined(GPIOE)
	#define FnGpioPortEClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOE)	
	#endif
	#define FnGpioPortFClockEnable  	 LL_IOP_GRP1_EnableClock (RCC_PBxPeriph_GPIOF);
 
	#define GPIO_Init(x,y) 						 LL_GPIO_Init(x,y)
	#define GPIO_SetBits(x,y)					 LL_GPIO_SetOutputPin(x,y)	
	#define GPIO_ResetBits(x,y)				 LL_GPIO_ResetOutputPin(x,y)
	#define	GPIO_ToggleBits(x,y)			 LL_GPIO_TogglePin(x,y)	
	#define	GPIO_ReadInputDataBit(x,y) LL_GPIO_IsInputPinSet(x,y)



/************************USART*****************************/

	#define USART_WordLength_8b   		 LL_USART_DATAWIDTH_8B 	  /*!< 8 bits word length : Start bit, 8 data bits, n stop bits */ 
	#define USART_WordLength_9b     	 LL_USART_DATAWIDTH_9B    /*!< 9 bits word length : Start bit, 9 data bits, n stop bits */
		 
		 
	#define USART_StopBits_0_5				 LL_USART_STOPBITS_0_5    /*!< 0.5 stop bit */
	#define USART_StopBits_1					 LL_USART_STOPBITS_1      /*!< 1 stop bit */
	#define USART_StopBits_1_5				 LL_USART_STOPBITS_1_5    /*!< 1.5 stop bits */
	#define USART_StopBits_2					 LL_USART_STOPBITS_2      /*!< 2 stop bits */		 


	#define USART_Parity_No 					 LL_USART_PARITY_NONE       /*!< Parity control disabled */
	#define USART_Parity_Even 				 LL_USART_PARITY_EVEN       /*!< Parity control enabled and Even Parity is selected */
	#define USART_Parity_Odd 					 LL_USART_PARITY_ODD           


	#define USART_HardwareFlowControl_None  	LL_USART_HWCONTROL_NONE    		/*!< CTS and RTS hardware flow control disabled */
	#define USART_HardwareFlowControl_RTS   	LL_USART_HWCONTROL_RTS     		/*!< RTS output enabled, data is only requested when there is space in the receive buffer */
	#define USART_HardwareFlowControl_CTS   	LL_USART_HWCONTROL_CTS     	  /*!< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
	#define USART_HardwareFlowControl_RTS_CTS  LL_USART_HWCONTROL_RTS_CTS /*!< CTS and RTS hardware flow control enabled */


	#define USART_Mode_Rx    					LL_USART_DIRECTION_RX     /*!< Transmitter is disabled and Receiver is enabled */
	#define USART_Mode_Tx    					LL_USART_DIRECTION_TX     /*!< Transmitter is enabled and Receiver is disabled */
	#define USART_Mode_Tx_RX 					LL_USART_DIRECTION_TX_RX  /*!< Transmitter and Receiver are enabled */

	#define USART_Init(x,y)						LL_USART_Init(x,y)	
	#define USART_Enable(x)						LL_USART_Enable(x)	

/************************CLock*****************************/
	#define RCC_PB2Periph_TIM1 				LL_APB2_GRP1_PERIPH_TIM1
	#define RCC_PB2Periph_TIM14 			LL_APB2_GRP1_PERIPH_TIM14
	#define RCC_PB2Periph_TIM15 			LL_APB2_GRP1_PERIPH_TIM15 
	#define RCC_PB2Periph_TIM16 			LL_APB2_GRP1_PERIPH_TIM16
	#define RCC_PB2Periph_TIM17 			LL_APB2_GRP1_PERIPH_TIM17
	#define RCC_PB2Periph_SPI1				LL_APB2_GRP1_PERIPH_SPI1
	#define RCC_PB2Periph_USART1 			LL_APB2_GRP1_PERIPH_USART1
	#define RCC_PB2Periph_ADC 				LL_APB2_GRP1_PERIPH_ADC
	#define RCC_PB2Periph_SYSCFG  		LL_APB2_GRP1_PERIPH_SYSCFG	


	#define RCC_PB1Periph_RTC 				LL_APB1_GRP1_PERIPH_RTC
	#define RCC_PB1Periph_WWDG 				LL_APB1_GRP1_PERIPH_WWDG
	#define RCC_PB1Periph_TIM2 				LL_APB1_GRP1_PERIPH_TIM2 
	#define RCC_PB1Periph_TIM3 				LL_APB1_GRP1_PERIPH_TIM3
	#define RCC_PB1Periph_TIM6 				LL_APB1_GRP1_PERIPH_TIM6 
	#define RCC_PB1Periph_TIM7 				LL_APB1_GRP1_PERIPH_TIM7 
	#define RCC_PB1Periph_SPI2 				LL_APB1_GRP1_PERIPH_SPI2
	#define RCC_PB1Periph_I2C1 				LL_APB1_GRP1_PERIPH_I2C1
	#define RCC_PB1Periph_I2C2 				LL_APB1_GRP1_PERIPH_I2C2	
	#define RCC_PB1Periph_USART2 			LL_APB1_GRP1_PERIPH_USART2
	#define RCC_PB1Periph_USART3 			LL_APB1_GRP1_PERIPH_USART3 
	#define RCC_PB1Periph_USART4 			LL_APB1_GRP1_PERIPH_USART4
	#define RCC_PB1Periph_DAC1 				LL_APB1_GRP1_PERIPH_DAC1
	#define RCC_PB1Periph_UCPD1 			LL_APB1_GRP1_PERIPH_UCPD1  
	#define RCC_PB1Periph_UCPD2 			LL_APB1_GRP1_PERIPH_UCPD2  
	#define RCC_PB1Periph_DBGMCU 			LL_APB1_GRP1_PERIPH_DBGMCU
	#define RCC_PB1Periph_CEC 				LL_APB1_GRP1_PERIPH_CEC
	#define RCC_PB1Periph_PWR 				LL_APB1_GRP1_PERIPH_PWR	
	#define RCC_PB1Periph_PUART1 			LL_APB1_GRP1_PERIPH_LPUART1	
	#define RCC_PB1Periph_LPTIM2 			LL_APB1_GRP1_PERIPH_LPTIM2 
	#define RCC_PB1Periph_LPTIM1		 	LL_APB1_GRP1_PERIPH_LPTIM1  


 #define RCC_PB1PeriphClockCmd(x,y)  (y == ENABLE) ? LL_APB1_GRP1_EnableClock(x) : LL_APB1_GRP1_DisableClock(x)
 #define RCC_PB2PeriphClockCmd(x,y)  (y == ENABLE) ? LL_APB2_GRP1_EnableClock(x) : LL_APB2_GRP1_DisableClock(x)
 
 
/* ----------------------------------------------------------------------------
 *                           EXTERNAL FUNCTIONS
 *  ----------------------------------------------------------------------------
*/


/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

  /* Timeout values for ADC operations. */
  /* (calibration, enable settling time, disable settling time, ...)          */
  /* Values defined to be higher than worst cases: low clock frequency,       */
  /* maximum prescalers.                                                      */
  /* Unit: ms                                                                 */
  #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS  (   1U)
  #define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
  #define ADC_ENABLE_TIMEOUT_MS            (   1U)
  #define ADC_DISABLE_TIMEOUT_MS           (   1U)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
  #define ADC_CONVERSION_TIMEOUT_MS        (4000U)
  
  /* Delay between ADC end of calibration and ADC enable.                     */
  /* Delay estimation in CPU cycles: Case of ADC enable done                  */
  /* immediately after ADC calibration, ADC clock setting slow                */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
  

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       (3300U)

/* Definitions of data related to this example */
  /* ADC unitary conversion timeout */
  /* Considering ADC settings, duration of 1 ADC conversion should always    */
  /* be lower than 1ms.                                                      */
  #define ADC_UNITARY_CONVERSION_TIMEOUT_MS (   1U)

  /* Init variable out of expected ADC conversion data range */
  #define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)


 
/*----------------------------------------------------------------------------
 *                           FUNCTIONS DECLARATION
 * ----------------------------------------------------------------------------
*/





/* ----------------------------------------------------------------------------
 *                           EXTERNAL VARIABLES
 * ----------------------------------------------------------------------------
 */
 
 
 /* ----------------------------------------------------------------------------
 *                           EXTERNAL FUNCTIONS
 *  ----------------------------------------------------------------------------
 */
 
 
 /*----------------------------------------------------------------------------
 *                           FUNCTIONS DECLARATION
 * ----------------------------------------------------------------------------
 */

/* ----------------------------------------------------------------------------
 *                           MACROS
 * ----------------------------------------------------------------------------
 */


/* ----------------------------------------------------------------------------
 *                           GLOBAL VARIABLES
 * ----------------------------------------------------------------------------
 */
 
/* ----------------------------------------------------------------------------
 *                           EXTERNAL VARIABLES
 * ----------------------------------------------------------------------------
 */
 
 
 /* ----------------------------------------------------------------------------
 *                           EXTERNAL FUNCTIONS
 *  ----------------------------------------------------------------------------
 */
 
 
 extern 	__IO uint16_t uhADCxConvertedData								;
 extern 	__IO uint16_t uhADCxConvertedData_Voltage_mVolt ;
 extern   __IO  uint8_t ubAdcGrpRegularUnitaryConvStatus  ;

 
 /*----------------------------------------------------------------------------
 *                           FUNCTIONS DECLARATION
 * ----------------------------------------------------------------------------
 */

/* Exported functions prototypes ---------------------------------------------*/

	  void 		  FnGPIOInitINorOUT	(GPIO_TypeDef *uiPort1,uint32_t uiPin1,uint8_t ucINorOUT);

		void 			FnUARTInit			(USART_TypeDef *USARTx,	uint32_t uiBaudRate, 
															 GPIO_TypeDef *uiPort1, GPIO_TypeDef *uiPort2, 
															 uint32_t 			uiPin1, uint32_t uiPin2 );
		
		void 			FnUSART_SendData( USART_TypeDef *stUSARTNo,
																uint8_t *ucData, int16_t ucSize);
	
		void 			FnADC1Init			(void);
		
		void 			FnActivateADC		(void);
		
		int32_t 	FnADCCmd				(uint8_t ucChannel);
		
		void 			ConversionStartPoll_ADC_GrpRegular(uint8_t ucSelec);
			
		void 			FnTimer1Init		(void);
		
		void 			FnDelayTimeDecrement(void);

		void 			FnDelay_25ms				(__IO uint32_t nTime);		
		
		
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
