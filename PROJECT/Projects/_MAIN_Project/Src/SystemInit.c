/******************************************************************************
  * @file    Templates_LL/Src/SystemInit.c
  * @author  MCD Application Team
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
		#include "SystemInit.h"
		#include "main.h"
	

/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */ 

/* Variables for ADC conversion data computation to physical values */
__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0U;  /* Value of voltage calculated from ADC conversion data (unit: mV) */


/* Variable to report status of ADC group regular unitary conversion          */
/*  0: ADC group regular unitary conversion is not completed                  */
/*  1: ADC group regular unitary conversion is completed                      */
/*  2: ADC group regular unitary conversion has not been started yet          */
/*     (initial state)                                                        */
__IO uint8_t ubAdcGrpRegularUnitaryConvStatus = 2U; /* Variable set into ADC interruption callback */


/*** TIMER VAR ***/
__IO	uint32_t TimingDelay;



void FnGPIOInitINorOUT(GPIO_TypeDef *uiPort1,uint32_t uiPin1,uint8_t ucINorOUT)
{

	LL_GPIO_InitTypeDef  GPIO_InitStruct  = {RESET};

				 if (uiPort1 == GPIOA)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOA );
	else if	(uiPort1 == GPIOB)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOB );
	else if	(uiPort1 == GPIOC)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOC );
	else if	(uiPort1 == GPIOD)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOD );
	#if defined(GPIOE)
	else if	(uiPort1 == GPIOE)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOE );
	#endif
	else if	(uiPort1 == GPIOF)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOF );

	
	GPIO_InitStruct.Pin 				= uiPin1;
  GPIO_InitStruct.Mode 				= ucINorOUT;
  GPIO_InitStruct.Speed 			= GPIO_Speed_50MHz;
  GPIO_InitStruct.OutputType 	= GPIO_OType_PP		;
  GPIO_InitStruct.Pull 				= GPIO_PuPd_UP		;
  GPIO_Init(uiPort1, &GPIO_InitStruct);

}


/**
  * @brief  The application entry point.
  * @retval int
  */
void FnUARTInit(USART_TypeDef *stUSARTNo , uint32_t uiBaudRate, 
								 GPIO_TypeDef *uiPort1	 , GPIO_TypeDef *uiPort2,
											 uint32_t uiPin1	 , uint32_t uiPin2 )
{

	
	/*Default structure init*/
  LL_USART_InitTypeDef USART_InitStruct = {RESET};
  LL_GPIO_InitTypeDef  GPIO_InitStruct  = {RESET};

  /* Peripheral clock enable */
	if (	stUSARTNo == USART1	)
		 {	RCC_PB2PeriphClockCmd  ( RCC_PB2Periph_USART1 , ENABLE );		}
	else
	if (	stUSARTNo == USART2	)
		 {	RCC_PB1PeriphClockCmd  ( RCC_PB1Periph_USART2 , ENABLE );		}		

/*		 
	else
	if (	stUSARTNo == USART3	)
		 {	RCC_PB1PeriphClockCmd  ( RCC_PB1Periph_USART3 , ENABLE );		}				 
	else
	if (	stUSARTNo == USART4	)
		 {	RCC_PB1PeriphClockCmd  ( RCC_PB1Periph_USART4 , ENABLE );		}			
*/		
		 
	FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOA );
	
			 if (uiPort1 == GPIOA)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOA );
	else if	(uiPort1 == GPIOB)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOB );
	else if	(uiPort1 == GPIOC)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOC );
	else if	(uiPort1 == GPIOD)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOD );
	#if defined(GPIOE)
	else if	(uiPort1 == GPIOE)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOE );
	#endif
	else if	(uiPort1 == GPIOF)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOF );


	
			 if (uiPort2 == GPIOA)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOA );
	else if	(uiPort2 == GPIOB)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOB );
	else if	(uiPort2 == GPIOC)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOC );
	else if	(uiPort2 == GPIOD)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOD );
	#if defined(GPIOE)
	else if	(uiPort2 == GPIOE)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOE );
	#endif
	else if	(uiPort2 == GPIOF)
					FnGpioPortXClockEnable ( RCC_PBxPeriph_GPIOF );
	
	
	/**USART1 GPIO Configuration  
  PA2   ------> USART1_TX
  PA3   ------> USART1_RX */
  GPIO_InitStruct.Pin 				= uiPin1;
  GPIO_InitStruct.Mode 				= GPIO_Mode_AF;
  GPIO_InitStruct.Speed 			= GPIO_Speed_50MHz;
  GPIO_InitStruct.OutputType 	= GPIO_OType_PP	;
  GPIO_InitStruct.Pull 				= GPIO_PuPd_UP	;
  GPIO_InitStruct.Alternate 	= GPIO_AF_USARTx;
  GPIO_Init(uiPort1, &GPIO_InitStruct);

	
  GPIO_InitStruct.Pin 			 = uiPin2;
  GPIO_InitStruct.Mode 			 = GPIO_Mode_AF;
  GPIO_InitStruct.Speed 		 = GPIO_Speed_50MHz;
  GPIO_InitStruct.OutputType = GPIO_OType_PP;
  GPIO_InitStruct.Pull 			 = GPIO_PuPd_UP;
  GPIO_InitStruct.Alternate  = GPIO_AF_USARTx;
  GPIO_Init(uiPort2, &GPIO_InitStruct);


	/*USART Structure Init*/	
  USART_InitStruct.BaudRate 					 = uiBaudRate					;
  USART_InitStruct.DataWidth 					 = USART_WordLength_8b;
  USART_InitStruct.StopBits 					 = USART_StopBits_1		;
  USART_InitStruct.Parity 					 	 = USART_Parity_No		;
  USART_InitStruct.TransferDirection 	 = USART_Mode_Tx_RX		;
  USART_InitStruct.HardwareFlowControl = USART_HardwareFlowControl_None;
	if(uiBaudRate <= 230400) 
		 USART_InitStruct.OverSampling 		 = LL_USART_OVERSAMPLING_16;
	else
		 USART_InitStruct.OverSampling 		 = LL_USART_OVERSAMPLING_8;
	
	USART_InitStruct.PrescalerValue			 = LL_USART_PRESCALER_DIV1;
  USART_Init(stUSARTNo, &USART_InitStruct);
	
	
  LL_USART_SetTXFIFOThreshold(stUSARTNo, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(stUSARTNo, LL_USART_FIFOTHRESHOLD_1_8);
  
	LL_USART_DisableFIFO		(stUSARTNo);
  LL_USART_ConfigAsyncMode(stUSARTNo);
  USART_Enable						(stUSARTNo);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(stUSARTNo))) || (!(LL_USART_IsActiveFlag_REACK(stUSARTNo))))
  {  }


}	


/**
  * @brief  The application entry point.
  * @retval int
  */
void FnUSART_SendData( USART_TypeDef *stUSARTNo, uint8_t *ucData, int16_t ucSize)
{
	
			uint16_t uiSendCnt = RESET ;
			/* Send characters one per one, until last char to be sent */
			while ( ucSize > (int16_t)0 )
						{
							/* Wait for TXE flag to be raised */
							while (!LL_USART_IsActiveFlag_TXE(stUSARTNo))
							{				}
							/* If last char to be sent, clear TC flag */
							if (ucSize ==  RESET)	{	LL_USART_ClearFlag_TC(stUSARTNo); }
							/* Write character in Transmit Data register.
							TXE flag is cleared by writing data in TDR register */
							LL_USART_TransmitData8(stUSARTNo, ucData[uiSendCnt++]);
							ucSize--;
						}

			/* Wait for TC flag to be raised for last char */
			while (!LL_USART_IsActiveFlag_TC(stUSARTNo))
						{			}
}




/**
  * @brief  The application entry point.
  * @retval int
  */
void FnTimer1Init(void)
{

	static uint32_t tim_prescaler = RESET;
	static uint32_t tim_period 		= RESET;
	static uint32_t TimOutClock 	= SET	 ;
	
	/*	In this example TIM1 input clock TIM1CLK is set to APB2 clock (PCLK2),   
			since APB2 pre-scaler is equal to 1.                                     
      TIM1CLK = PCLK2                                                       
      PCLK1 = HCLK                                                          
      => TIM1CLK = SystemCoreClock (56 MHz)                                 
  */
	
  tim_prescaler = __LL_TIM_CALC_PSC(SystemCoreClock, (uint16_t)10000);
	
  /* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)             */
  TimOutClock = SystemCoreClock / 1 ;
	
	//(((__TIMCLK__)/((__FREQ__) * ((__PSC__) + 1U))) - 1U) 
  tim_period = __LL_TIM_CALC_ARR( TimOutClock, tim_prescaler, (uint8_t)80 );

	
  LL_TIM_InitTypeDef 				TIM_InitStruct 		 = {RESET};
  LL_TIM_BDTR_InitTypeDef 	TIM_BDTRInitStruct = {RESET};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn , RESET);
  NVIC_EnableIRQ	(TIM1_BRK_UP_TRG_COM_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

	sprintf(ucDebugBuff,"PRE=%d PER=%d\r\n",tim_prescaler,tim_period); 	
	FnUSART_SendData(USART1,(uint8_t *)ucDebugBuff, strlen(ucDebugBuff));
	
	
  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler 					= tim_prescaler				 ;
  TIM_InitStruct.CounterMode 				= LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload 				= tim_period   				 ;
  TIM_InitStruct.ClockDivision 			= LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter 	= RESET 							 ;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  
	
	LL_TIM_DisableARRPreload		 (TIM1);
  LL_TIM_SetClockSource				 (TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput			 (TIM1, LL_TIM_TRGO_RESET );
  LL_TIM_SetTriggerOutput2		 (TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
	
  TIM_BDTRInitStruct.BreakAFMode 	= LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);



  /* Clear the update flag */
  LL_TIM_ClearFlag_UPDATE(TIM1);

  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);

  /* Enable counter */
  LL_TIM_EnableCounter(TIM1);

}

/*****************************************************************************
 **@Function 		  	: 	TimingDelay_Decrement
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/ 
void FnDelayTimeDecrement(void)
{
  if ( TimingDelay != _DNumber_0)
		 { 
			 TimingDelay--;
		 }	
}

/*****************************************************************************
 **@Function 		  	: 	FnDelay_25ms
 **@Descriptions		: 	
 **@parameters			: 	
 **@return					: 	
*****************************************************************************/  
void FnDelay_25ms(__IO uint32_t nTime)
{ 
				 TimingDelay  =  nTime * _DNumber_2;
  while( TimingDelay != _DNumber_0 )
	{
		/* Reload IWDG counter */
   //  IWDG_ReloadCounter();
	}
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void FnADC1Init(void)
{


  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {RESET};
  LL_ADC_InitTypeDef ADC_InitStruct 				= {RESET};
  LL_GPIO_InitTypeDef GPIO_InitStruct 			= {RESET};

	
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration  
	PA0	  ------> ADC1_IN0	
	PA1	  ------> ADC1_IN1		
  PA2   ------> ADC1_IN2
	PA3	  ------> ADC1_IN3
	PA4	  ------> ADC1_IN4
	PA5	  ------> ADC1_IN5		
  */
  GPIO_InitStruct.Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
												| GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;
  GPIO_InitStruct.Mode 	= GPIO_Mode_AN		;
  GPIO_InitStruct.Pull 	= GPIO_PuPd_NOPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct)		;

  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority( ADC1_IRQn , RESET);
  NVIC_EnableIRQ  ( ADC1_IRQn);
	

  /** Configure the global features of the ADC 
		(Clock, Resolution, Data Alignment and number of conversion) */
  ADC_REG_InitStruct.TriggerSource 		= LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength 	= LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode 	= LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer 			= LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun 					= LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	
  LL_ADC_SetOverSamplingScope					(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode			(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_REG_SetSequencerConfigurable	(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_39CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
 
	ADC_InitStruct.Clock 					= LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_InitStruct.DataAlignment 	= LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.Resolution 		= LL_ADC_RESOLUTION_12B;	
  ADC_InitStruct.LowPowerMode 	= LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

 
  /* Configuration of ADC interruptions */
  /* Enable interruption ADC group regular overrun */
  LL_ADC_EnableIT_OVR(ADC1);
}


/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @note   Operations:
  *         - ADC instance
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           Feature not available (feature not available on this STM32 serie)
  * @param  None
  * @retval None
  */
void FnActivateADC(void)
{
	
  __IO uint32_t wait_loop_index = _DNumber_0;
  __IO uint32_t backup_setting_adc_dma_transfer = _DNumber_0;
  
  /*## Operation on ADC hierarchical scope: ADC instance #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == _DNumber_0)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    
    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US 
										 * (SystemCoreClock / ((uint32_t)100000 * _DNumber_2))) / _DNumber_10);
    
		while( wait_loop_index != _DNumber_0)
				 { wait_loop_index--;	 }
    
    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 serie: Calibration factor is           */
    /*       available in data register and also transfered by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in DMA destination address, DMA transfer must be disabled during */
    /*       calibration.                                                     */
    
		backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != _DNumber_0)
    {    }
    
    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
    
    /* Delay between ADC end of calibration and ADC enable.              */
    /* Note: Variable divided by 2 to compensate partially               */
    /*       CPU processing cycles (depends on compilation optimization).*/
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> _DNumber_1);
    while(wait_loop_index != _DNumber_0)
    {
      wait_loop_index--;
    }
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == _DNumber_0)
    {    }
    
    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }
  
  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */
  
  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 serie */ 
  
}


/**
  * @brief ADC1 Initialization Function
  * @param  None
  * @retval None
  */
void ConversionStartPoll_ADC_GrpRegular(uint8_t ucSelec)
{
  
	if(ucSelec == _DNumber_1)
	{	
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
		LL_ADC_SetChannelSamplingTime(ADC1,  LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	else
	if(ucSelec == _DNumber_2)
	{		
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);	
		LL_ADC_SetChannelSamplingTime(ADC1,  LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	else
	if(ucSelec == _DNumber_3)
	{	
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);	
		LL_ADC_SetChannelSamplingTime(ADC1,  LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	else
	if(ucSelec == _DNumber_4)
	{		
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);	
		LL_ADC_SetChannelSamplingTime(ADC1,  LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	else
	if(ucSelec == _DNumber_5)
	{	
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);	
		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	else
	if(ucSelec == _DNumber_6)
	{
		LL_ADC_REG_SetSequencerRanks (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);	
		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_COMMON_1);
	}
	
	
  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of this feature is conditioned to     */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if ((LL_ADC_IsEnabled(ADC1) == _DNumber_1)      	&&
      (LL_ADC_IsDisableOngoing(ADC1) == _DNumber_0) &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == _DNumber_0) )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
    // LED_Blinking(LED_BLINK_ERROR);
		
			sprintf(ucDebugBuff,"ERROR ADC\n\r");
			FnUSART_SendData(USART1,(uint8_t *)ucDebugBuff, strlen(ucDebugBuff));
  }
    
  while (LL_ADC_IsActiveFlag_EOC(ADC1) == _DNumber_0)
  {	  }
  
  /* Clear flag ADC group regular end of unitary conversion */
  /* Note: This action is not needed here, because flag ADC group regular   */
  /*       end of unitary conversion is cleared automatically when          */
  /*       software reads conversion data from ADC data register.           */
  /*       Nevertheless, this action is done anyway to show how to clear    */
  /*       this flag, needed if conversion data is not always read          */
  /*       or if group injected end of unitary conversion is used (for      */
  /*       devices with group injected available).                          */
  LL_ADC_ClearFlag_EOC(ADC1);
  
}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */

int32_t	 FnADCCmd(uint8_t ucChannel)
{

  /* Reset status variable of ADC unitary conversion before performing        */
  /* a new ADC conversion start.                                              */
  /* Note: Optionally, for this example purpose, check ADC unitary            */
  /*       conversion status before starting another ADC conversion.          */
  if ( ubAdcGrpRegularUnitaryConvStatus != _DNumber_0)
		 {
			 ubAdcGrpRegularUnitaryConvStatus = _DNumber_0;
			 return (int8_t)-1;
		 }
  
  /* Init variable containing ADC conversion data */
  uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;
  
  /* Perform ADC group regular conversion start, poll for conversion          */
  /* completion.                                                              */
  ConversionStartPoll_ADC_GrpRegular(ucChannel);
  
  /* Retrieve ADC conversion data */
  /* (data scale corresponds to ADC resolution: 12 bits) */
  uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC1);
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
		uhADCxConvertedData_Voltage_mVolt = 
	__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
  
  /* Update status variable of ADC unitary conversion */
		 ubAdcGrpRegularUnitaryConvStatus = _DNumber_1;
	
	return uhADCxConvertedData_Voltage_mVolt ;

}


