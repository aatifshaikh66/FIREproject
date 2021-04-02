/**
  ******************************************************************************
  * @file    Project/STM32G0xx_StdPeriph_Template/app.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    10-OCTOBER-2020
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
  
/* ----------------------------------------------------------------------------
 *                           Includes
 * ----------------------------------------------------------------------------
*/
	 

		#include "main.h"
		#include "SystemInit.h"
		
		
/* ----------------------------------------------------------------------------
 *                           MACROS
 * ----------------------------------------------------------------------------
*/
 

/* ----------------------------------------------------------------------------
 *                           EXTERNAL VARIABLES
 * ----------------------------------------------------------------------------
*/
 
 
/* ----------------------------------------------------------------------------
 *                           GLOBAL VARIABLES
 * ----------------------------------------------------------------------------
*/






/*****************************************************************************
 **@Function 		  	: 	FnUsartPrint
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void SystemClock_Config(void)
{
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != (uint8_t)1 )
  {
  };

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  };

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(16000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);
}



/*****************************************************************************
 **@Function 		  	: 	FnUsartPrint
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnALLInit(void)
{

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR   );

  /* Configure the system clock */
  SystemClock_Config();
	

  /* Initialize all configured peripherals */
	/* Initialize USART FOR DEBUGG */
		 #ifdef _DEBUGG_USART_ENABLE	
						 FnUARTInit(_DEBUGG_USART_NO			,	_DEBUGG_USART_BAUD_RATE, 
								_DEBUGG_USART_TX_PORT , _DEBUGG_USART_RX_PORT	 ,
								_DEBUGG_USART_TX_PIN  , _DEBUGG_USART_RX_PIN	 );
		 	
						 FnUsartPrint("STM32G07x SPL TEST_10!\r\n");		
		 #endif	
	
	
		/* timer init */
		 FnTimer1Init();	
		 #ifdef _DEBUGG_USART_ENABLE	
						 FnUsartPrint("25ms Timer Init!\r\n");		
		 #endif		
			
			
		 	
		FnGPIOInitINorOUT(_SYSTEM_BUZZER_PORT			  , _SYSTEM_BUZZER_PIN				,GPIO_Mode_OUT);/*BUZZER LED GPIO INIT*/  
		FnGPIOInitINorOUT(_SYSTEM_SENSOR1_LED_PORT  , _SYSTEM_SENSOR1_LED_PIN		,GPIO_Mode_OUT);/*SENSOR1 LED GPIO INIT*/ 
		FnGPIOInitINorOUT(_SYSTEM_SENSOR2_LED_PORT	, _SYSTEM_SENSOR2_LED_PIN		,GPIO_Mode_OUT);/*SENSOR2 LED GPIO INIT*/ 
		FnGPIOInitINorOUT(_SYSTEM_SOLENOID1_LED_PORT, _SYSTEM_SOLENOID1_LED_PIN	,GPIO_Mode_OUT);/*SOLENOID1 LED GPIO INIT*/
		FnGPIOInitINorOUT(_SYSTEM_SOLENOID2_LED_PORT, _SYSTEM_SOLENOID2_LED_PIN	,GPIO_Mode_OUT);/*SOLENOID2 LED GPIO INIT*/		
		FnGPIOInitINorOUT(_SYSTEM_HOOTER_LED_PORT		, _SYSTEM_HOOTER_LED_PIN		,GPIO_Mode_OUT);/*HOOTER LED GPIO INIT*/ 	
		FnGPIOInitINorOUT(_SYSTEM_FIRE_LED_PORT			, _SYSTEM_FIRE_LED_PIN			,GPIO_Mode_OUT);/*FIRE LED GPIO INIT*/	
		FnGPIOInitINorOUT(_SYSTEM_MANUAL_LED_PORT		, _SYSTEM_MANUAL_LED_PIN		,GPIO_Mode_OUT);/*MANUAL LED GPIO INIT*/ 		
		FnGPIOInitINorOUT(_SYSTEM_STATUS_LED_PORT		, _SYSTEM_STATUS_LED_PIN		,GPIO_Mode_OUT);/*STATUS LED GPIO INIT*/	
		
		FnGPIOInitINorOUT(_SYSTEM_SOLENOID1_DRIVE_PORT, _SYSTEM_SOLENOID1_DRIVE_PIN	,GPIO_Mode_OUT);/*SOLENOID1 DRIVE GPIO INIT*/		 
		FnGPIOInitINorOUT(_SYSTEM_SOLENOID2_DRIVE_PORT, _SYSTEM_SOLENOID2_DRIVE_PIN	,GPIO_Mode_OUT);/*SOLENOID2 DRIVE GPIO INIT*/		 	 
		FnGPIOInitINorOUT(_SYSTEM_HOOTER_DRIVE_PORT   , _SYSTEM_HOOTER_DRIVE_PIN		,GPIO_Mode_OUT);/*HOOTER DRIVE GPIO INIT*/	
		FnGPIOInitINorOUT(_SYSTEM_RELAY_DRIVE_PORT    , _SYSTEM_RELAY_DRIVE_PIN		  ,GPIO_Mode_OUT);/*HOOTER DRIVE GPIO INIT*/  			
			 
	 
		FnGPIOInitINorOUT(_SYSTEM_BUTTON_TEST_PORT	 , _SYSTEM_BUTTON_TEST_PIN	 , GPIO_Mode_IN); /*STATUS LED GPIO INIT*/		
		FnGPIOInitINorOUT(_SYSTEM_BUTTON_SILENCE_PORT, _SYSTEM_BUTTON_SILENCE_PIN, GPIO_Mode_IN); /*STATUS LED GPIO INIT*/	
			
			
		_SYSTEM_BUZZER_OFF	
		_SYSTEM_SENSOR1_LED_OFF	
		_SYSTEM_SENSOR2_LED_OFF	
		_SYSTEM_SOLENOID1_LED_OFF	
		_SYSTEM_SOLENOID2_LED_OFF	
		_SYSTEM_HOOTER_LED_OFF
		_SYSTEM_FIRE_LED_OFF
		_SYSTEM_MANUAL_LED_OFF
		_SYSTEM_STATUS_LED_OFF	
			
			
		_SYSTEM_SOLENOID1_DRIVE_OFF
		_SYSTEM_SOLENOID2_DRIVE_OFF
		_SYSTEM_HOOTER_DRIVE_OFF
		_SYSTEM_RELAY_DRIVE_OFF
				
			
		/* Initialize ADC */		 
		FnADC1Init	 ();	
		FnActivateADC();			
		for (uint8_t ucLoop = _DNumber_0 ; 
								 ucLoop < _DNumber_80 ; ucLoop++ )
								 FnReadAndAvgTheAdcValue();
		 #ifdef _DEBUGG_USART_ENABLE	
						 FnUsartPrint("IADC Init!\r\n");		
		 #endif		

}


/*****************************************************************************
 **@Function 		  	: 	FnUsartPrint
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnUsartPrint(char *ucData)
{
		 FnUSART_SendData(_DEBUGG_USART_NO,(
		 uint8_t *)ucData, strlen((char*)ucData));
}


/*****************************************************************************
 **@Function 		  	: 	FnReadAndAvgTheAdcValue
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnReadAndAvgTheAdcValue(void)
{
	
			ucADCRet = FnADCCmd(ucADCChannelNo)	;
	if( ucADCRet > _DNumber_0 )
		{
				ucADCBUFF[ucADCChannelNo - _DNumber_1][ucADCVAvgNo] = (uint16_t)ucADCRet ;
				ucADCChannelNo++; 
		if( ucADCChannelNo == ucADCTotalChannel + _DNumber_1 )
			{ ucADCChannelNo  = _DNumber_1; 
				
				
						ucADCVAvgNo++;
				if( ucADCVAvgNo == ucMAXAdcSample )
					{ ucADCVAvgNo	 = _DNumber_0 ;	
						
						
					for(uint8_t  ucChannel_1 = _DNumber_0 ; ucChannel_1 < ucADCTotalChannel ; ucChannel_1++ )
							{
										ucADCSampleTotal 		 = _DNumber_0 ;
								for(uint8_t  ucChannel_2 = _DNumber_0 ; ucChannel_2 < ucMAXAdcSample ; ucChannel_2++ )
										ucADCSampleTotal		 = ucADCSampleTotal + ucADCBUFF[ucChannel_1][ucChannel_2] ;
						
										uiAdcAvgValue[ucChannel_1] = ucADCSampleTotal / (uint16_t)ucMAXAdcSample; 	
										
										#ifdef _DEBUGG_USART_ENABLE
										/*			sprintf( ucDebugBuff ,"V[%u] = A%u %u %u %u %u\n\r", 
																		 ucChannel_1 , uiAdcAvgValue[ucChannel_1]  ,
																		 ucADCBUFF[ucChannel_1][_DNumber_0], ucADCBUFF[ucChannel_1][_DNumber_1],
																		 ucADCBUFF[ucChannel_1][_DNumber_2] ,ucADCBUFF[ucChannel_1][_DNumber_3]  );	
														FnUsartPrint( ucDebugBuff );
										*/
										#endif
							}
							
							stVarADCValue_ALL.uiSensor1Value 	 = uiAdcAvgValue[_DNumber_0] ;
							stVarADCValue_ALL.uiSensor2Value 	 = uiAdcAvgValue[_DNumber_1] ;
							stVarADCValue_ALL.uiSolenoid1Value = uiAdcAvgValue[_DNumber_2] ;
							stVarADCValue_ALL.uiSolenoid2Value = uiAdcAvgValue[_DNumber_3] ;
							stVarADCValue_ALL.uiHooterValue 	 = uiAdcAvgValue[_DNumber_4] ;
							stVarADCValue_ALL.uiExternalSensorValue = uiAdcAvgValue[_DNumber_5]	;
							
							#ifdef _DEBUGG_USART_ENABLE
							 	/*	 	sprintf( ucDebugBuff ,"SEN1= %u SEN2= %u SOL1= %u SOL2=	%u HOT= %u EXT= %u	\n\r", 
															 stVarADCValue_ALL.uiSensor1Value  , stVarADCValue_ALL.uiSensor2Value		 ,
															 stVarADCValue_ALL.uiSolenoid1Value, stVarADCValue_ALL.uiSolenoid2Value	 ,
															 stVarADCValue_ALL.uiHooterValue	 , stVarADCValue_ALL.uiExternalSensorValue	);	
											FnUsartPrint( ucDebugBuff );
							  */
							#endif
				
					}
			}
 	 }		
}

/*****************************************************************************
 **@Function 		  	: 	FnTimerFunction
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnCheckProcessADC(void)
{

	if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
		&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
		&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
		&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
		&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
		&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
		   {
				 _SYSTEM_STATUS_LED_ON	
				 _SYSTEM_FIRE_LED_OFF		
				 _SYSTEM_SENSOR1_LED_OFF
				 _SYSTEM_SENSOR2_LED_OFF		
				 _SYSTEM_SOLENOID1_LED_OFF
				 _SYSTEM_SOLENOID2_LED_OFF
				 
				 
				 _SYSTEM_RELAY_DRIVE_OFF
				 _SYSTEM_HOOTER_DRIVE_OFF
				 _SYSTEM_SOLENOID1_DRIVE_OFF
				 _SYSTEM_SOLENOID2_DRIVE_OFF
				 
			  stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 = _DNumber_0 ;
				stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 = _SYSTEM_BUZZER_MODE_OFF ;			
		   }	
	

		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiSensor1Value >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT)
		 &&(	stVarADCValue_ALL.uiSensor1Value <=  _SYSTEM_ADC_NORMAL_UPPER_LIMIT))
			 {
				 
				 if( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_FIRE  )
					 { stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  = _SYSTEM_SENSOR_OK 		;
						 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET	 = _DNumber_1 					;	
						 _SYSTEM_SOLENOID1_DRIVE_OFF
						 
						 if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status	  == _SYSTEM_SENSOR_OK )	
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status	  == _SYSTEM_SENSOR_OK )
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_OK)
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_OK))
								 { 
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 = _DNumber_0 ;
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 = _SYSTEM_BUZZER_MODE_OFF ;	
									 
									 _SYSTEM_STATUS_LED_ON
									 _SYSTEM_FIRE_LED_OFF	
									 _SYSTEM_RELAY_DRIVE_OFF
									 _SYSTEM_HOOTER_DRIVE_OFF
								 }
							#ifdef _DEBUGG_USART_ENABLE							 
											FnUsartPrint("SENSOR 1 FIRE STOPED!\n\r");
							#endif	
					 
					 }
					else
				  if(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_BREAK )	
						{	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	= _SYSTEM_SENSOR_OK ;
							stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET	  = _DNumber_1 ;							
							_SYSTEM_SENSOR1_LED_OFF
							
						 if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	== _SYSTEM_SENSOR_OK )	
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status	  == _SYSTEM_SENSOR_OK )
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_OK)
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_OK))
								 { 
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 = _DNumber_0 ;
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 = _SYSTEM_BUZZER_MODE_OFF ;	
									 
									 _SYSTEM_STATUS_LED_ON
									 _SYSTEM_FIRE_LED_OFF	
									 _SYSTEM_RELAY_DRIVE_OFF
									 _SYSTEM_HOOTER_DRIVE_OFF
								 }							
							
							#ifdef _DEBUGG_USART_ENABLE							 
											FnUsartPrint("SENSOR 1 RE-CONNECT!\n\r");
							#endif	
						}
			 }
	 else
		 if(  stVarADCValue_ALL.uiSensor1Value >=  _SYSTEM_ADC_FIRE_SET_LIMIT	  )
			 {
				 if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_OK 		)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_BREAK ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  = _SYSTEM_SENSOR_FIRE 	; 
							
						 
							 _SYSTEM_FIRE_LED_ON	
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_SENSOR1_LED_OFF	
							 _SYSTEM_SOLENOID1_DRIVE_ON
							 _SYSTEM_HOOTER_DRIVE_ON
							 _SYSTEM_RELAY_DRIVE_ON	
							
								stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET	  = 
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		= _DNumber_90 ;
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		= _SYSTEM_BUZZER_MODE_HIGH_BEEP ;
								#ifdef _DEBUGG_USART_ENABLE							 
												FnUsartPrint("FIRE CAUSED BY SENSOR 1\n\r");
								#endif
							
								for (uint8_t ucLoop = _DNumber_0 ; 
								ucLoop < _DNumber_80 ; ucLoop++ )
								FnReadAndAvgTheAdcValue();
					 }
					 
			 }				 
	 else
		 if(  stVarADCValue_ALL.uiSensor1Value  <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			 {
				 if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_OK 		)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status == _SYSTEM_SENSOR_FIRE ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  = _SYSTEM_SENSOR_BREAK ;
							

							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_SENSOR1_LED_ON
							
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt  = _DNumber_90 					;
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;
							
							 #ifdef _DEBUGG_USART_ENABLE							 
											 FnUsartPrint("SENSOR 1 BREAK\n\r");
							 #endif
						}							
			 }
			 
			 
			 
		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiSensor2Value >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT )
		 &&(	stVarADCValue_ALL.uiSensor2Value <=  _SYSTEM_ADC_NORMAL_UPPER_LIMIT ))
			 {
				 
				 if( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_FIRE  )
					 { stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  = _SYSTEM_SENSOR_OK 		;
						 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET	 = _DNumber_1 					;	
						 _SYSTEM_SOLENOID2_DRIVE_OFF

						 
						 if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	== _SYSTEM_SENSOR_OK )	
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	== _SYSTEM_SENSOR_OK )
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_OK)
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_OK))
								 { 
									 
									 _SYSTEM_STATUS_LED_ON
									 _SYSTEM_FIRE_LED_OFF	
									 _SYSTEM_RELAY_DRIVE_OFF
									 _SYSTEM_HOOTER_DRIVE_OFF
								 }
							#ifdef _DEBUGG_USART_ENABLE							 
											FnUsartPrint("SENSOR 2 FIRE STOPED!\n\r");
							#endif	
								 
								// stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 	 = _DNumber_0 ;
								// stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 	 = _SYSTEM_BUZZER_MODE_OFF ;	
					 
					 }
					else
				  if(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_BREAK )	
						{	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	= _SYSTEM_SENSOR_OK ;
							stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET	  = _DNumber_1 ;							
							_SYSTEM_SENSOR2_LED_OFF
							
						 if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	== _SYSTEM_SENSOR_OK )	
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	== _SYSTEM_SENSOR_OK )
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_OK)
							 &&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_OK))
								 { 	
									 
									 _SYSTEM_STATUS_LED_ON
									 _SYSTEM_FIRE_LED_OFF	
									 _SYSTEM_RELAY_DRIVE_OFF
									 _SYSTEM_HOOTER_DRIVE_OFF
									 
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 = _DNumber_0 ;
									 //stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 = _SYSTEM_BUZZER_MODE_OFF ;	
								 }							
							
							#ifdef _DEBUGG_USART_ENABLE							 
											FnUsartPrint("SENSOR 2 RE-CONNECT!\n\r");
							#endif	
						}
			 }
	 else
		 if( stVarADCValue_ALL.uiSensor2Value >= _SYSTEM_ADC_FIRE_SET_LIMIT )
			 {
				 if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_OK 		)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_BREAK ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  = _SYSTEM_SENSOR_FIRE 	; 
							
						 
							 _SYSTEM_FIRE_LED_ON	
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_SENSOR2_LED_OFF	
							 _SYSTEM_RELAY_DRIVE_ON
							 _SYSTEM_SOLENOID2_DRIVE_ON
							 _SYSTEM_HOOTER_DRIVE_ON
								
								stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET	= 
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 	= _DNumber_90 ;
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 	= _SYSTEM_BUZZER_MODE_HIGH_BEEP ;	
								#ifdef _DEBUGG_USART_ENABLE							 
												FnUsartPrint("FIRE CAUSED BY SENSOR 2\n\r");
								#endif
							
								for (uint8_t ucLoop = _DNumber_0 ; 
								ucLoop < _DNumber_80 ; ucLoop++ )
								FnReadAndAvgTheAdcValue();
					  }
			 }				 
	 else
		 if(  stVarADCValue_ALL.uiSensor2Value <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			 {
				 if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_OK 	)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status == _SYSTEM_SENSOR_FIRE ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  = _SYSTEM_SENSOR_BREAK ;
							
							
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_SENSOR2_LED_ON
									
							  stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 	= _DNumber_90 ;
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode   = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;
							 #ifdef _DEBUGG_USART_ENABLE							 
											 FnUsartPrint("SENSOR 2 BREAK\n\r");
							 #endif
						}							
			 }


		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiSolenoid1Value >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT)
		 &&(	stVarADCValue_ALL.uiSolenoid1Value <= (uint16_t)4095/* _SYSTEM_ADC_NORMAL_UPPER_LIMIT */ ))
			 {
					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_ON 		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_BREAK ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  = _SYSTEM_SOLENOID_OK ;				 					
									#ifdef _DEBUGG_USART_ENABLE							 
													FnUsartPrint("SOLENOID 1 FIXED!\n\r");
									#endif	
							}
								if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
										{ _SYSTEM_STATUS_LED_ON 	}
											_SYSTEM_SOLENOID1_LED_OFF					
			 }				 
	 else
		 if(  stVarADCValue_ALL.uiSolenoid1Value <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			 {
				 
					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_OK		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status == _SYSTEM_SOLENOID_ON	 ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  = _SYSTEM_SOLENOID_BREAK ;				 					
									 #ifdef _DEBUGG_USART_ENABLE							 
											 FnUsartPrint("SOLENOID 1 BREAK\n\r");
									 #endif
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_SOLENOID1_LED_ON
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt 	= _DNumber_90 ;
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode  = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;		
							}					 		
			 }		
			 
			 
		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiSolenoid2Value >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT)
		 &&(	stVarADCValue_ALL.uiSolenoid2Value <= (uint16_t)4095/* _SYSTEM_ADC_NORMAL_UPPER_LIMIT */ ))
			 {
					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_ON 		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_BREAK ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  = _SYSTEM_SOLENOID_OK ;				 					
									#ifdef _DEBUGG_USART_ENABLE							 
													FnUsartPrint("SOLENOID 2 FIXED!\n\r");
									#endif	
							}
						if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
								{ _SYSTEM_STATUS_LED_ON }
								 _SYSTEM_SOLENOID2_LED_OFF				
			 }			 
	 else
		 if(  stVarADCValue_ALL.uiSolenoid2Value <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			 {
				  
				if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_OK		)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status == _SYSTEM_SOLENOID_ON	 ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  = _SYSTEM_SOLENOID_BREAK ;				 					
								 #ifdef _DEBUGG_USART_ENABLE							 
										 FnUsartPrint("SOLENOID 2 BREAK\n\r");
								 #endif
							_SYSTEM_STATUS_LED_OFF
							_SYSTEM_SOLENOID2_LED_ON
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt 	  	 = _DNumber_90 ;
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode   		 = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;									
						}					 
			 }			 

			 
		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiHooterValue >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT )
		 &&(	stVarADCValue_ALL.uiHooterValue <=  (uint16_t)4095/* _SYSTEM_ADC_NORMAL_UPPER_LIMIT */ ))
			 {
					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus == _SYSTEM_SOLENOID_ON 		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus == _SYSTEM_SOLENOID_BREAK ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus  = _SYSTEM_SOLENOID_OK ;				 					
									#ifdef _DEBUGG_USART_ENABLE							 
													FnUsartPrint("HOOTER FIXED!\n\r");
									#endif	
							}
								if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
									&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
										{ _SYSTEM_STATUS_LED_ON 		}
										 _SYSTEM_HOOTER_LED_OFF				
			 }				 
	 else
		 if(  stVarADCValue_ALL.uiHooterValue <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			 {
					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus == _SYSTEM_SOLENOID_OK		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus == _SYSTEM_SOLENOID_ON	 ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus  = _SYSTEM_SOLENOID_BREAK ;				 					
									 #ifdef _DEBUGG_USART_ENABLE							 
											 FnUsartPrint("HOOTER BREAK\n\r");
									 #endif
								_SYSTEM_STATUS_LED_OFF
								_SYSTEM_HOOTER_LED_ON
							   stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt 	 = _DNumber_90 ;
							   stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 	 = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;									
							}					 
			 }			 
			 
			 		/**************************************************************************/
	  if((	stVarADCValue_ALL.uiExternalSensorValue >=  _SYSTEM_ADC_NORMAL_LOWER_LIMIT )
		 &&(	stVarADCValue_ALL.uiExternalSensorValue <=  _SYSTEM_ADC_NORMAL_UPPER_LIMIT ))
			 {

					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_ON 		)
						||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_BREAK ))	
							{ stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus  = _SYSTEM_SOLENOID_OK ;				 					
									#ifdef _DEBUGG_USART_ENABLE							 
													FnUsartPrint("EXT SW RELEASED!\n\r");
									#endif	
									_SYSTEM_MANUAL_LED_OFF
									stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwFault	 =  RESET ;					
							}	
			 }
			 		
		else
	  if( stVarADCValue_ALL.uiExternalSensorValue >=  _SYSTEM_ADC_NORMAL_UPPER_LIMIT )
		  {
				
				if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
						 { 
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  		= _SYSTEM_SENSOR_FIRE ;	
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  		= _SYSTEM_SENSOR_FIRE ;	
							 
							 _SYSTEM_FIRE_LED_ON	
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_MANUAL_LED_ON			
							 
							 _SYSTEM_RELAY_DRIVE_ON
							 _SYSTEM_SOLENOID1_DRIVE_ON
							 _SYSTEM_SOLENOID2_DRIVE_ON
							 _SYSTEM_HOOTER_DRIVE_ON
	
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;
								#ifdef _DEBUGG_USART_ENABLE							 
												FnUsartPrint("FIRE CAUSED BY ETXERNAL SWITCH\n\r");
								#endif
						 } 		
								//stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwFault = 
								stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus    = SET ;
						 
								stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount		 =		
								stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceStatus = RESET ;
			 } 			
			 
	 else
		if(  stVarADCValue_ALL.uiExternalSensorValue <=  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT )
			{
				 
				if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK		)
					||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_ON	 ))	
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus  = _SYSTEM_SOLENOID_BREAK ;				 					
								 #ifdef _DEBUGG_USART_ENABLE							 
										 FnUsartPrint("EXTERNAL SWITCH BREAK\n\r");
								 #endif
							
								_SYSTEM_STATUS_LED_OFF
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwFault	 =  SET ;	
							   stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt 			 = _DNumber_90 ;
							   stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 			 = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;									
							}					 
			}				 
			 
			 
}

/*****************************************************************************
 **@Function 		  	: 	FnBuzzerStatus
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnFireProcessStatus (void)
{
	
	if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt 	 != _DNumber_0 )
		 { stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt		 --;
			 
			if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt == _DNumber_0 )
				 {
					  	
					  stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode = _SYSTEM_BUZZER_MODE_OFF ;
					  #ifdef _DEBUGG_USART_ENABLE	
										FnUsartPrint("TURNING OFF THE BUZZER!\n\r");	
					  #endif
				 }	
		 }
		 
		 
	if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET != _DNumber_0 )
		 { stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET --;
		 
			 if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET == _DNumber_0 )
					{
					  #ifdef _DEBUGG_USART_ENABLE	
										FnUsartPrint("TURNING OFF THE SOLENOID & HOOTER!\n\r");	
					  #endif
						if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET == RESET )	
							&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET == RESET))
								{ 
									_SYSTEM_HOOTER_DRIVE_OFF
									_SYSTEM_RELAY_DRIVE_OFF
								}
							_SYSTEM_SOLENOID1_DRIVE_OFF
				 }	
		 }

		 
	if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET != _DNumber_0 )
		 { stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET --;
		 
			 if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET == _DNumber_0 )
					{
					  #ifdef _DEBUGG_USART_ENABLE	
										FnUsartPrint("TURNING OFF THE SOLENOID & HOOTER!\n\r");	
					  #endif
					if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1RESET == RESET )	
						&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2RESET == RESET))
							 { 
								 _SYSTEM_HOOTER_DRIVE_OFF
							   _SYSTEM_RELAY_DRIVE_OFF
							 }
						_SYSTEM_SOLENOID2_DRIVE_OFF
				 }	
		 }

	if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwFault == SET )
		 {
			 _SYSTEM_MANUAL_LED_TOGGLE
		 }		 
}



/*****************************************************************************
 **@Function 		  	: 	fnBuzzerAlarm
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnProcessUSERButtons (void)
{
				
	
					stVarSYSTEM_STATUS_ALL.ucSYSTEMManualPin  =  _SYSTEM_BUTTON_SILENCE_READ ;
		 if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMManualPin == 	SET )
				{	
					
					if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceEvent != SET )	
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceEvent  = SET ; 
					
							 //sprintf(ucDebugBuff,"MAN CNT = %d\n\r",stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount);
							 //FnUsartPrint(ucDebugBuff);
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount ++ ;
				 if (  stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount >= _SYSTEM_MANUAL_SET_TIME )
						{  stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus =   SET ;
						}				
				}
			else
				{
					
				 if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceEvent  ==   SET )
						{ stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceEvent		= RESET ;
							stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceStatus  =   SET ;
						}
				}
						
					
	
		 if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMManualPin     ==  RESET   )
		 &&(( stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceStatus ==  	 SET   )
		 ||(( stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus  == 	 SET   )
		 && ( stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount		 >= _SYSTEM_MANUAL_SET_TIME  ))))
			{	  
			  			 
						 sprintf( ucDebugBuff ,"MAN/SIL = %u\n\r ",
						 stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus);
					#ifdef _DEBUGG_USART_ENABLE	
									FnUsartPrint(ucDebugBuff);	
					#endif	

					if((	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	 	==  _SYSTEM_SENSOR_FIRE    )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status 	 	==  _SYSTEM_SENSOR_BREAK   )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	 	==  _SYSTEM_SENSOR_FIRE    )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  	==  _SYSTEM_SENSOR_BREAK   )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  ==  _SYSTEM_SOLENOID_BREAK )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  ==  _SYSTEM_SOLENOID_BREAK )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus		  ==  _SYSTEM_SOLENOID_BREAK )
					 ||(	stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus ==  _SYSTEM_SOLENOID_BREAK ))					
						 {
							
								_SYSTEM_HOOTER_DRIVE_OFF
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	 		 = _DNumber_0 ;
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode 		 =  _SYSTEM_BUZZER_MODE_OFF ;
								#ifdef _DEBUGG_USART_ENABLE	
												FnUsartPrint("TURNING OFF THE BUZZER!\n\r");	
								#endif
						 
						 } 
						 
				 if(( stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount		 		>= _SYSTEM_MANUAL_SET_TIME  ) 
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
					&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK ))
						 { 
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  		= _SYSTEM_SENSOR_FIRE ;	
							 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  		= _SYSTEM_SENSOR_FIRE ;	
							 
							 _SYSTEM_FIRE_LED_ON	
							 _SYSTEM_STATUS_LED_OFF
							 _SYSTEM_MANUAL_LED_ON			
							 
							 _SYSTEM_RELAY_DRIVE_ON
							 _SYSTEM_SOLENOID1_DRIVE_ON
							 _SYSTEM_SOLENOID2_DRIVE_ON
							 _SYSTEM_HOOTER_DRIVE_ON
	
								stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode = _SYSTEM_BUZZER_MODE_HIGH_BEEP ;
								#ifdef _DEBUGG_USART_ENABLE							 
												FnUsartPrint("FIRE CAUSED BY MANUAL\n\r");
								#endif
						 } 					
			
					stVarSYSTEM_STATUS_ALL.ucSYSTEMManualCount		 =		
					stVarSYSTEM_STATUS_ALL.ucSYSTEMSilenceStatus = RESET ;
			}
			
					 stVarSYSTEM_STATUS_ALL.ucSYSTEMTestStatus = _SYSTEM_BUTTON_TEST_READ ;
			if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMTestStatus == SET )
				 {
					 
						if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMTestEvent != SET )	
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMTestEvent  = SET ; 
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMTestCnt++;
						
				 }
			else
				 {
						if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMTestEvent ==  SET )
							 { stVarSYSTEM_STATUS_ALL.ucSYSTEMTestEvent = RESET ;
								
								 if ( stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus ==  SET )
										{ stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus = RESET ;
											stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwFault = RESET ;
											
											stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status  = _SYSTEM_SENSOR_OK ;	
											stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status  = _SYSTEM_SENSOR_OK ;	
											
											_SYSTEM_FIRE_LED_OFF	
											_SYSTEM_STATUS_LED_ON
											_SYSTEM_MANUAL_LED_OFF
											_SYSTEM_RELAY_DRIVE_OFF	
											_SYSTEM_SOLENOID1_DRIVE_OFF
											_SYSTEM_SOLENOID2_DRIVE_OFF
											_SYSTEM_HOOTER_DRIVE_OFF
											
											
											 for (uint8_t ucLoop = _DNumber_0 ; 
																		ucLoop < _DNumber_80 ; ucLoop++ )
																		FnReadAndAvgTheAdcValue();
											
											stVarSYSTEM_STATUS_ALL.ucSYSTEMTestCnt	  =
											stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERCnt	= _DNumber_0 ;
											stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode = _SYSTEM_BUZZER_MODE_OFF ;
											#ifdef _DEBUGG_USART_ENABLE							 
															FnUsartPrint("RESET THE MANUAL FUNCTION!\n\r");
											#endif
										}
									else
										{
											
											if (( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status    == _SYSTEM_SENSOR_OK 	)	
												&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status 	  == _SYSTEM_SENSOR_OK 	)
												&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status  == _SYSTEM_SOLENOID_OK )
												&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status  == _SYSTEM_SOLENOID_OK )
												&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMHooterStatus 	  == _SYSTEM_SOLENOID_OK )
												&&( stVarSYSTEM_STATUS_ALL.ucSYSTEMExternalSwStatus == _SYSTEM_SOLENOID_OK )
											  &&(	stVarSYSTEM_STATUS_ALL.ucSYSTEMTestCnt	  			>= _SYSTEM_TEST_SET_TIME	))
													{
														
														#ifdef _DEBUGG_USART_ENABLE							 
																		FnUsartPrint("TEST CONDITION STARTED!\n\r");
														#endif
														
																_SYSTEM_BUZZER_OFF  			 _SYSTEM_SENSOR1_LED_OFF	 	_SYSTEM_SENSOR2_LED_OFF	
																_SYSTEM_SOLENOID1_LED_OFF	 _SYSTEM_SOLENOID2_LED_OFF  _SYSTEM_HOOTER_LED_OFF
																_SYSTEM_FIRE_LED_OFF			 _SYSTEM_MANUAL_LED_OFF			_SYSTEM_STATUS_LED_OFF
														
																
															for ( uint8_t ucLoop1 = RESET ; ucLoop1 < _DNumber_6 ; ucLoop1++  )	
																	{
																		
																		_SYSTEM_SENSOR1_LED_TOGGLE
																		_SYSTEM_SENSOR2_LED_TOGGLE
																		_SYSTEM_SOLENOID1_LED_TOGGLE
																		_SYSTEM_SOLENOID2_LED_TOGGLE
																		_SYSTEM_HOOTER_LED_TOGGLE
																		_SYSTEM_FIRE_LED_TOGGLE
																		_SYSTEM_MANUAL_LED_TOGGLE
																		_SYSTEM_STATUS_LED_OFF
																		 FnDelay_25ms(_DNumber_40);
																	
																	}

																/**************************************************/	
																_SYSTEM_BUZZER_OFF  			 _SYSTEM_SENSOR1_LED_OFF	 	_SYSTEM_SENSOR2_LED_OFF	
																_SYSTEM_SOLENOID1_LED_OFF	 _SYSTEM_SOLENOID2_LED_OFF  _SYSTEM_HOOTER_LED_OFF
																_SYSTEM_FIRE_LED_OFF			 _SYSTEM_MANUAL_LED_OFF			_SYSTEM_STATUS_LED_OFF	

																	
																/**************************************************/																		
																_SYSTEM_HOOTER_DRIVE_ON
																_SYSTEM_BUZZER_ON 
																 FnDelay_25ms(_DNumber_80 + _DNumber_40);																 	
																_SYSTEM_HOOTER_DRIVE_OFF
																_SYSTEM_BUZZER_OFF	

														
														#ifdef _DEBUGG_USART_ENABLE							 
																		FnUsartPrint("TEST CONDITION COMPLETED!\n\r");
														#endif																	
													}
													
											stVarSYSTEM_STATUS_ALL.ucSYSTEMTestCnt = RESET ;				
										}										
							 }							
				 }									
}


/*****************************************************************************
 **@Function 		  	: 	fnBuzzerAlarm
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void fnBuzzerAlarm (uint8_t ucHooterSelecMode)
{

	static uint8_t ucHooterCnt 			 = _DNumber_0;
	static uint8_t ucHooterLastState = _DNumber_0;
	
		switch (ucHooterSelecMode)
		{
			
			case _DNumber_0 :
				 if (ucHooterSelecMode != ucHooterLastState)
						{
							_SYSTEM_BUZZER_OFF
							ucHooterLastState = ucHooterSelecMode;	
						}
						break;
						
			case _DNumber_1 :
				 if (ucHooterSelecMode != ucHooterLastState)
						{
							_SYSTEM_BUZZER_ON
							ucHooterLastState = ucHooterSelecMode;	
						}
						break;
						
			case _DNumber_2 :
						if (ucHooterSelecMode != ucHooterLastState)
						{
								ucHooterCnt = _DNumber_0 ;
								ucHooterLastState = ucHooterSelecMode;
						}
						if( ucHooterCnt == _DNumber_4)
							{ ucHooterCnt = _DNumber_0 ;
								_SYSTEM_BUZZER_TOGGLE
							} ucHooterCnt++;
					  break;
							
			case _DNumber_3 :
						if (ucHooterSelecMode != ucHooterLastState)
						{
								ucHooterCnt = _DNumber_0 ;
								ucHooterLastState = ucHooterSelecMode;
						}
						if( ucHooterCnt == _DNumber_40)
							{ ucHooterCnt = _DNumber_0 ;
								_SYSTEM_BUZZER_TOGGLE
							} ucHooterCnt++;
					  break;					
				
			default :
				 _SYSTEM_BUZZER_OFF
		}
}


/*****************************************************************************
 **@Function 		  	: 	FnTimerFunction
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
void FnTimerFunction(void)
{	
				 stVarTime_All.uc12_5msFlag = SET ;	
				 stVarTime_All.uc12_5msCnt++			;
	
		if ( stVarTime_All.uc12_5msCnt == _DNumber_2 )
			 { stVarTime_All.uc12_5msCnt  = _DNumber_0 ;
				 stVarTime_All.uc25msFlag = SET ;
				 stVarTime_All.uc25msCnt++;
			 }
		if ( stVarTime_All.uc25msCnt  ==  _DNumber_4 )
			 { stVarTime_All.uc25msCnt   =  _DNumber_0 ;  
				 stVarTime_All.uc100msCnt++;	
				 stVarTime_All.uc100msFlag = SET ;
			 }
		if ( stVarTime_All.uc100msCnt == _DNumber_5  )
			 { stVarTime_All.uc100msCnt  = _DNumber_0  ; 
				 stVarTime_All.uc500msCnt++;
				 stVarTime_All.uc500msFlag = SET ;
			 }
		if ( stVarTime_All.uc500msCnt	== _DNumber_2	 ) 	 
			 { stVarTime_All.uc500msCnt  = _DNumber_0  ; 
				 stVarTime_All.uc1sCnt++;
				 stVarTime_All.uc1sFlag = SET ;
			 }
		if ( stVarTime_All.uc1sCnt		== _DNumber_60 ) 	 
			 { stVarTime_All.uc1sCnt  	 = _DNumber_0  ; 
				 stVarTime_All.uc1MCnt++;
				 
			 }			 
		if ( stVarTime_All.uc1MCnt		== _DNumber_60 ) 	 
			 { stVarTime_All.uc1MCnt  	 = _DNumber_0  ; 
				 stVarTime_All.uc1HCnt++;
				 
			 }			
		if ( stVarTime_All.uc1HCnt		== _DNumber_24 ) 	 
			 { 
				 stVarTime_All.uc12_5msCnt=
				 stVarTime_All.uc25msCnt  =  stVarTime_All.uc100msCnt = 
				 stVarTime_All.uc500msCnt =  stVarTime_All.uc1sCnt 	  = 	
				 stVarTime_All.uc1MCnt 	  =  stVarTime_All.uc1HCnt 	  = _DNumber_0 ;
			 }				 
}


