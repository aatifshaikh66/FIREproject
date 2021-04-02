/**
  ******************************************************************************
  * @file    Project/STM32G0xx_StdPeriph_Template/main.c 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H
 
/* ----------------------------------------------------------------------------
 *                           INCLUDES
 * ----------------------------------------------------------------------------
*/ 

		#include "stdint.h"
 
 
/* ----------------------------------------------------------------------------
 *                           MACROS
 * ----------------------------------------------------------------------------
*/

	 #define _DEBUGG_USART_NO							USART1
	 #define _DEBUGG_USART_BAUD_RATE		 (uint32_t)115200
	 #define _DEBUGG_USART_TX_PORT				GPIOA
	 #define _DEBUGG_USART_RX_PORT				GPIOA
	 #define _DEBUGG_USART_TX_PIN					GPIO_Pin_10
	 #define _DEBUGG_USART_RX_PIN					GPIO_Pin_9	 
//   #define _DEBUGG_USART_ENABLE					SET


	 #define _SYSTEM_BUZZER_PORT					GPIOA
	 #define _SYSTEM_BUZZER_PIN						GPIO_Pin_8
	 #define _SYSTEM_BUZZER_ON						GPIO_SetBits   (_SYSTEM_BUZZER_PORT, _SYSTEM_BUZZER_PIN);
	 #define _SYSTEM_BUZZER_OFF						GPIO_ResetBits (_SYSTEM_BUZZER_PORT, _SYSTEM_BUZZER_PIN);
	 #define _SYSTEM_BUZZER_TOGGLE				GPIO_ToggleBits(_SYSTEM_BUZZER_PORT, _SYSTEM_BUZZER_PIN);	 
	 
	 #define _SYSTEM_SENSOR1_LED_PORT			GPIOB
	 #define _SYSTEM_SENSOR1_LED_PIN			GPIO_Pin_8
	 #define _SYSTEM_SENSOR1_LED_ON				GPIO_SetBits   (_SYSTEM_SENSOR1_LED_PORT, _SYSTEM_SENSOR1_LED_PIN);
	 #define _SYSTEM_SENSOR1_LED_OFF			GPIO_ResetBits (_SYSTEM_SENSOR1_LED_PORT, _SYSTEM_SENSOR1_LED_PIN);
	 #define _SYSTEM_SENSOR1_LED_TOGGLE		GPIO_ToggleBits(_SYSTEM_SENSOR1_LED_PORT, _SYSTEM_SENSOR1_LED_PIN);

	 #define _SYSTEM_SENSOR2_LED_PORT			GPIOB
	 #define _SYSTEM_SENSOR2_LED_PIN			GPIO_Pin_9
	 #define _SYSTEM_SENSOR2_LED_ON				GPIO_SetBits   (_SYSTEM_SENSOR2_LED_PORT, _SYSTEM_SENSOR2_LED_PIN);
	 #define _SYSTEM_SENSOR2_LED_OFF			GPIO_ResetBits (_SYSTEM_SENSOR2_LED_PORT, _SYSTEM_SENSOR2_LED_PIN);
	 #define _SYSTEM_SENSOR2_LED_TOGGLE		GPIO_ToggleBits(_SYSTEM_SENSOR2_LED_PORT, _SYSTEM_SENSOR2_LED_PIN);	 

	 #define _SYSTEM_SOLENOID1_LED_PORT		GPIOB
	 #define _SYSTEM_SOLENOID1_LED_PIN		GPIO_Pin_7
	 #define _SYSTEM_SOLENOID1_LED_ON			GPIO_SetBits   (_SYSTEM_SOLENOID1_LED_PORT, _SYSTEM_SOLENOID1_LED_PIN);
	 #define _SYSTEM_SOLENOID1_LED_OFF		GPIO_ResetBits (_SYSTEM_SOLENOID1_LED_PORT, _SYSTEM_SOLENOID1_LED_PIN);
	 #define _SYSTEM_SOLENOID1_LED_TOGGLE	GPIO_ToggleBits(_SYSTEM_SOLENOID1_LED_PORT, _SYSTEM_SOLENOID1_LED_PIN); 

	 #define _SYSTEM_SOLENOID2_LED_PORT		GPIOB
	 #define _SYSTEM_SOLENOID2_LED_PIN		GPIO_Pin_6
	 #define _SYSTEM_SOLENOID2_LED_ON			GPIO_SetBits   (_SYSTEM_SOLENOID2_LED_PORT, _SYSTEM_SOLENOID2_LED_PIN);
	 #define _SYSTEM_SOLENOID2_LED_OFF		GPIO_ResetBits (_SYSTEM_SOLENOID2_LED_PORT, _SYSTEM_SOLENOID2_LED_PIN);
	 #define _SYSTEM_SOLENOID2_LED_TOGGLE	GPIO_ToggleBits(_SYSTEM_SOLENOID2_LED_PORT, _SYSTEM_SOLENOID2_LED_PIN); 

	 #define _SYSTEM_HOOTER_LED_PORT			GPIOB
	 #define _SYSTEM_HOOTER_LED_PIN				GPIO_Pin_5
	 #define _SYSTEM_HOOTER_LED_ON				GPIO_SetBits   (_SYSTEM_HOOTER_LED_PORT, _SYSTEM_HOOTER_LED_PIN);
	 #define _SYSTEM_HOOTER_LED_OFF				GPIO_ResetBits (_SYSTEM_HOOTER_LED_PORT, _SYSTEM_HOOTER_LED_PIN);
	 #define _SYSTEM_HOOTER_LED_TOGGLE		GPIO_ToggleBits(_SYSTEM_HOOTER_LED_PORT, _SYSTEM_HOOTER_LED_PIN); 

	 #define _SYSTEM_FIRE_LED_PORT				GPIOB
	 #define _SYSTEM_FIRE_LED_PIN					GPIO_Pin_4
	 #define _SYSTEM_FIRE_LED_ON					GPIO_SetBits   (_SYSTEM_FIRE_LED_PORT, _SYSTEM_FIRE_LED_PIN);
	 #define _SYSTEM_FIRE_LED_OFF					GPIO_ResetBits (_SYSTEM_FIRE_LED_PORT, _SYSTEM_FIRE_LED_PIN);
	 #define _SYSTEM_FIRE_LED_TOGGLE			GPIO_ToggleBits(_SYSTEM_FIRE_LED_PORT, _SYSTEM_FIRE_LED_PIN); 

	 #define _SYSTEM_MANUAL_LED_PORT			GPIOB
	 #define _SYSTEM_MANUAL_LED_PIN				GPIO_Pin_3
	 #define _SYSTEM_MANUAL_LED_ON				GPIO_SetBits   (_SYSTEM_MANUAL_LED_PORT, _SYSTEM_MANUAL_LED_PIN);
	 #define _SYSTEM_MANUAL_LED_OFF				GPIO_ResetBits (_SYSTEM_MANUAL_LED_PORT, _SYSTEM_MANUAL_LED_PIN);
	 #define _SYSTEM_MANUAL_LED_TOGGLE		GPIO_ToggleBits(_SYSTEM_MANUAL_LED_PORT, _SYSTEM_MANUAL_LED_PIN);

	 #define _SYSTEM_STATUS_LED_PORT			GPIOA
	 #define _SYSTEM_STATUS_LED_PIN				GPIO_Pin_15
	 #define _SYSTEM_STATUS_LED_ON				GPIO_SetBits   (_SYSTEM_STATUS_LED_PORT, _SYSTEM_STATUS_LED_PIN);
	 #define _SYSTEM_STATUS_LED_OFF				GPIO_ResetBits (_SYSTEM_STATUS_LED_PORT, _SYSTEM_STATUS_LED_PIN);
	 #define _SYSTEM_STATUS_LED_TOGGLE		GPIO_ToggleBits(_SYSTEM_STATUS_LED_PORT, _SYSTEM_STATUS_LED_PIN);

	 #define _SYSTEM_SOLENOID1_DRIVE_PORT	GPIOA
	 #define _SYSTEM_SOLENOID1_DRIVE_PIN	GPIO_Pin_6
	 #define _SYSTEM_SOLENOID1_DRIVE_ON		GPIO_SetBits  (_SYSTEM_SOLENOID1_DRIVE_PORT, _SYSTEM_SOLENOID1_DRIVE_PIN);
	 #define _SYSTEM_SOLENOID1_DRIVE_OFF	GPIO_ResetBits(_SYSTEM_SOLENOID1_DRIVE_PORT, _SYSTEM_SOLENOID1_DRIVE_PIN);

	 #define _SYSTEM_SOLENOID2_DRIVE_PORT	GPIOA
	 #define _SYSTEM_SOLENOID2_DRIVE_PIN	GPIO_Pin_7
	 #define _SYSTEM_SOLENOID2_DRIVE_ON		GPIO_SetBits  (_SYSTEM_SOLENOID2_DRIVE_PORT, _SYSTEM_SOLENOID2_DRIVE_PIN);
	 #define _SYSTEM_SOLENOID2_DRIVE_OFF	GPIO_ResetBits(_SYSTEM_SOLENOID2_DRIVE_PORT, _SYSTEM_SOLENOID2_DRIVE_PIN);

	 #define _SYSTEM_HOOTER_DRIVE_PORT		GPIOB
	 #define _SYSTEM_HOOTER_DRIVE_PIN			GPIO_Pin_0
	 #define _SYSTEM_HOOTER_DRIVE_ON			GPIO_SetBits  (_SYSTEM_HOOTER_DRIVE_PORT, _SYSTEM_HOOTER_DRIVE_PIN);
	 #define _SYSTEM_HOOTER_DRIVE_OFF			GPIO_ResetBits(_SYSTEM_HOOTER_DRIVE_PORT, _SYSTEM_HOOTER_DRIVE_PIN);

	 #define _SYSTEM_RELAY_DRIVE_PORT			GPIOB
	 #define _SYSTEM_RELAY_DRIVE_PIN			GPIO_Pin_1
	 #define _SYSTEM_RELAY_DRIVE_ON				GPIO_SetBits  (_SYSTEM_RELAY_DRIVE_PORT, _SYSTEM_RELAY_DRIVE_PIN );
	 #define _SYSTEM_RELAY_DRIVE_OFF			GPIO_ResetBits(_SYSTEM_RELAY_DRIVE_PORT, _SYSTEM_RELAY_DRIVE_PIN );
	 

	 #define _SYSTEM_BUTTON_TEST_PORT			GPIOC
	 #define _SYSTEM_BUTTON_TEST_PIN			GPIO_Pin_6
	 #define _SYSTEM_BUTTON_TEST_READ			!GPIO_ReadInputDataBit (_SYSTEM_BUTTON_TEST_PORT, _SYSTEM_BUTTON_TEST_PIN);


	 #define _SYSTEM_BUTTON_SILENCE_PORT	GPIOB
	 #define _SYSTEM_BUTTON_SILENCE_PIN		GPIO_Pin_2
	 #define _SYSTEM_BUTTON_SILENCE_READ	!GPIO_ReadInputDataBit (_SYSTEM_BUTTON_SILENCE_PORT, _SYSTEM_BUTTON_SILENCE_PIN)



	 #define  _SYSTEM_SENSOR_OK									_DNumber_0
	 #define  _SYSTEM_SENSOR_FIRE								_DNumber_1
	 #define  _SYSTEM_SENSOR_BREAK							_DNumber_2	 
	 
	 
	 #define  _SYSTEM_BUZZER_MODE_OFF						_DNumber_0
	 #define  _SYSTEM_BUZZER_MODE_ON						_DNumber_1	 
	 #define  _SYSTEM_BUZZER_MODE_HIGH_BEEP			_DNumber_2
	 #define  _SYSTEM_BUZZER_MODE_LOW_BEEP			_DNumber_3	

	 #define  _SYSTEM_MANUAL_SET_TIME	  		  	_DNumber_80
	 #define  _SYSTEM_TEST_SET_TIME	 		 		  	_DNumber_80

	 #define  _SYSTEM_SOLENOID_OK							  _DNumber_0		
	 #define  _SYSTEM_SOLENOID_ON								_DNumber_1
	 #define  _SYSTEM_SOLENOID_BREAK	          _DNumber_2
	 

	 #define  _SYSTEM_ADC_FIRE_SET_LIMIT				 (uint16_t)2000		

	 #define  _SYSTEM_ADC_NORMAL_UPPER_LIMIT		 (uint16_t)1600	

   #define  _SYSTEM_ADC_NORMAL_LOWER_LIMIT		 (uint16_t)1100		

	 #define  _SYSTEM_ADC_CHANNEL_BREAK_LIMIT		 (uint8_t )15


	 #define  _SYSTEM_ADC_NUMBER_SAMPLES				 (uint8_t)0x02	


// #define  _SYSTEM_ADC_ACTIVE_UPPER_LIMIT 		 (uint8_t )400 	
// #define  _SYSTEM_ADC_ACTIVE_LOWER_LIMIT 		 (uint8_t )20 	

	 	

/* ----------------------------------------------------------------------------
 *                           STRUCTURE
 * ----------------------------------------------------------------------------
*/


		typedef struct
		{
			
			
			uint8_t  ucSYSTEMManualPin	  	  ;						
			uint8_t  ucSYSTEMManualStatus  	  ;			
      uint16_t ucSYSTEMManualCount 		  ;		
			uint8_t	 ucSYSTEMSilenceEvent			;	
			uint8_t	 ucSYSTEMSilenceStatus		;
			
			uint8_t	 ucSYSTEMTestEvent				;	
			uint8_t  ucSYSTEMTestStatus	   	  ;			
      uint8_t  ucSYSTEMTestCnt 			 	  ;	
			
			uint8_t  ucSYSTEMSolenoid1Status  ;	
			uint8_t  ucSYSTEMSolenoid2Status  ;	
			uint8_t  ucSYSTEMHooterStatus  		;
			
			uint8_t  ucSYSTEMSensor1Status 	  ;
			uint8_t  ucSYSTEMSensor1RESET 	  ;

			uint8_t  ucSYSTEMSensor2Status	  ;	
			uint8_t  ucSYSTEMSensor2RESET 	  ;			
			
			uint8_t  ucSYSTEMExternalSwStatus	;
			uint8_t  ucSYSTEMExternalSwFault	;			
			
			uint8_t  ucSYSTEMBUZZERCnt				;
			uint8_t	 ucSYSTEMBUZZERMode				;
			
		}stSYSTEM_STATUS_ALL;

		
		typedef struct
		{
			
			uint8_t	uc12_5msFlag;
			uint8_t uc25msFlag 	;
			uint8_t uc100msFlag ;
			uint8_t uc500msFlag ;			
			uint8_t uc1sFlag 		;

			

			uint8_t	uc12_5msCnt ;
			uint8_t	uc25msCnt  	;
			uint8_t	uc100msCnt 	;
			uint8_t	uc500msCnt  ;
			uint8_t	uc1sCnt 	  ;	
			uint8_t	uc1MCnt 	  ; 
			uint8_t	uc1HCnt 	  ;
				
			
		}stTime_All;			


	typedef struct
	{
	
		uint16_t uiSensor1Value 	;		
		uint16_t uiSensor2Value 	;		
		uint16_t uiSolenoid1Value ;		
		uint16_t uiSolenoid2Value ;		
		uint16_t uiHooterValue		;
		uint16_t uiExternalSensorValue 	;		
	
	}stADCValue_ALL;		
		

	
		
/* ----------------------------------------------------------------------------
 *                           FUNCTION DECLAIRATION
 * ----------------------------------------------------------------------------
*/

	
		extern void FnALLInit       ( void );
		extern void FnUsartPrint		( char *ucData);
		extern void FnTimerFunction	( void );
		extern void fnBuzzerAlarm	  ( uint8_t ucHooterSelecMode);
		extern void FnReadAndAvgTheAdcValue( void );
		extern void FnCheckProcessADC( void );
		extern void FnFireProcessStatus ( void );
		extern void FnProcessUSERButtons ( void );
	
	
#endif

