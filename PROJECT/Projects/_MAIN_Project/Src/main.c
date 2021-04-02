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
  
	/*
			KABOOOOOOM...........!
		
	          _ ._  _ , _ ._
        (_ ' ( `  )_  .__)
      ( (  (    )   `)  ) _)
     (__ (_   (_ . _) _) ,__)
         `~~`\ ' . /`~~`
              ;   ;
              /   \
_____________/_ __ \_____________
	
	*/
  
/* ----------------------------------------------------------------------------
 *                           Includes
 * ----------------------------------------------------------------------------
 */
	 

		#include "main.h"
		
		
/* ----------------------------------------------------------------------------
 *                           MACROS
 * ----------------------------------------------------------------------------
 */
 

/* ----------------------------------------------------------------------------
 *                           GLOBAL VARIABLES
 * ----------------------------------------------------------------------------
 */
 
 
 
	 char	 		ucDebugBuff [ _DNumber_100 * _DNumber_2];	


	 uint8_t  ucMAXAdcSample 		= _SYSTEM_ADC_NUMBER_SAMPLES ;
	 uint8_t  ucADCTotalChannel = _DNumber_6 ; 
	 uint8_t  ucADCChannelNo 	  = _DNumber_1 ;
	 uint8_t  ucADCVAvgNo 			= _DNumber_0 ;	
	 uint32_t ucADCSampleTotal  = _DNumber_0 ;
	 uint16_t ucADCBUFF[_DNumber_6][_DNumber_6];
	 int32_t 	ucADCRet 					= _DNumber_0 ;
	 uint16_t uiAdcAvgValue[_DNumber_6]		 	 ;	


	 stSYSTEM_STATUS_ALL  stVarSYSTEM_STATUS_ALL;
	 stTime_All						stVarTime_All					;	
	 stADCValue_ALL				stVarADCValue_ALL			;	


 
/* ----------------------------------------------------------------------------
 *                           MAIN FUNCTION
 * ----------------------------------------------------------------------------
*/



/*****************************************************************************
 **@Function 		  	: 	main
 **@Descriptions		: 	
 **@parameters			: 	None
 **@return					: 	None
*****************************************************************************/
int main(void)
{
	
	
	/*************COMPLETE SYSTEM INIT************/
	 FnALLInit();

	
  /* Infinite loop */
  while (_DNumber_1)
  {
		
		
		/********************************************************/
		if ( stVarTime_All.uc12_5msFlag ==   SET )
			 { stVarTime_All.uc12_5msFlag  = RESET ;
				 
				 /*********************/
				 FnReadAndAvgTheAdcValue  (  );

				 /*********************/				 
				 FnProcessUSERButtons	( 	);
				 
			 }		
			 
		/********************************************************/	 			 
		if ( stVarTime_All.uc100msFlag  ==  SET )
			 { stVarTime_All.uc100msFlag	= RESET ;
				 
				 /*********************/
				 if (	stVarSYSTEM_STATUS_ALL.ucSYSTEMManualStatus == RESET )
		 					FnCheckProcessADC();
				 
				 /*********************/
				 fnBuzzerAlarm(stVarSYSTEM_STATUS_ALL.ucSYSTEMBUZZERMode);
				 
			 }			
			 
		/********************************************************/	 
		if ( stVarTime_All.uc1sFlag	==	 SET )	 
			 { stVarTime_All.uc1sFlag	 = RESET ;

				 
				 #ifdef _DEBUGG_USART_ENABLE
								 sprintf(ucDebugBuff,"T = %u.%u.%u SN1=%u %u SN2=%u %u SL1=%u %u SL2=%u %u HOT=%u EXT=%u \n\r",
								 stVarTime_All.uc1HCnt,stVarTime_All.uc1MCnt,stVarTime_All.uc1sCnt,
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor1Status, stVarADCValue_ALL.uiSensor1Value,
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMSensor2Status, stVarADCValue_ALL.uiSensor2Value,
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid1Status , stVarADCValue_ALL.uiSolenoid1Value , 
								 stVarSYSTEM_STATUS_ALL.ucSYSTEMSolenoid2Status , stVarADCValue_ALL.uiSolenoid2Value ,
															 stVarADCValue_ALL.uiHooterValue	, stVarADCValue_ALL.uiExternalSensorValue);
								 FnUsartPrint(ucDebugBuff);
				 #endif
				 
				 /*********************/
				 FnFireProcessStatus();
				 
			 }	 
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(_DNumber_1) 
  {
		
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
