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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 *                           INCLUDES
 * ----------------------------------------------------------------------------
*/ 

		#include "stm32g0xx_it.h"		
		#include "stdint.h"
		#include "SystemInit.h"
	  #include "stm32g0xx.h"
		
		
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* ----------------------------------------------------------------------------
 *                           EXTERNAL VARIABLES
 * ----------------------------------------------------------------------------
*/



	 extern char  	 ucDebugBuff [ _DNumber_100 * _DNumber_2];	
	 
	 extern uint8_t  ucADCTotalChannel ;
	 extern uint8_t  ucADCChannelNo 	 ;
	 extern uint8_t  ucMAXAdcSample 	 ;
	 extern uint8_t  ucADCVAvgNo 			 ;	
	 
	 extern int32_t  ucADCRet 				 ;
	 extern uint32_t ucADCSampleTotal  ;
	 extern uint16_t ucADCBUFF[_DNumber_6][_DNumber_6];	 
	 extern uint16_t uiAdcAvgValue[_DNumber_6];	


	 extern stSYSTEM_STATUS_ALL  stVarSYSTEM_STATUS_ALL;
	 extern stTime_All					 stVarTime_All	  		 ;	
	 extern stADCValue_ALL			 stVarADCValue_ALL		 ;		 



/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

