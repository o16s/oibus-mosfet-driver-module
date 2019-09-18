/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "oi_uavcan.h"
#include "oi_driver.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartUAVCANTask(void const * argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(UAVCANTask, StartUAVCANTask, osPriorityHigh, 1, 256);  // define job1 as thread function
  osThreadCreate(osThread(UAVCANTask),NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  
  int64_t last_drivers_val = 0; //stores the last value in this parameter and updates it in case changed

  //get params 
  param_t* p_all = oi_uavcan_getParamByName("drivers");

  param_t* p_ch1 = oi_uavcan_getParamByName("ch1");
  param_t* p_ch1_period = oi_uavcan_getParamByName("ch1_period");
  param_t* p_ch1_toffset = oi_uavcan_getParamByName("ch1_toffset");
  param_t* p_ch2 = oi_uavcan_getParamByName("ch2");
  param_t* p_ch3 = oi_uavcan_getParamByName("ch3");
  param_t* p_ch4 = oi_uavcan_getParamByName("ch4");
  param_t* p_ch5 = oi_uavcan_getParamByName("ch5");
  param_t* p_ch6 = oi_uavcan_getParamByName("ch6");
  param_t* p_ch7 = oi_uavcan_getParamByName("ch7");
  param_t* p_ch8 = oi_uavcan_getParamByName("ch8");
  param_t* p_ch9 = oi_uavcan_getParamByName("ch9");
  param_t* p_ch10 = oi_uavcan_getParamByName("ch10");

  //timer
  int onTimestamp = 0;
  int offTimestamp = 0;
  int currentTimestamp = 0;
  int offsetCounter = 0;

  /* Infinite loop */
  for(;;)
  {
    if(p_all->val == last_drivers_val)
    {
      // The drivers parameter was not changed, i.e. the individual channels may have updates which will overwrite "drivers".
      last_drivers_val =  ((p_ch1->val) << 4) +
                          ((p_ch2->val) << 3) +
                          ((p_ch3->val) << 2) +
                          ((p_ch4->val) << 1) +
                          ((p_ch5->val)     ) +
                          ((p_ch6->val) << 5) +
                          ((p_ch7->val) << 6) +
                          ((p_ch8->val) << 7) +
                          ((p_ch9->val) << 8) +
                          ((p_ch10->val) << 9);
      p_all->val = last_drivers_val;
    }
    else
    {
      // the drivers parameter was updated, so we force update all the individual channels' parameters
      (p_ch1->val) = ((p_all->val) >> 4) & 1;
      (p_ch2->val) = ((p_all->val) >> 3) & 1;
      (p_ch3->val) = ((p_all->val) >> 2) & 1;
      (p_ch4->val) = ((p_all->val) >> 1) & 1;
      (p_ch5->val) = ((p_all->val))      & 1;
      (p_ch6->val) = ((p_all->val) >> 5) & 1;
      (p_ch7->val) = ((p_all->val) >> 6) & 1;
      (p_ch8->val) = ((p_all->val) >> 7) & 1;
      (p_ch9->val) = ((p_all->val) >> 8) & 1;
      (p_ch10->val) = ((p_all->val) >> 9) & 1;

      last_drivers_val = p_all->val;
    }
    
    oi_driver_set(GPIOB, In1_Pin, 0, p_all->val);
    oi_driver_set(GPIOB, In2_Pin, 1, p_all->val);
    oi_driver_set(GPIOA, In3_Pin, 2, p_all->val);
    oi_driver_set(GPIOA, In4_Pin, 3, p_all->val);
    oi_driver_set(GPIOA, In5_Pin, 4, p_all->val);
    oi_driver_set(GPIOA, In6_Pin, 5, p_all->val);
    oi_driver_set(GPIOA, In7_Pin, 6, p_all->val);
    oi_driver_set(GPIOC, In8_Pin, 7, p_all->val);
    oi_driver_set(GPIOB, In9_Pin, 8, p_all->val);
    oi_driver_set(GPIOA, In10_Pin, 9, p_all->val);

    if(p_ch1_toffset->val > 0){
      offsetCounter++;
    }

    if(p_ch1_period->val > 0 && offsetCounter >= p_ch1_toffset->val){
      currentTimestamp = ceil(osKernelSysTick()/1000);

      if(p_ch1->val == 1){

        if((currentTimestamp - onTimestamp) > p_ch1_period->val){
          //turn off
          offTimestamp = ceil(osKernelSysTick()/1000);
          p_ch1->val = 0;
        }

      }else{

        if((currentTimestamp - offTimestamp) > p_ch1_period->val){
          //turn on
          onTimestamp = ceil(osKernelSysTick()/1000);
          p_ch1->val = 1;
        }
        
      }

      
    }

    HAL_GPIO_TogglePin(GPIOA, LED_Pin);

    osDelay(50);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartUAVCANTask(void const *argument)  {

  //initialization of can and uavcan
  oi_uavcan_init();

  while (1)  {

    oi_uavcan_sendCanard();
    oi_uavcan_receiveCanard();
    oi_uavcan_spinCanard(osKernelSysTick());

    osDelay (10);                                // delay execution for 10 milliseconds
  }
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
