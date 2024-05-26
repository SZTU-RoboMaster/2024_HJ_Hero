/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
osThreadId calibrateTaskHandle;
osThreadId chassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId detectTaskHandle;
osThreadId usbtaskHandle;
osThreadId decodetaskHandle;
osThreadId uipaintTaskHandle;
osThreadId ledTaskHandle;
osThreadId CapTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Calibrate_task(void const * argument);
void Chassis_task(void const * argument);
void Gimbal_task(void const * argument);
void INS_task(void const * argument);
void Detect_task(void const * argument);
void USB_task(void const * argument);
void Decode_task(void const * argument);
void UI_Paint_task(void const * argument);
void led_Task(void const * argument);
void cap_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
QueueHandle_t CDC_send_queue;

/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of calibrateTask */
  osThreadDef(calibrateTask, Calibrate_task, osPriorityNormal, 0, 512);
  calibrateTaskHandle = osThreadCreate(osThread(calibrateTask), NULL);

  /* definition and creation of chassisTask */
  osThreadDef(chassisTask, Chassis_task, osPriorityLow, 0, 512);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, Gimbal_task, osPriorityHigh, 0, 512);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, INS_task, osPriorityIdle, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of detectTask */
  osThreadDef(detectTask, Detect_task, osPriorityIdle, 0, 128);
  detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

  /* definition and creation of usbtask */
  osThreadDef(usbtask, USB_task, osPriorityHigh, 0, 128);
  usbtaskHandle = osThreadCreate(osThread(usbtask), NULL);

  /* definition and creation of decodetask */
  osThreadDef(decodetask, Decode_task, osPriorityHigh, 0, 128);
  decodetaskHandle = osThreadCreate(osThread(decodetask), NULL);

  /* definition and creation of uipaintTask */
  osThreadDef(uipaintTask, UI_Paint_task, osPriorityIdle, 0, 128);
  uipaintTaskHandle = osThreadCreate(osThread(uipaintTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, led_Task, osPriorityBelowNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of CapTask */
  osThreadDef(CapTask, cap_task, osPriorityAboveNormal, 0, 256);
  CapTaskHandle = osThreadCreate(osThread(CapTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    CDC_send_queue = xQueueCreate(1, 128);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Calibrate_task */
/**
* @brief Function implementing the calibrateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Calibrate_task */
__weak void Calibrate_task(void const * argument)
{
  /* USER CODE BEGIN Calibrate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Calibrate_task */
}

/* USER CODE BEGIN Header_Chassis_task */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_task */
__weak void Chassis_task(void const * argument)
{
  /* USER CODE BEGIN Chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_task */
}

/* USER CODE BEGIN Header_Gimbal_task */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_task */
__weak void Gimbal_task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_task */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_Detect_task */
/**
* @brief Function implementing the detectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_task */
__weak void Detect_task(void const * argument)
{
  /* USER CODE BEGIN Detect_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_task */
}

/* USER CODE BEGIN Header_USB_task */
/**
* @brief Function implementing the usbtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USB_task */
__weak void USB_task(void const * argument)
{
  /* USER CODE BEGIN USB_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END USB_task */
}

/* USER CODE BEGIN Header_Decode_task */
/**
* @brief Function implementing the decodetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decode_task */
__weak void Decode_task(void const * argument)
{
  /* USER CODE BEGIN Decode_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Decode_task */
}

/* USER CODE BEGIN Header_UI_Paint_task */
/**
* @brief Function implementing the uipaintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Paint_task */
__weak void UI_Paint_task(void const * argument)
{
  /* USER CODE BEGIN UI_Paint_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_Paint_task */
}

/* USER CODE BEGIN Header_led_Task */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_Task */
__weak void led_Task(void const * argument)
{
  /* USER CODE BEGIN led_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_Task */
}

/* USER CODE BEGIN Header_cap_task */
/**
* @brief Function implementing the CapTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cap_task */
__weak void cap_task(void const * argument)
{
  /* USER CODE BEGIN cap_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END cap_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
