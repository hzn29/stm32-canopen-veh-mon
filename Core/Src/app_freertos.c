/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 提供 FreeRTOS 静态 Idle 任务所需的控制块和栈空间。 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  /* 保存 Idle 任务控制块，确保调度器启动后地址持续有效。 */
  static StaticTask_t idleTaskTCB;
  /* 保存 Idle 任务栈，数组长度使用 FreeRTOS 栈字数。 */
  static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];
  /* 返回 Idle 任务控制块地址。 */
  *ppxIdleTaskTCBBuffer = &idleTaskTCB;
  /* 返回 Idle 任务栈首地址。 */
  *ppxIdleTaskStackBuffer = idleTaskStack;
  /* 返回 Idle 任务栈容量。 */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* 提供 FreeRTOS 静态 Timer 任务所需的控制块和栈空间。 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
  /* 保存 Timer 任务控制块，确保定时器服务任务持续可用。 */
  static StaticTask_t timerTaskTCB;
  /* 保存 Timer 任务栈，数组长度使用 FreeRTOS 配置值。 */
  static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];
  /* 返回 Timer 任务控制块地址。 */
  *ppxTimerTaskTCBBuffer = &timerTaskTCB;
  /* 返回 Timer 任务栈首地址。 */
  *ppxTimerTaskStackBuffer = timerTaskStack;
  /* 返回 Timer 任务栈容量。 */
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* USER CODE END Application */

