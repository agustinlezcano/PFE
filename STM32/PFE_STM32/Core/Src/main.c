/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/point.h>
#include <example_interfaces/msg/string.h>
#include <example_interfaces/msg/w_string.h>
#include <extra_interfaces/extra_interfaces/msg/trama.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//STEP 1 -> PA_6 = D12


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 6100 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mControlTask */
osThreadId_t mControlTaskHandle;
const osThreadAttr_t mControlTask_attributes = {
  .name = "mControlTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for KinematicsTask */
osThreadId_t KinematicsTaskHandle;
const osThreadAttr_t KinematicsTask_attributes = {
  .name = "KinematicsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

// PubMutex not used: changed for BitFlags

// micro-ROS publisher
rcl_publisher_t publisher;
std_msgs__msg__String pub_msg;								            // used if there is no tim_msg
geometry_msgs__msg__Point tim_msg;							          // One publisher used by Timer (callback uses printf)

osMessageQueueId_t mid_JointQueue;							          // motor cmd (1 for each DOF triplet)
osMessageQueueId_t mid_PositionQueue;                		  // message queue id (QueueHandle_t CMSIS wrapper)
osMessageQueueId_t mid_FeedbackQueue;                		  // message queue id (QueueHandle_t CMSIS wrapper)

// TODO: check uses
osEventFlagsId_t evt_id;                        			    // Event group: event flags id Callback-Task communication

// Constantes de transmisión
const uint16_t STEPS_PER_REV = 200*8; // 8 micropasos
const float DEG_TO_STEPS = STEPS_PER_REV / 360.0;

//Maquina de estados I2C
typedef enum {
    I2C_IDLE,
    I2C_MUX_TX,
    I2C_AS5600_RX
} I2C_State_t;
volatile I2C_State_t i2c_state = I2C_IDLE;
volatile int i2c_last_ret = 0;
volatile int current_motor = 0;
uint8_t mux_data;
uint8_t as5600_buf[2];
uint8_t sensor_index = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartKinematicsTask(void *argument);

/* USER CODE BEGIN PFP */
void homing_callback(const void * msgin);
//void inverse_kinematics_callback(const void * msgin);
void cmd_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void request_angles_callback(const void * msgin);
void estop_callback(const void * msgin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  motor1.currentAngle = readAngle_AS5600(1);
  motor2.currentAngle = readAngle_AS5600(2);
  motor3.currentAngle = readAngle_AS5600(3);
  electromagnetOn(false);
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  __HAL_DBGMCU_FREEZE_TIM13(); ///////////////////////////////////////////////////////////
  __HAL_DBGMCU_FREEZE_TIM2();  // Sirve para que los timers se detengan en los breakpoints
  __HAL_DBGMCU_FREEZE_TIM3();  ///////////////////////////////////////////////////////////

  printf("System ready.\r\n");
  printf("Angle1 = %.2f ---- Angle2 = %.2f ---- Angle3 = %.2f\r\n", motor1.currentAngle, motor2.currentAngle, motor3.currentAngle);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  // Mailbox or queue - TODO: check for need to save setpoints (not lost SP)
	mid_PositionQueue = osMessageQueueNew(1, sizeof(CARTESIAN_POS_t), NULL); // This function cannot be called from Interrupt Service Routines.
	if (mid_PositionQueue == NULL) {
		printf("Could not create CARTESIAN_POS_t queue.\r\n"); // Message Queue object not created, handle failure
	}

	mid_JointQueue = osMessageQueueNew(1, sizeof(JOINT_POS_t), NULL); // This function cannot be called from Interrupt Service Routines.
	if (mid_JointQueue == NULL) {
		printf("Could not create Joint_FeedbackQueue queue.\r\n"); // Message Queue object not created, handle failure
	}

	// TODO: check if CARTESIAN or JOINT
	mid_FeedbackQueue = osMessageQueueNew(1, sizeof(CARTESIAN_POS_t), NULL); // This function cannot be called from Interrupt Service Routines.
	if (mid_FeedbackQueue == NULL) {
		printf("Could not create mid_FeedbackQueue queue.\r\n"); // Message Queue object not created, handle failure
	}

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of mControlTask */
  mControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &mControlTask_attributes);

  /* creation of KinematicsTask */
  KinematicsTaskHandle = osThreadNew(StartKinematicsTask, NULL, &KinematicsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  // Event Group (CMSIS Layer)
// 	evt_id = osEventFlagsNew(NULL);
// 	if (evt_id == NULL) {
// 		printf("Event flag creation failed\r\n"); // Event Flags object not created, handle failure
// 	}
 	// First set to true to begin OK TODO: optional- check doing doHoming() and it sets to true
 	// TODO: revisar esto porque si se pone en alto y luego no se lee el mensaje, pasa de largo
 	//osEventFlagsSet(evt_id, FLAGS_MSK1);		// flag to be read by TaskControlMotor

  // Task creation (CMSIS Layer).
  // TODO: check when rebuild .ioc (automatic creation of tasks) if it overwrites this part of code
  MX_Tasks_Init();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) { //Sirve para redirigir el printf a la UART3
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void parseCmd(const geometry_msgs__msg__Point *msg,
		CARTESIAN_POS_t *xSetpointToSend) {
	xSetpointToSend->x = msg->x;
	xSetpointToSend->y = msg->y;
	xSetpointToSend->z = msg->z;
}

void sendJointCmd(osMessageQueueId_t mid_PositionQueue, CARTESIAN_POS_t xSetpointToSend) {
	// Poner message type Point en queue
	osStatus_t xStatus;	// container para Overwrite wrapper
	CARTESIAN_POS_t dummy;

	if (osMessageQueueGetSpace(mid_PositionQueue) == 0) {
			osMessageQueueGet(mid_PositionQueue, &dummy, NULL, 0); // eliminar el más antiguo
		}
	xStatus = osMessageQueuePut(mid_PositionQueue, &xSetpointToSend, osPriorityNormal, 10);
	if (xStatus != osOK) {
		printf( "Could not send to the queue.\r\n" );
	}

}

void sendFeedbackCmd(osMessageQueueId_t mid_FeedbackQueue, CARTESIAN_POS_t xSetpointToSend) {
	// put message type Point in queue
	osStatus_t xStatus;	// container for Overwrite wrapper
	CARTESIAN_POS_t dummy;

	// mailbox: populate message one by one Point - CARTESIAN_POS_t TODO: check for requirements/control
	if (osMessageQueueGetSpace(mid_FeedbackQueue) == 0) {
			osMessageQueueGet(mid_FeedbackQueue, &dummy, NULL, 0); // clean the oldest
		}
	xStatus = osMessageQueuePut(mid_FeedbackQueue, &xSetpointToSend, osPriorityNormal, 10); // Used for only one position in queue
	if (xStatus != osOK) {
		printf( "Could not send to the queue.\r\n" );
	}

}

void cmd_callback(const void * msgin) {
	// Cast to Trama message type
	const extra_interfaces__msg__Trama *msg = (const extra_interfaces__msg__Trama *)msgin;
	//printf("CMD-ACK\r\n");

	// Build JOINT_POS_t from Trama message
	JOINT_POS_t cmd;
	osStatus_t status;

	// Copy joint positions q[3]
	for (int i = 0; i < 3; i++) {
		cmd.q[i] = msg->q[i];
		cmd.qd[i] = msg->qd[i];
	}
	cmd.t_total = msg->t_total;
	cmd.n_iter = msg->n_iter;
	cmd.traj_state = msg->traj_state;  // Use value from trajectory planner

	//printf("Trama: q=[%.2f, %.2f, %.2f] qd=[%.2f, %.2f, %.2f] t=%.2f n=%ld traj_state=%d\r\n",
		   //cmd.q[0], cmd.q[1], cmd.q[2],
		   //cmd.qd[0], cmd.qd[1], cmd.qd[2],
		   //cmd.t_total, (long)cmd.n_iter, cmd.traj_state);

	// Put in queue (overwrite if full)
	status = osMessageQueuePut(mid_JointQueue, &cmd, 0, 0);
	//if (status != osOK)
		//printf("Error putting Trama in mid_JointQueue\r\n");
}

//void inverse_kinematics_callback(const void * msgin)
//{
//	CARTESIAN_POS_t xSetpointToSend;
//	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
//	//printf("CMD-KIN\r\n");
//	// Callback managed by executor
//	//parseCmd(msg, &xSetpointToSend);						// parse geometry_msgs__msg__Point to CARTESIAN_POS_t
//	//sendJointCmd(mid_PositionQueue, xSetpointToSend);		// put setpoint in Position Queue
//	moveToAbsAngle(&motor1, msg->x);
//	moveToAbsAngle(&motor2, msg->y);
//	moveToAbsAngle(&motor3, msg->z);
//}

void homing_callback(const void * msgin)
{
	// TODO: trigger
	// TODO: implementar homing: cola o if el de motorCmd, o en task Nueva
	// TODO: hacer cola homing o flag global pero para Joint en vez de Position
	// TODO: validate inner fileds to be put in queue
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
	if (msg->data) {
		printf("Homing enabled\r\n");

		// Homing command: zero positions trigger homing sequence
		JOINT_POS_t cmd = {
			.q = {-100.0, -100.0, -100.0},
			.qd = {0.0, 0.0, 0.0},
			.t_total = 0.0,
			.n_iter = 0,
			.traj_state = 0
		};
		osStatus_t status = osMessageQueuePut(mid_JointQueue, &cmd, 0, 0);

		//if (status != osOK)
			//printf("Error sending homing command to queue\r\n");
	}
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	// OPTIONAL: if queues logic failed, use topics  and publish odometry on demand
	CARTESIAN_POS_t xReceivedStructure;
	osStatus_t xStatus;
	// TODO: check for use Point and no Mutex
	xStatus = osMessageQueueGet(mid_FeedbackQueue, &xReceivedStructure, NULL,
	0);

	if (xStatus == osOK) {
		printf("Received Kinematics setpoint: [%lf %lf %lf].\r\n",
				xReceivedStructure.x, xReceivedStructure.y,
				xReceivedStructure.z);
		// Assign values to Point message
		tim_msg.x = xReceivedStructure.x;
		tim_msg.y = xReceivedStructure.y;
		tim_msg.z = xReceivedStructure.z;

		if (timer != NULL) {
			// Publish Point message
			rcl_publish(&publisher, &tim_msg, NULL);
		}
	}

}


void electromagnet_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  // Read requested state and act (no E-Stop override here)
  bool requested_state = msg->data;

	// Actuate the electromagnet (non-blocking HAL_GPIO call)
	electromagnetOn(requested_state);
	// Fallback: UART printf as heartbeat confirmation
	printf("Electromagnet CB: requested=%s actual=%s\r\n", msg->data ? "ON" : "OFF", requested_state ? "ON" : "OFF");

  // State echo publisher removed; rely on other feedback channels (printf)
}

void estop_callback(const void * msgin){
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  // Read E-Stop state
  bool estop_engaged = msg->data;

  // Actuate E-Stop: if engaged, stop all motors immediately; if disengaged, allow normal operation (no override here)
  if (estop_engaged) {
    // Stop all motors: set PWM to zero and optionally set a flag to ignore further commands until reset
    for (int i = 0; i < 3; i++) {
	  Motor* m = motors[i];
      m->state = MOTOR_ERROR;
    }
    printf("E-Stop ENGAGED: Motors stopped.\r\n");
  } else {
    for (int i = 0; i < 3; i++) {
	  Motor* m = motors[i];
      m->state = IDLE;
    }
    printf("E-Stop DISENGAGED: Motors can operate.\r\n");
  }
}

void request_angles_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data) {
    // Publish current angles using the Point message
    tim_msg.x = (double)motor1.currentAngle;
    tim_msg.y = (double)motor2.currentAngle;
    tim_msg.z = (double)motor3.currentAngle;

    rcl_publish(&publisher, &tim_msg, NULL);
    printf("Current angles published: [%.2f, %.2f, %.2f]\r\n",
           motor1.currentAngle, motor2.currentAngle, motor3.currentAngle);
  }
}

//### Configura PWM de los motores ###
void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz) {
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2; // Depende del bus/timer *2 xq PCLK1 es 42 MHz
    uint32_t prescaler = htim->Instance->PSC + 1;
    uint32_t arr = (timer_clk / (prescaler * freq_hz)) - 1;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);
}

//### Selecciona canal en TCA9548A ###
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel) {
    uint8_t data = 1 << channel;  // Ej: canal 0 = 0x01, canal 1 = 0x02
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &data, 1, HAL_MAX_DELAY);
}

//### Lee angulos del multiplexor AS5600 ###
float readAngle_AS5600(int motor) { //Lee el angulo del sensor AS5600 en la I2C1
    uint8_t data[2];
    bool invert;

    if (motor == 2){
      TCA9548A_SelectChannel(1); // Seleccionar canal 1 → leer sensor 1
      invert = true;
    }
    else if(motor == 3){
      TCA9548A_SelectChannel(0); // Seleccionar canal 0 → leer sensor 2
      invert = false;
    }
    else if(motor == 1){
      TCA9548A_SelectChannel(2); // Seleccionar canal 0 → leer sensor 2
      invert = true;
    }
    else{
      return 0.0;
    }

    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH, I2C_MEMADD_SIZE_8BIT, &data[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_LOW,  I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
    uint16_t angle = ((uint16_t)data[0] << 8) | data[1];

    uint16_t rawAngle = angle & 0x0FFF;
    if (invert){
    	rawAngle = (4096 - rawAngle) & 0x0FFF;
    	//Inversion de direccion, porque en I2C no se invierte la direccion al escribir el registro, solo ocurre eso leyendo el valor analogico
    	//La direccion adoptada para motores 2 y 3 es creciente sentido horario, visto desde motor 2
    }

    float currentAngle = (rawAngle * 360.0f) / 4096.0f; // El ángulo es de 12 bits -> 4096 en 360°

    //Para pasar al rango (-180, 180]
    if (currentAngle > 180.0f) currentAngle -= 360.0f;

    return currentAngle;
}

float filterAngle(float angleReaded, float lastAngle){
	if (fabs(angleReaded - lastAngle) > ANGLE_REJECT_DEG){
		return lastAngle; //se descarta el ultimo angulo leido
	}
	else {
		return angleReaded; //se considera el ultimo angulo leido
	}
}

//bool inverseKinematics(const float x, const float y, const float z) {
//    float T_obj[4][4] = {
//            {1, 0,  0, x},
//            {0, 1,  0, y},
//            {0, 0,  1, z},
//            {0, 0,  0, 1}
//            };
//
//    float qm[3];
//	bool ok = cinv_geometrica_f32(dh, T_obj, Base, Tool, offset, lim, qm);
//
//	if(ok){
//		printf("C_Inv OK\r\n");
//
//		motor1.targetAngle = qm[0]*180/M_PI;
//		motor2.targetAngle = qm[1]*180/M_PI;
//		motor3.targetAngle = qm[2]*180/M_PI;
//	}
//	else{
//		printf("C_Inv NO OK\r\n");
//	}
//	return ok;
//}

void moveToAbsAngle(Motor *m, float angulo_abs){
    float angulo_rel = angulo_abs - m->currentAngle;
    m->targetAngle = angulo_abs;

    if(fabs(angulo_rel) > ANGLE_TOLERANCE){
    	GPIO_PinState dir = (angulo_rel >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

		m->dir = dir;
		m->state = P2P;
		m->speed = 300;

		HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, dir);
		Stepper_SetSpeed(m->timer, m->timerChannel, m->speed);

		HAL_TIM_PWM_Start(m->timer, m->timerChannel);
    	HAL_TIM_Base_Start_IT(m->timer);
    }
}

void electromagnetOn(bool turn_on){
	if(turn_on){
		HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_RESET);
	}
}

void doHoming(Motor *m) {
	moveToAbsAngle(m, m->angleHoming);
}

void trajectoryControl(Motor *m, float q_ref, float qd_ref){
	float v_cmd, q_err;
	//m->targetAngle = q_ref;

	switch(m->state){
	case TRAJ:
		q_err = q_ref - m->currentAngle;
		v_cmd = qd_ref + KP_POS * q_err; //feedforward de velocidad + feedback proporcional de posicion
		break;
	case APPROX:
		q_err = m->targetAngle - m->currentAngle;
		v_cmd = KP_APPROX * q_err; //se compensa error de posicion al final de trayectoria
		break;
	default: return;
	}

	//Se aplican los cambios: direccion con HAL_GPIO_WritePin y amplitud de velocidad en Stepper_SetSpeed
    float v_hz = fabsf(v_cmd) * DEG_TO_STEPS * m->i;

    if(v_hz > 1000) v_hz = 1000;
    if(v_hz < 1) v_hz = 1; //para evitar la division por cero

    m->dir = (v_cmd >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    m->speed = v_hz;
	HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);
	Stepper_SetSpeed(m->timer, m->timerChannel, m->speed);
}

void AS5600_StartRead_IT(int motor){
    if (i2c_state != I2C_IDLE)
        return;

    current_motor = motor;

    if (motor == 1) mux_data = 1 << 2;
    else if (motor == 2) mux_data = 1 << 1;
    else if (motor == 3) mux_data = 1 << 0;
    else return;

  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(&hi2c1, TCA9548A_ADDR, &mux_data, 1);
  if (ret == HAL_OK) {
    i2c_state = I2C_MUX_TX;
  }
}

//////////////////////INTERRUPCIONES I2C

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c != &hi2c1) return;

  if (i2c_state == I2C_MUX_TX){
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH, I2C_MEMADD_SIZE_8BIT, as5600_buf, 2);
    if (ret == HAL_OK){
      i2c_state = I2C_AS5600_RX;
    } else {
      i2c_last_ret = (int)ret;
      i2c_state = I2C_IDLE; // reset state so next timer can retry
    }
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c != &hi2c1) return;

    if (i2c_state == I2C_AS5600_RX)
    {
        uint16_t raw = ((uint16_t)as5600_buf[0] << 8) | as5600_buf[1];
        raw &= 0x0FFF;
        bool invert;

        switch(current_motor){
        case 1:
        case 2: invert = true; break;
        case 3: invert = false; break;
        default: return;
        }

		if (invert){
			raw = (4096 - raw) & 0x0FFF;
		}
		float currentAngle = (raw * 360.0f) / 4096.0f; // El ángulo es de 12 bits -> 4096 en 360°
		if (currentAngle > 180.0f) currentAngle -= 360.0f;

		switch(current_motor){
		    case 1: motor1.currentAngle = filterAngle(currentAngle, motor1.currentAngle); break;
		    case 2: motor2.currentAngle = filterAngle(currentAngle, motor2.currentAngle); break;
		    case 3: motor3.currentAngle = filterAngle(currentAngle, motor3.currentAngle); break;
		    default: return;
        }

        i2c_state = I2C_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if (hi2c != &hi2c1) return;
  i2c_last_ret = (int)hi2c->ErrorCode;
  i2c_state = I2C_IDLE; // free the state machine so timer can retry
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //for(;;)
  //{
  //  osDelay(1);
  //}
  /* USER CODE END 5 */
//}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the mControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
//void StartMotorControlTask(void *argument)
//{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  //for(;;)
  //{
  //  osDelay(1);
  //}
  /* USER CODE END StartMotorControlTask */
//}

/* USER CODE BEGIN Header_StartKinematicsTask */
/**
* @brief Function implementing the KinematicsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKinematicsTask */
//void StartKinematicsTask(void *argument)
//{
  /* USER CODE BEGIN StartKinematicsTask */
  /* Infinite loop */
  //for(;;)
  //{
    //osDelay(1);
  //}
  /* USER CODE END StartKinematicsTask */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  // Motor 1 → TIM1 (PWM13/1)
  if (htim->Instance == TIM13){
	  switch (motor1.state){
		  case APPROX:
		  case HOMING:
		  case P2P:
			  float err = fabsf(motor1.currentAngle - motor1.targetAngle);
			  if(err < 3){
			  	  motor1.speed = err * 60;
			  	  if(motor1.speed < 20.0f) motor1.speed = 20.0f;
				  Stepper_SetSpeed(motor1.timer, motor1.timerChannel, motor1.speed);
			  }
			  if((motor1.dir && motor1.currentAngle >= (motor1.targetAngle - ANGLE_TOLERANCE)) || (!motor1.dir && motor1.currentAngle <= (motor1.targetAngle + ANGLE_TOLERANCE))){
				  motor1.state = IDLE;
				  if(ALL_MOTORS_IDLE){ // Ultimo motor que se detiene publica y lo lee el publisher
					  CARTESIAN_POS_t xFeedback_ = {
							.x = motor1.currentAngle,
							.y = motor2.currentAngle,
							.z = motor3.currentAngle
						};
					  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					  xQueueSendFromISR(mid_FeedbackQueue, &xFeedback_, &xHigherPriorityTaskWoken);
                      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				  }
				  HAL_TIM_Base_Stop_IT(motor1.timer);
				  HAL_TIM_PWM_Stop(motor1.timer, motor1.timerChannel);
			  }
			  break;
		      case MOTOR_ERROR:
		  		  HAL_TIM_Base_Stop_IT(motor1.timer);
		  		  HAL_TIM_PWM_Stop(motor1.timer, motor1.timerChannel);
		  		break;
		  default: break;
	  }
  }

  else if (htim->Instance == TIM2){
          switch(motor2.state){
			  case APPROX:
			  case HOMING:
			  case P2P:
				  float err = fabsf(motor2.currentAngle - motor2.targetAngle);
				  if(err < 3){
					  motor2.speed = err * 60;
					  if(motor2.speed < 20.0f) motor2.speed = 20.0f;
					  Stepper_SetSpeed(motor2.timer, motor2.timerChannel, motor2.speed);
				  }
				  if((motor2.dir && motor2.currentAngle >= (motor2.targetAngle - ANGLE_TOLERANCE)) || (!motor2.dir && motor2.currentAngle <= (motor2.targetAngle + ANGLE_TOLERANCE))){
					motor2.state = IDLE;
					if(ALL_MOTORS_IDLE){ // Ultimo motor que se detiene publica y lo lee el publisher
						CARTESIAN_POS_t xFeedback_ = {
							.x = motor1.currentAngle,
							.y = motor2.currentAngle,
							.z = motor3.currentAngle
						};
						BaseType_t xHigherPriorityTaskWoken = pdFALSE;
						xQueueSendFromISR(mid_FeedbackQueue, &xFeedback_, &xHigherPriorityTaskWoken);
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					}
					HAL_TIM_Base_Stop_IT(motor2.timer);
					HAL_TIM_PWM_Stop(motor2.timer, motor2.timerChannel);
				  }
				  break;
			  case MOTOR_ERROR:
				  HAL_TIM_Base_Stop_IT(motor2.timer);
				  HAL_TIM_PWM_Stop(motor2.timer, motor2.timerChannel);
				  break;
			  default: break;
          }
      }

  else if (htim->Instance == TIM3){
	  switch (motor3.state){
		  case APPROX:
		  case HOMING:
		  case P2P:
			  float err = fabsf(motor3.currentAngle - motor3.targetAngle);
			  if(err < 3){
				  motor3.speed = err * 60;
				  if(motor1.speed < 20.0f) motor1.speed = 20.0f;
				  Stepper_SetSpeed(motor3.timer, motor3.timerChannel, motor3.speed);
			  }
			  if((motor3.dir && motor3.currentAngle >= (motor3.targetAngle - ANGLE_TOLERANCE)) || (!motor3.dir && motor3.currentAngle <= (motor3.targetAngle + ANGLE_TOLERANCE))){
				  motor3.state = IDLE;
				  if(ALL_MOTORS_IDLE){ // Ultimo motor que se detiene publica y lo lee el publisher
					  CARTESIAN_POS_t xFeedback_ = {
						.x = motor1.currentAngle,
						.y = motor2.currentAngle,
						.z = motor3.currentAngle
					  };
					  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					  xQueueSendFromISR(mid_FeedbackQueue, &xFeedback_, &xHigherPriorityTaskWoken);
					  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				  }
				  HAL_TIM_Base_Stop_IT(motor3.timer);
				  HAL_TIM_PWM_Stop(motor3.timer, motor3.timerChannel);
			  }
			  break;
		  case MOTOR_ERROR:
		  	  HAL_TIM_Base_Stop_IT(motor3.timer);
		  	  HAL_TIM_PWM_Stop(motor3.timer, motor3.timerChannel);
		  	  break;
		  default: break;
	  }
  }

    else if(htim->Instance == TIM7){
    	//float angle1 = readAngle_AS5600(1);
    	//motor1.currentAngle = filterAngle(angle1, motor1.currentAngle);
    	//float angle2 = readAngle_AS5600(2);
    	//motor2.currentAngle = filterAngle(angle2, motor2.currentAngle);
    	//float angle3 = readAngle_AS5600(3);
    	//motor3.currentAngle = filterAngle(angle3, motor3.currentAngle);
        if (i2c_state == I2C_IDLE){
          AS5600_StartRead_IT(sensor_index);
          sensor_index++;
          if (sensor_index > 3) sensor_index = 1;
        }
    }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
