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
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 7100 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mControlTask */
osThreadId_t mControlTaskHandle;
const osThreadAttr_t mControlTask_attributes = {
  .name = "mControlTask",
  .stack_size = 180 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for KinematicsTask */
osThreadId_t KinematicsTaskHandle;
const osThreadAttr_t KinematicsTask_attributes = {
  .name = "KinematicsTask",
  .stack_size = 220 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for xPubMutex */
osMutexId_t xPubMutexHandle;
const osMutexAttr_t xPubMutex_attributes = {
  .name = "xPubMutex"
};
/* USER CODE BEGIN PV */
// micro-ROS publisher
rcl_publisher_t publisher;
std_msgs__msg__String pub_msg;

osMessageQueueId_t mid_JointQueue[N_MOTORS];				// motor cmd (1 each DOF)
osMessageQueueId_t mid_PositionQueue;                		// message queue id (QueueHandle_t CMSIS wrapper)

osThreadId_t tid_Thread_MsgQueue1;              			// thread id 1
osThreadId_t tid_Thread_MsgQueue2;              			// thread id 2

// Constantes de transmisión
const int STEPS_PER_REV = 200*8; // 8 micropasos

//CONSTANTES CINEMATICAS DEL ROBOT
const float L1 = 0.078;   //[m]
const float L2 = 0.135;   //[m]
const float L3 = 0.147;   //[m]
const float L4 = 0.0253;  //[m]

// DH: [theta, d, a, alpha] por fila
float dh[4][4] = {
	{0, L1,  0,   PI/2},
	{0, 0,  L2,   0.0f},   // L2
	{0, 0,  L3,   0.0f},   // L3
	{0, 0,  L4,   0.0f}    // L4
};

float Tool[4][4] = {
            {1,0,0,0},
            {0,1,0,0},
            {0,0,1,0},
            {0,0,0,1}
};

float tool_z = 0.04115;

float Base[4][4] = {
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
};

float offset[4] = {0, PI/2, -PI/2, 0};
float lim[3][2] = {
	{ -90*PI/180, 90*PI/180},
	{ 60*PI/180, 157*PI/180},
	{ -25*PI/180, 65*PI/180}
};

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
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartKinematicsTask(void *argument);

/* USER CODE BEGIN PFP */
void subscription_callback(const void * msgin);
void homing_callback(const void * msgin);
void inverse_kinematics_callback(const void * msgin);
void cmd_callback(const void * msgin);

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
  /* USER CODE BEGIN 2 */
  //Measure motors 2 & 3
  //	motor2.currentAngle = readAngle_AS5600(2);
  //	motor3.currentAngle = readAngle_AS5600(3);

  //	HAL_TIM_Base_Start_IT(&htim13);
  //	HAL_TIM_Base_Start_IT(&htim2);
  //	HAL_TIM_Base_Start_IT(&htim3);
  printf("System ready.\r\n");
  printf("Angle2 = %.2f ---- Angle3 = %.2f\r\n", motor2.currentAngle,
  			motor3.currentAngle);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of xPubMutex */
  xPubMutexHandle = osMutexNew(&xPubMutex_attributes);

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
  mid_PositionQueue = osMessageQueueNew(1, sizeof(CARTESIAN_POS_t), NULL);	// This function cannot be called from Interrupt Service Routines.
  if (mid_PositionQueue == NULL) {
		printf("Could not create CARTESIAN_POS_t queue.\r\n"); // Message Queue object not created, handle failure
  }
  // 3 mailbox or 1 queue 3 values --> must be individual queues
  for(int i = 0; i<N_MOTORS; i++){
	  mid_JointQueue[i] = osMessageQueueNew(1, sizeof(float), NULL);
	    if (mid_JointQueue[i] == NULL) {
	  	printf("Could not create mid_JointQueue %d queue.\r\n",i); // Message Queue object not created, handle failure
	    }
  }


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
	// put message type Point in queue
	osStatus_t xStatus;	// container for Overwrite wrapper
	CARTESIAN_POS_t dummy;

	// mailbox: populate message one by one Point - CARTESIAN_POS_t TODO: check for requirements/control
	if (osMessageQueueGetSpace(mid_PositionQueue) == 0) {
			osMessageQueueGet(mid_PositionQueue, &dummy, NULL, 0); // eliminar el más antiguo
		}
	xStatus = osMessageQueuePut(mid_PositionQueue, &xSetpointToSend, osPriorityNormal, 10); // Used for only one position in queue
	if (xStatus != osOK) {
		printf( "Could not send to the queue.\r\n" );
	}

}

void cmd_callback(const void * msgin) {
	// TODO: make and cast direct kinematics cmd
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
	char buffer[100];
//	char data[] = "Ingreso a Callback\r\n";
//	snprintf(buffer, sizeof(buffer), "ACK: %s");
	// Asignar al mensaje global
	publish_text("CMD-ACK");
	float cmd[3];
	osStatus_t status;
	cmd[0] = (float) msg->x;
	cmd[1] = (float) msg->y;
	cmd[2] = (float) msg->z;

//	printf("cmd_callback\r\n");
//	char data[] = "cmd_callback\r\n";
//	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);

    for (int i=0; i<N_MOTORS; i++) {
    	status = osMessageQueuePut(mid_JointQueue[i], &cmd[i], 0, 0);	//TODO: validate inner fields
    	if(status != 0){
    		printf("Error creating mid_JointQueue element in cmd_callback\r\n");
    	}
    }
}

void inverse_kinematics_callback(const void * msgin)
{
	CARTESIAN_POS_t xSetpointToSend;
//	char data2[] = "Ingreso a Callback\r\n";
//	HAL_UART_Transmit(&huart3, &data2, sizeof(data2), 10);
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
	// Asignar al mensaje global
	publish_text("CMD-KIN");
	// Callback managed by executor
	parseCmd(msg, &xSetpointToSend);						// parse geometry_msgs__msg__Point to CARTESIAN_POS_t
	sendJointCmd(mid_PositionQueue, xSetpointToSend);		// put setpoint in Position Queue
}

void subscription_callback(const void * msgin)
{
//	char data2[] = "Ingreso a Callback\r\n";
//	HAL_UART_Transmit(&huart3, &data2, sizeof(data2), 10);
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
//	char buffer[100];
//	snprintf(buffer, sizeof(buffer), "ACK: ");
//	char data[50];
//	snprintf(data, sizeof(data), "PRE-ASSIGN");
//	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);
	// Asignar al mensaje global
	publish_text("CMD-SUB");
}

void homing_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "HOM: %d", msg->data);
    publish_text(buffer);
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
    else{
      return 0.0;
    }
    //HAL_Delay(2); // tiempo corto de estabilización

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
    if (currentAngle > 270){
    	currentAngle -= 360;
    }

    return currentAngle;
}

bool inverseKinematics(const float x, const float y, const float z) {
    float T_obj[4][4] = {
            {1, 0,  0, x},
            {0, 1,  0, y},
            {0, 0,  1, z},
            {0, 0,  0, 1}
            };

    float qm[3];
	bool ok = cinv_geometrica_f32(dh, T_obj, Base, Tool, offset, lim, qm);

	if(ok){
		printf("C_Inv OK\r\n");

		motor1.targetAngle = qm[0]*180/PI;
		motor2.targetAngle = qm[1]*180/PI;
		motor3.targetAngle = qm[2]*180/PI;
	}
	else{
		printf("C_Inv NO OK\r\n");
	}
	return ok;
}

void moveToAbsAngle(int motor, float angulo_abs, int velocidad){ //La logica podria ser: mover motor X pasos, leer angulo, si le falta/sobra compensar
	//velocidad [steps/s] = freq [Hz]  xq 1 paso es 1 periodo de PWM
	float angulo_actual;
	float relacion;

	switch (motor) {
	  case 1: angulo_actual = motor1.currentAngle; motor1.targetAngle = angulo_abs; relacion = motor1.i; Stepper_SetSpeed(&htim13, TIM_CHANNEL_1, velocidad); break;
	  case 2: angulo_actual = motor2.currentAngle; motor2.targetAngle = angulo_abs; relacion = motor2.i; Stepper_SetSpeed(&htim2, TIM_CHANNEL_2, velocidad); break;
	  case 3: angulo_actual = motor3.currentAngle; motor3.targetAngle = angulo_abs; relacion = motor3.i; Stepper_SetSpeed(&htim3, TIM_CHANNEL_2, velocidad); break;
	  default: return;
	}

	float angulo_rel = angulo_abs - (angulo_actual);
	printf("Motor %d targetAngle %.2f relativeAngle %.2f\r\n", motor, angulo_abs, angulo_rel);
	GPIO_PinState direccion = (angulo_rel >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	float pasos = (fabs(angulo_rel) / 360.0) * STEPS_PER_REV * relacion;
	moveMotor(motor, (int)pasos, (int)velocidad, direccion);
}

void angleCompensation(int motor, float angulo_abs, float angulo_actual, float relacion){
	printf("Falta compensar %.2f\r\n", angulo_abs - angulo_actual);
	float angulo_rel = angulo_abs - angulo_actual;

	GPIO_PinState direccion = (angulo_rel >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	float pasos = (fabs(angulo_rel) / 360.0) * STEPS_PER_REV * relacion;
	printf("Pasos a realizar motor %d = %d\r\n", motor, (int)pasos);
	moveMotor(motor, (int)pasos, 200, direccion);
}

void electromagnetOn(bool turn_on){
	if(turn_on){
		HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_RESET);
	}
}

void doHoming(int motor, GPIO_PinState dir) {
  GPIO_TypeDef* dirPort;
  uint16_t dirPin;
  float angleHoming;

  switch (motor) {
    case 1: dirPort = motor1.dirPort; dirPin = motor1.dirPin; break;
    case 2: dirPort = motor2.dirPort; dirPin = motor2.dirPin; angleHoming = motor2.angleHoming; break;
    case 3: dirPort = motor3.dirPort; dirPin = motor3.dirPin; angleHoming = motor3.angleHoming; break;
    default: return;
  }

  if (motor == 1){
	  HAL_GPIO_WritePin(dirPort, dirPin, dir); // 1. Configurar dirección hacia el fin de carrera
	  Stepper_SetSpeed(&htim13, TIM_CHANNEL_1, 250); // 2. Configurar velocidad lenta para homing

	  motor1.homing = true;
	  motor1.totalSteps = 0; // 3. Inicializar contadores
	  motor1.currentStep = 0;

	  HAL_TIM_Base_Start_IT(&htim13);
	  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); // 4. Iniciar PWM
  }
  else if(motor == 2 || motor == 3){
	  moveToAbsAngle(motor, angleHoming, 500);
  }

}

void moveMotor(int motor, int pasos, int velocidad, GPIO_PinState dir) {
  switch (motor) {
      case 1:
    	  HAL_GPIO_WritePin(motor1.dirPort, motor1.dirPin, dir);
    	  motor1.currentStep = 0;
    	  motor1.targetStep = pasos;
    	  motor1.angleDone = false;
    	  motor1.dir = dir;
    	  HAL_TIM_Base_Start_IT(&htim13);
    	  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    	  break;
      case 2:
    	  HAL_GPIO_WritePin(motor2.dirPort, motor2.dirPin, dir);
    	  motor2.currentStep = 0;
    	  motor2.targetStep = pasos;
    	  motor2.angleDone = false;
    	  HAL_TIM_Base_Start_IT(&htim2);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    	  break;
      case 3:
    	  HAL_GPIO_WritePin(motor3.dirPort, motor3.dirPin, dir);
    	  motor3.currentStep = 0;
    	  motor3.targetStep = pasos;
    	  motor3.angleDone = false;
    	  HAL_TIM_Base_Start_IT(&htim3);
		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    	  break;
      default: return;
    }
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
//  /* USER CODE BEGIN 5 */
//////  /* Infinite loop */
//////  for(;;)
//////  {
//////    osDelay(1);
//////  }
//  /* USER CODE END 5 */
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
//  /* USER CODE BEGIN StartMotorControlTask */
//////  /* Infinite loop */
//////  for(;;)
//////  {
//////    osDelay(1);
//////  }
//  /* USER CODE END StartMotorControlTask */
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
//  /* USER CODE BEGIN StartKinematicsTask */
//////  /* Infinite loop */
//////  for(;;)
//////  {
//////    osDelay(1);
//////  }
//  /* USER CODE END StartKinematicsTask */
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
	  if(motor1.homing){ // Hace el homing
		  if (!HAL_GPIO_ReadPin(HALL_SENSOR_GPIO, HALL_SENSOR_PIN)){
			  motor1.homing = false;
			  motor1.totalSteps = 0;
			  motor1.currentStep = 0;
			  motor1.currentAngle = motor1.angleHoming;
			  motor1.angleDone = true;

		      HAL_TIM_Base_Stop_IT(&htim13);
		      HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
		  }
	  }
	  else{ // Mueve el motor 1 con normalidad
		  motor1.currentStep++;

		  if (motor1.dir == CW){
			  motor1.totalSteps++;
		  }
		  else{
			  motor1.totalSteps--;
		  }

		  if (motor1.currentStep >= motor1.targetStep){
			  motor1.angleDone = true;
			  HAL_TIM_Base_Stop_IT(&htim13);
			  HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
		  }

		  motor1.currentAngle = (motor1.totalSteps * 360.0f / STEPS_PER_REV) / motor1.i;
  }}

  // Motor 2 → TIM2 (PWM2/2)
  else if (htim->Instance == TIM2){
	  motor2.currentStep++;
	  motor2.currentAngle = readAngle_AS5600(2);

	  if (motor2.currentStep >= motor2.targetStep){
		  motor2.angleDone = true;
		  HAL_TIM_Base_Stop_IT(&htim2);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	  }
  }

  // Motor 3 → TIM3 (PWM3/2)
  else if (htim->Instance == TIM3){
	  motor3.currentStep++;
	  motor3.currentAngle = readAngle_AS5600(3);

	  if (motor3.currentStep >= motor3.targetStep){
		  motor3.angleDone = true;
		  HAL_TIM_Base_Stop_IT(&htim3);
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
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
