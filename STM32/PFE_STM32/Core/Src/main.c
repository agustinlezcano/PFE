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
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MSGQUEUE_OBJECTS 1                     // number of Message Queue Objects
#define N_MOTORS 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 7150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mControlTask */
osThreadId_t mControlTaskHandle;
const osThreadAttr_t mControlTask_attributes = {
  .name = "mControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for KinematicsTask */
osThreadId_t KinematicsTaskHandle;
const osThreadAttr_t KinematicsTask_attributes = {
  .name = "KinematicsTask",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for xMotorCmd */
osMessageQueueId_t xMotorCmdHandle;
const osMessageQueueAttr_t xMotorCmd_attributes = {
  .name = "xMotorCmd"
};
/* USER CODE BEGIN PV */
// micro-ROS publisher
rcl_publisher_t publisher;
std_msgs__msg__String pub_msg;
// TODO: check if use buffer
typedef struct {
	// Point position
	double x;
	double y;
	double z;
	// Quaternion orientation
	// double x_0;
	// double y_0;
	// double z_0;
	// double w_1;

} MSGQUEUE_OBJ_t;
osMessageQueueId_t xCmdQueue[N_MOTORS];				// motor cmd (1 each DOF)
osMessageQueueId_t mid_MsgQueue;                // message queue id (QueueHandle_t CMSIS wrapper)

osThreadId_t tid_Thread_MsgQueue1;              // thread id 1
osThreadId_t tid_Thread_MsgQueue2;              // thread id 2

// Constantes de transmisión
const float i1 = 108.0 / 19.0;
const float i2 = 47.0 / 19.0;
const float i3 = 47.0 / 19.0;
const int microSteps = 32;
const int stepsPerRev = 200 * 32;

// Angulos
float currentAngle1 = 0; //Angulos actuales
float currentAngle2 = 90;
float currentAngle3 = 0;
float targetAngle1;
float targetAngle2;
float targetAngle3;
const float angleHoming1 = 0;
const float angleHoming2 = 90;
const float angleHoming3 = 0;


// Robot Parameters
float Tool[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0,
		1 } };

float tool_z = 0.04115;

float Base[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0,
		1 } };

const float L1 = 0.078;   //[m]
const float L2 = 0.135;   //[m]
const float L3 = 0.147;   //[m]
const float L4 = 0.0253;  //[m]

// DH: [theta, d, a, alpha] por fila // TODO: corregir constante PI
float dh[4][4] = { { 0, L1, 0, PI / 2 }, { 0, 0, L2, 0.0f },   // L2
		{ 0, 0, L3, 0.0f },   // L3
		{ 0, 0, L4, 0.0f }    // L4
};

float offset[4] = { 0, PI / 2, -PI / 2, 0 };
float lim[4][2] = { { -185 * PI / 180, 185 * PI / 180 }, { -65 * PI / 180, 45
		* PI / 180 }, { -70 * PI / 180, 20 * PI / 180 }, { -90 * PI / 180, 90
		* PI / 180 } };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartKinematicsTask(void *argument);

/* USER CODE BEGIN PFP */
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
  /* USER CODE BEGIN 2 */

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

  /* Create the queue(s) */
  /* creation of xMotorCmd */
  xMotorCmdHandle = osMessageQueueNew (3, sizeof(uint16_t), &xMotorCmd_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  // Mailbox or queue - TODO: check for need to save setpoints (not lost SP)
  mid_MsgQueue = osMessageQueueNew(1, sizeof(MSGQUEUE_OBJ_t), NULL);	// This function cannot be called from Interrupt Service Routines.
  if (mid_MsgQueue == NULL) {
		printf("Could not create MSGQUEUE_OBJ_t queue.\r\n"); // Message Queue object not created, handle failure
  }
  // 3 mailbox or 1 queue 3 values --> must be individual queues
  for(int i = 0; i<N_MOTORS; i++){
	  xCmdQueue[i] = osMessageQueueNew(1, sizeof(int), NULL);
	    if (xCmdQueue[i] == NULL) {
	  	printf("Could not create xCmdQueue %d queue.\r\n",i); // Message Queue object not created, handle failure
	    }
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA8 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
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

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
void inverseKinematics(const float x, const float y, const float z);
void subscription_callback(const void * msgin);
void homing_callback(const void * msgin);
void inverse_kinematics_callback(const void * msgin);
void cmd_callback(const void * msgin);

int __io_putchar(int ch) { //Sirve para redirigir el printf a la UART3
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void cmd_callback(const void * msgin) {
	// TODO: make and cast direct kinematics cmd
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
	char buffer[100];
	char data[] = "Ingreso a Callback\r\n";
	snprintf(buffer, sizeof(buffer), "ACK: %d");
	// Asignar al mensaje global
	rosidl_runtime_c__String__assign(&pub_msg.data, "Recibido Cmd");
	// Publicar respuesta
	RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

	double cmd[3];
	osStatus_t status;
	cmd[0] = msg->x;
	cmd[1] = msg->y;
	cmd[2] = msg->z;

	printf("cmd_callback\r\n");
//	char data[] = "cmd_callback\r\n";
//	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);

    for (int i=0; i<N_MOTORS; i++) {
    	status = osMessageQueuePut(xCmdQueue[i], &cmd[i], 0, 0);	//TODO: validate inner fields
    	if(status != 0){
    		printf("Error creating xCmdQueue element in cmd_callback\r\n");
    	}
    }
}

void inverse_kinematics_callback(const void * msgin)
{
	BaseType_t xStatus;	// container for Overwrite wrapper
	char data2[] = "Ingreso a Callback\r\n";
	HAL_UART_Transmit(&huart3, &data2, sizeof(data2), 10);
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
	// Asignar al mensaje global
	rosidl_runtime_c__String__assign(&pub_msg.data, "Recibido Kinematics");
	// Publicar respuesta
	RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

	// Callback managed by executor
	// put message type Point in queue
	// TODO: corregir. No deberia ser una definicion y redefinicion
	// TODO: check estructura
	MSGQUEUE_OBJ_t xSetpointToSend;

	xSetpointToSend.x = msg->x;
	xSetpointToSend.y = msg->y;
	xSetpointToSend.z = msg->z;

	// mailbox: populate message one by one Point - MSGQUEUE_OBJ_t TODO: check for requirements/control
	xStatus = osMessageQueuePut(mid_MsgQueue, &xSetpointToSend, 0, 0); // Used for only one position in queue TODO change msg
	if(xStatus != osOK){
		printf( "Could not send to the queue.\r\n" );
	}
}

void subscription_callback(const void * msgin)
{
	char data2[] = "Ingreso a Callback\r\n";
	HAL_UART_Transmit(&huart3, &data2, sizeof(data2), 10);
	const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "ACK: ");
	char data[50];
	snprintf(data, sizeof(data), "PRE-ASSIGN");
	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);
	// Asignar al mensaje global
	rosidl_runtime_c__String__assign(&pub_msg.data, "ACK");
	snprintf(data, sizeof(data), "POST-ASSIGN");
	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);
	// Publicar respuesta
	RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

void homing_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "ACK: %d", msg->data);
    // Asignar al mensaje global
    rosidl_runtime_c__String__assign(&pub_msg.data, "Recibido Homing");
    // Publicar respuesta
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

void inverseKinematics(const float x, const float y, const float z) {
		float T_obj[4][4] = { { 1, 0, 0, x }, { 0, 1, 0, y }, { 0, 0, 1, z }, {
				0, 0, 0, 1 } };

		float qf[4];
		int ok = cinv_geometrica_f32(dh, T_obj, Base, Tool, offset, lim, qf);
		if (ok) {
			printf("C_Inv OK\r\n");

			for (int i = 0; i < 4; i++) {
				printf("%.2f\r\n", qf[i] * 180 / PI);
			}

			targetAngle1 = qf[0] * 180 / PI;
			targetAngle2 = (PI / 2 - qf[1]) * 180 / PI;
			targetAngle3 = (-(qf[1] + qf[2])) * 180 / PI;
		} else {
			printf("C_Inn NO OK\r\n");
		}
	}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
	/** * @brief Function implementing the defaultTask thread. * @param argument: Not used * @retval None */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// micro-ROS configuration
	rmw_uros_set_custom_transport(
	true, (void*) &huart2, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app
	// micro-ROS subscriber
	rcl_subscription_t subscriber;
	rcl_subscription_t subs_bool;
	rcl_subscription_t subs_invk;
	rcl_subscription_t subs_cmd;

	// micro-ROS message type
	geometry_msgs__msg__Point sub_msg;			// Suscripcion de control TODO: cambiar
	std_msgs__msg__Bool homing_msg;				// Homing
	geometry_msgs__msg__Point cmd_msg;			// Cinematica directa TODO: usar un msg custom con 3 float
	geometry_msgs__msg__Point inverse_msg;		// Pos articular Cinem inversa

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "microros_pubsub_rclc", "", &support));

	// Start Publisher/Subscriber msgs
	// TODO: check if init is necessary for all msg types
	std_msgs__msg__Bool__init(&homing_msg);

	char data[] = "configurado\r\n";
	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);

	// create subscriber
	// TODO: corregit nombre de topico en subscriber
	RCCHECK(
			rclc_subscription_init_default( &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/int32_subscriber"));

	RCCHECK(
			rclc_subscription_init_default( &subs_bool, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/homing"));

	RCCHECK(
			rclc_subscription_init_default( &subs_invk, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/inverse"));

	RCCHECK(
			rclc_subscription_init_default( &subs_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/cmd"));

	// create publisher
	RCCHECK(
			rclc_publisher_init_default( &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/string_publisher"));

	// create executor
	// TODO: update number of callbacks
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));// Tercer parámetro: tiemrs y callbacks
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
					&subscription_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_bool, &homing_msg,
					&homing_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_invk, &inverse_msg,
					&inverse_kinematics_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_cmd, &cmd_msg,
					&cmd_callback, ON_NEW_DATA));

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_subscription_fini(&subs_bool, &node));
	RCCHECK(rcl_subscription_fini(&subs_invk, &node));
	RCCHECK(rcl_subscription_fini(&subs_cmd, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the mControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartKinematicsTask */
/**
* @brief Function implementing the KinematicsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKinematicsTask */
void StartKinematicsTask(void *argument)
{
  /* USER CODE BEGIN StartKinematicsTask */
	/* Infinite loop */
	MSGQUEUE_OBJ_t xReceivedStructure;
	BaseType_t xStatus;

	for (;;) {
		xStatus = xQueueReceive(mid_MsgQueue, &xReceivedStructure,
				portMAX_DELAY);
		if (xStatus == pdPASS) {
			printf("Received Kinematics setpoint: [%lf %lf %lf].\r\n",
					xReceivedStructure.x, xReceivedStructure.y,
					xReceivedStructure.z);
			// TODO: parse de los valores de la cola
			//Hace la cinematica en el punto XYZ indicado --->>> OJO!! es sin tool, se la agregamos aparte <<<---
			inverseKinematics(xReceivedStructure.x, xReceivedStructure.y, xReceivedStructure.z + tool_z);
			printf("Kinematics: angle to make: [%f; %f; %f]", targetAngle1, targetAngle2, targetAngle3);
		}
		else{
			printf("Failed queue reception.\r\n");
		}
	}
  /* USER CODE END StartKinematicsTask */
}

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
