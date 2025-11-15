/*
 * tasks.c
 *
 *  Created on: Oct 27, 2025
 *      Author: agustin
 */
#include "tasks.h"
#include "main.h"
#include "cmsis_os.h"
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
#define N_MOTORS 3

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

typedef struct {
	// Point position
	double x;
	double y;
	double z;} MSGQUEUE_OBJ_t;
extern float tool_z;
extern float currentAngle1;
extern float currentAngle2;
extern float currentAngle3;
extern float targetAngle1;
extern float targetAngle2;
extern float targetAngle3;
osMessageQueueId_t mid_JointQueue[N_MOTORS];				// motor cmd (1 each DOF)
osMessageQueueId_t mid_PositionQueue;

extern rcl_publisher_t publisher;
extern std_msgs__msg__String pub_msg;

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

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// micro-ROS configuration
	rmw_uros_set_custom_transport(true, (void*) &huart2, cubemx_transport_open, cubemx_transport_close,
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

	char data[] = "Configured\r\n";
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

void StartMotorControlTask(void *argument) {
	/* USER CODE BEGIN StartMotorControlTask */
	/* Infinite loop */
	MSGQUEUE_OBJ_t xReceivedStructure;
	BaseType_t xStatus;
	for (;;) {
		for (int i=0; i<N_MOTORS; i++){
			xStatus = osMessageQueueGet(mid_JointQueue, &xReceivedStructure, osPriorityAboveNormal,
					portMAX_DELAY);
		}
		vTaskDelay(pdMS_TO_TICKS(1));	//osDelay(1);
	}
	/* USER CODE END StartMotorControlTask */
}

void StartKinematicsTask(void *argument)
{
  /* USER CODE BEGIN StartKinematicsTask */
	/* Infinite loop */
	MSGQUEUE_OBJ_t xReceivedStructure;
	BaseType_t xStatus;
	BaseType_t status;

	for (;;) {
//		xStatus = xQueueReceive(mid_PositionQueue, &xReceivedStructure,
//				portMAX_DELAY);
		xStatus = osMessageQueueGet(mid_PositionQueue, &xReceivedStructure, osPriorityAboveNormal,
						portMAX_DELAY);

		if (xStatus == pdPASS) {
			printf("Received Kinematics setpoint: [%lf %lf %lf].\r\n",
					xReceivedStructure.x, xReceivedStructure.y,
					xReceivedStructure.z);
			// TODO: parse queue values
			////Resolve kinematics and assign to global variables
			inverseKinematics(xReceivedStructure.x, xReceivedStructure.y, xReceivedStructure.z + tool_z);
			//Send command to queues - to be consumed by Motor Task
			// TODO: check for variable or struct atttribute
			status = osMessageQueuePut(mid_JointQueue[0], &motor1.targetAngle, osPriorityAboveNormal, 0);
			if(status != 0) printf("Error creating xCmdQueue element 1 in cmd_callback\r\n");

			status = osMessageQueuePut(mid_JointQueue[1], &motor2.targetAngle, osPriorityAboveNormal, 0);
			if(status != 0) printf("Error creating xCmdQueue element 2 in cmd_callback\r\n");

			status = osMessageQueuePut(mid_JointQueue[2], &motor3.targetAngle, osPriorityAboveNormal, 0);
			if(status != 0) printf("Error creating xCmdQueue element 3 in cmd_callback\r\n");

			printf("Kinematics: angles to make: [%f; %f; %f]", targetAngle1, targetAngle2, targetAngle3);
		}
		else{
			printf("Failed queue reception.\r\n");
		}
	}
  /* USER CODE END StartKinematicsTask */
}

/* Función para crear las tareas */
void MX_Tasks_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  mControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &mControlTask_attributes);
  KinematicsTaskHandle = osThreadNew(StartKinematicsTask, NULL, &KinematicsTask_attributes);
}

