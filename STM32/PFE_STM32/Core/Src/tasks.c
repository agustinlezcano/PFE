/*
 * tasks.c
 *
 *  Created on: Oct 27, 2025
 *      Author: agustin
 */
#include "tasks.h"
#include "main.h"
#include "motors.h"
#include "cmsis_os.h"
#include <math.h>
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
	float angle1;
	float angle2;
	float angle3;} MOTORQUEUE_t;
extern float tool_z;
extern float currentAngle1;
extern float currentAngle2;
extern float currentAngle3;
extern float targetAngle1;
extern float targetAngle2;
extern float targetAngle3;
extern osMessageQueueId_t mid_JointQueue;				// motor cmd (1 each DOF)
extern osMessageQueueId_t mid_PositionQueue;
extern osMessageQueueId_t mid_FeedbackQueue;

//extern osMutexId_t xPubMutexHandle;								// TODO: check if delete
extern osEventFlagsId_t evt_id;

extern rcl_publisher_t publisher;
extern geometry_msgs__msg__Point tim_msg;

char pub_buffer[48];											// buffer para memoria estatica en mensajes
const unsigned int timer_period = RCL_MS_TO_NS(500);

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
	rcl_subscription_t subs_bool;
	//rcl_subscription_t subs_invk;
	rcl_subscription_t subs_cmd;
	rcl_subscription_t subs_emagnet;
	rcl_subscription_t subs_estop; /////////////////////////////////////////////////////////////////
	rcl_subscription_t subs_req_angles;

	// micro-ROS message types
	std_msgs__msg__Bool homing_msg;					// Homing
	extra_interfaces__msg__Trama cmd_msg;			// Joint trajectory command (Trama)
	//geometry_msgs__msg__Point inverse_msg;			// Cartesian position for inverse kinematics
	std_msgs__msg__Bool emagnet_msg;				// Electromagnet on/off
	std_msgs__msg__Bool estop_msg;					// Emergency stop /////////////////////////////////////////////////////////////////
	std_msgs__msg__Bool req_angles_msg;				// Request current angles

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
//	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	// Esperar infinitamente hasta que el agente responda
	rcl_ret_t ret = RMW_RET_ERROR;
	while (ret != RCL_RET_OK) {
	    ret = rmw_uros_ping_agent(100, 1);
	    vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100ms y reintenta
	}

	// Una vez que el ping responde, inicializamos
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "microros_pubsub_rclc", "", &support));

	// Start Publisher/Subscriber msgs - initialize all message types
	std_msgs__msg__Bool__init(&homing_msg);
	extra_interfaces__msg__Trama__init(&cmd_msg);  // Trama for /microROS/cmd
	//geometry_msgs__msg__Point__init(&inverse_msg);
	std_msgs__msg__Bool__init(&emagnet_msg);
	std_msgs__msg__Bool__init(&estop_msg); /////////////////////////////////////////////////////////////////
	std_msgs__msg__Bool__init(&req_angles_msg);

	char data[] = "Configured\r\n";
	HAL_UART_Transmit(&huart3, &data, sizeof(data), 10);

	// Create and initialize timer object
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

	// create subscriber
	// TODO: corregir nombre de topico en subscriber
	RCCHECK(
			rclc_subscription_init_default( &subs_bool, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/homing"));

//	RCCHECK(
//			rclc_subscription_init_default( &subs_invk, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/inverse"));

	RCCHECK(
			rclc_subscription_init_default( &subs_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(extra_interfaces, msg, Trama), "/microROS/cmd"));

	RCCHECK(
			rclc_subscription_init_default( &subs_emagnet, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/electroiman"));
	RCCHECK(
			rclc_subscription_init_default( &subs_estop, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/emergency_stop"));
	/////////////////////////////////////////////////////////////////

	// create publisher
	RCCHECK(
			rclc_subscription_init_default( &subs_req_angles, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/request_current_angles"));

	RCCHECK(
			rclc_publisher_init_default( &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/string_publisher"));

	//init_publisher_message();		// init message definition used in publisher

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));// 7 callbacks: homing, inverse, cmd, emagnet, estop, req_angles, timer

	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_bool, &homing_msg,
					&homing_callback, ON_NEW_DATA));
//	RCCHECK(
//			rclc_executor_add_subscription(&executor, &subs_invk, &inverse_msg,
//					&inverse_kinematics_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_cmd, &cmd_msg,
					&cmd_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_emagnet, &emagnet_msg,
					&electromagnet_callback, ON_NEW_DATA));
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_estop, &estop_msg,
					&estop_callback, ON_NEW_DATA)); /////////////////////////////////////////////////////////////////
	RCCHECK(
			rclc_executor_add_subscription(&executor, &subs_req_angles, &req_angles_msg,
					&request_angles_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subs_bool, &node));
	//RCCHECK(rcl_subscription_fini(&subs_invk, &node));
	RCCHECK(rcl_subscription_fini(&subs_cmd, &node));
	RCCHECK(rcl_subscription_fini(&subs_emagnet, &node));
	RCCHECK(rcl_subscription_fini(&subs_estop, &node));
	RCCHECK(rcl_subscription_fini(&subs_req_angles, &node));
	RCCHECK(rcl_timer_fini(&timer));	// Destroy timer

	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);

  /* USER CODE END 5 */
}

/**
 * @brief  Motor Control Task (Consumer)
 * @note   Waits for MOTOR_READY_BIT before fetching the latest target from the queue.
 *         This ensures no stale data is processed while motors are busy.
 * @param  argument: Not used
 * @retval None
 */
void StartMotorControlTask(void *argument) {
	/* USER CODE BEGIN StartMotorControlTask */
	osStatus_t xStatus;
	uint32_t flags;
	JOINT_POS_t joint_value;

	for (;;) {
		// Step 1: Wait for MOTOR_READY_BIT (blocking wait - motors must be idle)
		// osFlagsWaitAny: returns when ANY of the specified flags are set
		// osFlagsNoClear: we DON'T clear here, we clear when we START moving
		// TODO: enable event flags (just commented for test)
//		flags = osEventFlagsWait(evt_id, MOTOR_READY_BIT, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
//
//		if (flags & osFlagsError) {
//			// Error in event wait, retry
//			vTaskDelay(pdMS_TO_TICKS(10));
//			continue;
//		}

		// Step 2: Try to get the latest target from the queue (non-blocking)
		// Queue has size 1, so this always gets the most recent value
		xStatus = osMessageQueueGet(mid_JointQueue, &joint_value, NULL, 0);

		if (xStatus != osOK) {
			// No new target available, yield and retry
//			printf("failed get mid_JointQueue\r\n");
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		// Step 3: Clear MOTOR_READY_BIT - motors are now BUSY
		// This prevents processing new targets until movement completes
		//osEventFlagsClear(evt_id, MOTOR_READY_BIT);

		//printf("Motor Task: Processing q=[%.2f, %.2f, %.2f] t=%.2f\r\n",
		  // joint_value.q[0], joint_value.q[1], joint_value.q[2], joint_value.t_total);

		// Step 5: Execute motor movements using joint positions
		float angles[3] = {(float)joint_value.q[0], (float)joint_value.q[1], (float)joint_value.q[2]};

		for (int i = 0; i < 3; i++) {
			float angle = angles[i];

			Motor* m = motors[i];
			switch(joint_value.traj_state){
				case 0:
						if (angle <= -100.0f) {
							m->state = HOMING; // Step 4: Reset angleDone flags before starting movement
							doHoming(m);
						}
						else {
							m->state = P2P;
							moveToAbsAngle(m, angle);
						}
						break;
				case 1: //Start TRAJECTORY
						m->state = TRAJ;
					    HAL_TIM_PWM_Start(m->timer, m->timerChannel);
					    HAL_TIM_Base_Start_IT(m->timer);
						trajectoryControl(m, joint_value.q[i], joint_value.qd[i]);
						break;
				case 2:
						trajectoryControl(m, joint_value.q[i], joint_value.qd[i]);
						break;
				case 3: //Finish TRAJECTORY
						m->state = APPROX;
						m->targetAngle = joint_value.q[i];
						break;
				default: break;
			}
		}
		printf("ST%ld %.2f %.2f %.2f\r\n", joint_value.traj_state, joint_value.q[0], joint_value.q[1], joint_value.q[2]);

		// Note: MOTOR_READY_BIT will be SET by HAL_TIM_PeriodElapsedCallback
		// when all motors complete their movement (motor1.angleDone && motor2.angleDone && motor3.angleDone)

		// Step 6: Send feedback with current joint targets
//		CARTESIAN_POS_t xFeedback = {
//					.x = motor1.currentAngle,
//					.y = motor2.currentAngle,
//					.z = motor3.currentAngle
//				};
//		sendFeedbackCmd(mid_FeedbackQueue, xFeedback);

		// Stack monitoring (debug)
		UBaseType_t stackFree = uxTaskGetStackHighWaterMark(NULL);
		if (stackFree < 80) {
			printf("MotorControlTask stack LOW: %lu\r\n", stackFree);
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
	/* USER CODE END StartMotorControlTask */
}

/**
 * @brief  Overwrite the single-item queue with the latest value (Producer helper)
 * @note   Ensures queue always holds only the most recent target.
 *         If queue is full, removes old item and puts new one.
 * @param  queueId: Message queue handle
 * @param  pJointCmd: Pointer to new joint command
 * @retval osStatus_t: osOK on success
 */
osStatus_t Queue_OverwriteLatest(osMessageQueueId_t queueId, const JOINT_POS_t *pJointCmd)
{
	osStatus_t status;
	JOINT_POS_t dummy;

	// If queue is full, remove the old (stale) item first
	if (osMessageQueueGetSpace(queueId) == 0) {
		// Non-blocking get to discard old value
		osMessageQueueGet(queueId, &dummy, NULL, 0);
	}

	// Put the new (latest) value
	status = osMessageQueuePut(queueId, pJointCmd, osPriorityNormal, 0);

	return status;
}

/**
 * @brief  Kinematics Task (Producer)
 * @note   Receives Cartesian positions, computes inverse kinematics,
 *         and overwrites the JointQueue with the latest target.
 * @param  argument: Not used
 * @retval None
 */
void StartKinematicsTask(void *argument)
{
  /* USER CODE BEGIN StartKinematicsTask */
	CARTESIAN_POS_t xReceivedStructure;
	osStatus_t xStatus;
	osStatus_t status;

	for (;;) {
		// Block until a new Cartesian target arrives
		xStatus = osMessageQueueGet(mid_PositionQueue, &xReceivedStructure, NULL, osWaitForever);

		if (xStatus == osOK) {
			printf("Kinematics: Received setpoint [%.3lf, %.3lf, %.3lf]\r\n",
					xReceivedStructure.x, xReceivedStructure.y,
					xReceivedStructure.z);

			// Parse Cartesian coordinates
			const float x = (float) xReceivedStructure.x;
			const float y = (float) xReceivedStructure.y;
			const float z = (float) xReceivedStructure.z;

			// Compute inverse kinematics (adds tool offset)
			bool ok = false; //inverseKinematics(x, y, z + tool_z);

			if (ok) {
				// Build joint command from computed target angles
				JOINT_POS_t jointStruct = {
					.q = {motor1.targetAngle, motor2.targetAngle, motor3.targetAngle},
					.qd = {0.0, 0.0, 0.0},  // No velocity from IK
					.t_total = 0.0,
					.n_iter = 1
				};

				// OVERWRITE pattern: always keep only the latest target
				// If motors are busy, old target is discarded in favor of new one
				status = Queue_OverwriteLatest(mid_JointQueue, &jointStruct);

				if (status != osOK) {
					printf("Error: Failed to queue joint command\r\n");
				} else {
					printf("Kinematics: Queued angles [%.2f, %.2f, %.2f]\r\n",
						   jointStruct.q[0], jointStruct.q[1], jointStruct.q[2]);
				}
			} else {
				printf("Kinematics: IK computation failed (unreachable point)\r\n");
			}
		} else {
			printf("Kinematics: Queue reception error\r\n");
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
