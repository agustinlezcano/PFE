/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
//#include <arm_math.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <motors.h>
#include <tasks.h>
#include <queue.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim13;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int __io_putchar(int ch);
void cmd_callback(const void * msgin);
//void inverse_kinematics_callback(const void * msgin);
void subscription_callback(const void * msgin);
void homing_callback(const void * msgin);
void electromagnet_callback(const void * msgin);

void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);
float readAngle_AS5600(int motor);
float filterAngle(float angleReaded, float lastAngle);
bool inverseKinematics(const float x, const float y, const float z);
void moveToAbsAngle(Motor *m, float angulo_abs);
void electromagnetOn(bool turn_on);
void doHoming(Motor *m);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void AS5600_StartRead_IT(int motor);
void trajectoryControl(Motor *m, float q_ref, float qd_ref);
void request_angles_callback(const void * msgin);
void estop_callback(const void * msgin);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define DIR1_GPIO    GPIOA
#define DIR1_PIN     GPIO_PIN_5  //DIR 1 -> PA_5 = D13
//STEP 2 -> PB_3 = D3
#define DIR2_GPIO    GPIOB
#define DIR2_PIN     GPIO_PIN_10 //DIR 2 -> PB_10 = D6
//STEP 3 -> PB_5 = D4
#define DIR3_GPIO    GPIOA
#define DIR3_PIN     GPIO_PIN_8  //DIR 3 -> PA_8 = D7

// Sensor Hall - limite de carrera motor 1
#define LED_EMAGNET_GPIO GPIOC
#define LED_EMAGNET_PIN  GPIO_PIN_7 //PC_7 = D9 - X limit switch en CNC Shield

//Electroiman
#define ELECTROMAGNET_GPIO GPIOC
#define ELECTROMAGNET_PIN  GPIO_PIN_3

//Sensores de angulo AS5600 con multiplexor TCA9548A
#define TCA9548A_ADDR  (0x70 << 1)  // Dirección base (con A0–A2 = GND)
#define AS5600_ADDR (0x36 << 1)  // Dirección I2C del AS5600
#define ANGLE_REG_HIGH 0x0E      // MSB del ángulo
#define ANGLE_REG_LOW  0x0F      // LSB del ángulo


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MSGQUEUE_OBJECTS 1                     // number of Message Queue Objects
#define N_MOTORS 3
#define ALL_MOTORS_IDLE (motor1.state == IDLE && motor2.state == IDLE && motor3.state == IDLE)

// Event Flags for Motor-Kinematics synchronization
#define MOTOR_READY_BIT     0x00000001U   // Set when all motors finished movement
#define NEW_TARGET_BIT      0x00000002U   // Set when new target is available
#define FLAGS_MSK1          MOTOR_READY_BIT   // Backward compatibility

#define ANGLE_TOLERANCE     0.10f    // grados aceptables para considerar que llego
#define ANGLE_REJECT_DEG   20.0f    // rechazo de saltos grandes (umbral de filtro)

#define KP_POS 5.0f
#define KP_APPROX 5.0f

typedef struct {
	double x;
	double y;
	double z;
} CARTESIAN_POS_t;

// typedef struct {
// 	float m1;
// 	float m2;
// 	float m3;
// } JOINT_POS_t;

// TODO: typedef for float Joint position instead of three queues


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
