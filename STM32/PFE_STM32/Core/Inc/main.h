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
#include <motors.h>
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* USER CODE BEGIN EFP */
int __io_putchar(int ch);
void cmd_callback(const void * msgin);
void inverse_kinematics_callback(const void * msgin);
void subscription_callback(const void * msgin);
void homing_callback(const void * msgin);
void inverseKinematics(const float x, const float y, const float z);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
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

#define CW  1
#define CCW 0

// Sensor Hall - limite de carrera motor 1
#define HALL_SENSOR_GPIO GPIOC
#define HALL_SENSOR_PIN  GPIO_PIN_7 //PC_7 = D9 - X limit switch en CNC Shield

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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
