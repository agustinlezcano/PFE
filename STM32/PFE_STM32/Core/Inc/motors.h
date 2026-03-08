/*
 * motor.h
 *
 *  Created on: Nov 1, 2025
 *      Author: agustin
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
//#include <arm_math.h>
#include <math.h>
#include <stdbool.h>

//Maquina de estados (una por motor)
typedef enum {
    IDLE,      // detenido, sin control
    TRAJ,      // siguiendo trayectoria
	P2P,      // siguiendo trayectoria
    APPROX,      // sosteniendo posición final
    HOMING,    // búsqueda de referencia
    MOTOR_ERROR      // fallo (opcional)
} MotorState;

//Estructura para los motores
typedef struct {
	const uint8_t id;
	const float i;
	const float angleHoming;

	GPIO_TypeDef *DIR_PORT;
    uint16_t DIR_PIN;

    TIM_HandleTypeDef *timer;
    uint32_t timerChannel;

    volatile float currentAngle;
    volatile float targetAngle;
    GPIO_PinState dir;

    MotorState state;
    uint16_t speed;
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;

extern Motor* motors[];

#endif /* INC_MOTORS_H_ */
