/*
 * motor.h
 *
 *  Created on: Nov 1, 2025
 *      Author: agustin
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>
#include <math.h>
#include <stdbool.h>

//Estructura para los motores
typedef struct {
	const uint8_t id;
	const float i;
	const float angleHoming;

	GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    float currentAngle;
    float targetAngle;
    uint16_t currentStep;
    uint16_t targetStep;
    int16_t totalSteps;
    bool dir;

    bool angleDone;
    bool homing;

    const uint16_t minSpeed;     // Hz
    const uint16_t maxSpeed;    // Hz
    uint16_t currentSpeed;
    float nextAccelPoint;
    float startAngle;

    float syncedMaxSpeed;
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;

#endif /* INC_MOTORS_H_ */
