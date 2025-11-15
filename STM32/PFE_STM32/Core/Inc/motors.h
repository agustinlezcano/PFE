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

typedef struct {
    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    float currentAngle;
    float targetAngle;
    uint16_t currentStep;
    uint16_t targetStep;
    uint16_t totalSteps;
    bool dir;

    const float i;
    bool angleDone;
    bool homing;

    const float angleHoming;
} Motor;


extern Motor motor1;
extern Motor motor2;
extern Motor motor3;

#endif /* INC_MOTORS_H_ */
