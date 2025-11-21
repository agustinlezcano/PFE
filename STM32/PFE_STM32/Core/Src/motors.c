/*
 * motors.c
 *
 *  Created on: Nov 1, 2025
 *      Author: agustin
 */

#include <motors.h>
#include <main.h>	// TODO: add constants an cmacros to main.h

Motor motor1 = {
	.id = 1,
	.i = 108/19.0,
	.angleHoming = 0.0f,
    .dirPort = DIR1_GPIO,
    .dirPin = DIR1_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.currentStep = 0,
	.targetStep = 0,
	.totalSteps = 0,
	.dir = CW,
	.homing = false,
	.angleDone = false
};

Motor motor2 = {
	.id = 2,
	.i = 32/12.0,
	.angleHoming = 90.0f,
    .dirPort = DIR2_GPIO,
    .dirPin = DIR2_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.currentStep = 0,
	.targetStep = 0,
	.angleDone = false
};

Motor motor3 = {
	.id = 3,
	.i = 32/12.0,
	.angleHoming = 0.0f,
    .dirPort = DIR3_GPIO,
    .dirPin = DIR3_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.currentStep = 0,
	.targetStep = 0,
	.angleDone = false
};

// TODO: define methods to init and update motors
