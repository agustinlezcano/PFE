/*
 * motors.c
 *
 *  Created on: Nov 1, 2025
 *      Author: agustin
 */

#include <motors.h>
#include <main.h>	// TODO: add constants an cmacros to main.h

Motor motor1 = {
    .dirPort = DIR1_GPIO,
    .dirPin = DIR1_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.currentStep = 0,
	.targetStep = 10,
	.totalSteps = 0,
	.i = 108/19.0,
	.angleHoming = 0.0f,
	.dir = CW,
	.homing = false,
	.angleDone = false
};

Motor motor2 = {
    .dirPort = DIR2_GPIO,
    .dirPin = DIR2_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.currentStep = 0,
	.targetStep = 10,
	.i = 32/12.0,
	.angleHoming = 90.0f,
	.angleDone = false
};

Motor motor3 = {
    .dirPort = DIR3_GPIO,
    .dirPin = DIR3_PIN,
    .currentAngle = 0.0f,
    .targetAngle = 0.0f,
	.i = 32/12.0,
	.currentStep = 0,
	.targetStep = 10,
	.angleHoming = 0.0f,
	.angleDone = false
};

// TODO: define methods to init and update motors
