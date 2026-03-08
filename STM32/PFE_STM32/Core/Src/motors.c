/*
 * motors.c
 *
 *  Created on: Nov 1, 2025
 *      Author: agustin
 */

#include "main.h"	// TODO: add constants an cmacros to main.h
#include "motors.h"

Motor motor1 = {
	.id = 1,
	.i = 108/19.0,
	.angleHoming = 0.0f,
    .DIR_PORT = DIR1_GPIO,
    .DIR_PIN = DIR1_PIN,
	.timer = &htim13,
	.timerChannel = TIM_CHANNEL_1,
	.state = IDLE,
	.speed = 300
};

Motor motor2 = {
	.id = 2,
	.i = 32/12.0,
	.angleHoming = 90.0f,
    .DIR_PORT = DIR2_GPIO,
    .DIR_PIN = DIR2_PIN,
	.timer = &htim2,
    .timerChannel = TIM_CHANNEL_2,
	.state = IDLE,
	.speed = 300
};

Motor motor3 = {
	.id = 3,
	.i = 32/12.0,
	.angleHoming = 0.0f,
    .DIR_PORT = DIR3_GPIO,
    .DIR_PIN = DIR3_PIN,
	.timer = &htim3,
	.timerChannel = TIM_CHANNEL_2,
	.state = IDLE,
	.speed = 300
};

// TODO: define methods to init and update motors
Motor* motors[] = { &motor1, &motor2, &motor3 };
