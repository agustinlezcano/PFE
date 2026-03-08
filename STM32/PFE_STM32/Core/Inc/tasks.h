/*
 * tasks.h
 *
 *  Created on: Oct 27, 2025
 *      Author: agustin
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_


// DELETE these lines:
#include "cmsis_os.h"    // duplicate
#include "tasks.h"       // self-include (useless)
#include "main.h"        // causes circular dependency
#include <stdint.h>
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


/* Joint trajectory command structure - matches Trama.msg */
typedef struct {
    double q[3];      // Joint positions [rad]
    double qd[3];     // Joint velocities [rad/s]
    double t_total;   // Total trajectory time [s]
    int32_t n_iter;   // Number of iterations/points
    int32_t traj_state;
} JOINT_POS_t;

/* Handles declaration */
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t mControlTaskHandle;
extern osThreadId_t KinematicsTaskHandle;

/* Functions declarations */
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartKinematicsTask(void *argument);

/* Queue helper for overwrite pattern (Producer side) */
osStatus_t Queue_OverwriteLatest(osMessageQueueId_t queueId, const JOINT_POS_t *pJointCmd);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);


void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* Attributes declaration */
extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t mControlTask_attributes;
extern const osThreadAttr_t KinematicsTask_attributes;

void MX_Tasks_Init(void);  // Init tasks

#endif /* INC_TASKS_H_ */
