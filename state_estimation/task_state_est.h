/*
 * task_state_est.h
 *
 *  Created on: Nov 29, 2019
 *      Author: Jonas
 */

#ifndef INC_TASKS_TASK_STATE_EST_H_
#define INC_TASKS_TASK_STATE_EST_H_

/* Includes */
#include "cmsis_os.h"
#include "../util.h"
#include "../math_utils.c"
#include "../environment/env.c"
#include "state_est_const.h"
#include "state_est.c"
#include "kf.c"
#include "../flight_phase_detection/flight_phase_detection.c"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define PI 3.1415


/* Commands */

/* Extern */
/* Sensor Board 1 */
extern osMutexId_t sb1_mutex;
extern sb_data_t sb1_data;

/* Sensor Board 2 */
extern osMutexId_t sb2_mutex;
extern sb_data_t sb2_data;

/* Sensor Board 3 */
extern osMutexId_t sb3_mutex;
extern sb_data_t sb3_data;

/* State Estimation Mutex */
extern osMutexId_t state_est_mutex;
extern state_est_data_t state_est_data_global;

/* fsm Mutex */
extern osMutexId_t fsm_mutex;
extern osMutexId_t environment_mutex;
extern flight_phase_detection_t global_flight_phase_detection;
extern env global_env;

/* Tasks */
void vTaskStateEst(void *argument);

#endif /* INC_TASKS_TASK_STATE_EST_H_ */
