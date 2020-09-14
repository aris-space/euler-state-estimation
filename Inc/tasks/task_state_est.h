/*
 * task_state_est.h
 *
 *  Created on: Nov 29, 2019
 *      Author: Jonas
 */

#ifndef INC_TASKS_TASK_STATE_EST_H_
#define INC_TASKS_TASK_STATE_EST_H_

 /* Includes */
#include "Sim_Con/env.h"
#include "Sim_Con/kf.h"
#include "Sim_Con/flight_phase_detection.h"
#include "Sim_Con/state_est_settings.h"
#include "Sim_Con/state_est.h"
#include "Sim_Con/state_est_const.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Util/math_utils.h"
#include "Util/util.h"
#include "Util/mutex.h"
/* Constants */
/* -> Are in state_est_settings */


/* Commands */

/* Extern */
/* Sensor Board 1 */
extern custom_mutex_t sb1_mutex;
extern imu_data_t sb1_imu;
extern baro_data_t sb1_baro;

/* Sensor Board 2 */
extern custom_mutex_t sb2_mutex;
extern imu_data_t sb2_imu;
extern baro_data_t sb2_baro;

/* Sensor Board 3 */
extern custom_mutex_t sb3_mutex;
extern imu_data_t sb3_imu;
extern baro_data_t sb3_baro;

/* State Estimation Mutex */
extern custom_mutex_t state_est_mutex;
extern state_est_data_t state_est_data_global;

/* fsm Mutex */
extern custom_mutex_t fsm_mutex;
extern custom_mutex_t environment_mutex;
extern flight_phase_detection_t global_flight_phase_detection;
extern env_t global_env;

/* Command Mutex */
extern custom_mutex_t command_mutex;
extern command_e global_telemetry_command;




/* Tasks */
void vTaskStateEst(void *argument);

#endif /* INC_TASKS_TASK_STATE_EST_H_ */
