/*
 * task_state_est.c
 *
 *  Created on: Nov 29, 2019
 *      Author: Jonas
 */

#include "tasks/task_state_est.h"


void vTaskStateEst(void *argument) {

	/* For periodic update */
	uint32_t tick_count, tick_update;


	/* Initialise Variables */
	state_est_state_t state_est_state = { 0 };
	init_state_est_state(&state_est_state);

	command_e telemetry_command = IDLE_COMMAND;

	/* average Temperature */
	float average_temp = 0;
	float sum_temp = 0;
	/* average Pressure */
	float average_press = 0;
	float sum_press = 0;
	uint16_t calibrate_count = 0;

	/* reset counter */
	uint32_t reset_counter = 0;
	bool was_reset = false;

	osDelay(900);


	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / STATE_ESTIMATION_FREQUENCY;

	for (;;) {
		tick_count += tick_update;

		/* Acquire New Command */
		ReadMutex(&command_mutex, &global_telemetry_command, &telemetry_command, sizeof(global_telemetry_command));

		/*
		 * Check if we need to reset the state estimation
		 * and if we are in idle state to be able
		 * to do so
		 */
		if (flight_phase_detection.flight_phase == IDLE && global_telemetry_command == CALIBRATE_SENSORS) {
			reset_state_est_state(average_press, average_temp, &state_est_state);
		}

		/* Reset the whole thing automatically after 30 Seconds of running */
		if (reset_counter > 30 * STATE_ESTIMATION_FREQUENCY && !was_reset) {
			reset_state_est_state(average_press, average_temp, &state_est_state);
			was_reset = true;
		}
		reset_counter++;


		/* Acquire the Sensor data */

		/* Sensor Board 1 */
		ReadMutexStateEst(&sb1_mutex, &sb1_baro, &sb1_imu, &state_est_state->state_est_meas, 1);

		/* Sensor Board 2 */
		ReadMutexStateEst(&sb2_mutex, &sb2_baro, &sb2_imu, &state_est_state->state_est_meas, 2);

		/* Sensor Board 3 */
		ReadMutexStateEst(&sb3_mutex, &sb3_baro, &sb3_imu, &state_est_state->state_est_meas, 3);

		/* calculate averaging */
		if (flight_phase_detection.flight_phase == IDLE) {
			sum_press += (float)(sb1_baro.pressure + sb2_baro.pressure + sb3_baro.pressure);
			sum_temp += ((float)(sb1_baro.temperature + sb2_baro.temperature + sb3_baro.temperature)) / 100;
			calibrate_count += 3;
			if (calibrate_count > 150) {
				average_press = sum_press / (float)calibrate_count;
				average_temp = sum_temp / (float)calibrate_count;
				sum_press = 0;
				sum_temp = 0;
				calibrate_count = 0;
			}
		}

		/* get new Phase Detection*/
		ReadMutex(&fsm_mutex, &global_flight_phase_detection, &state_est_state->flight_phase_detection, sizeof(state_est_state->flight_phase_detection));

		state_est_step(tick_count, &state_est_state, false);

		/* Update global State Estimation Data */
		if (AcquireMutex(&state_est_mutex) == osOK) {
			update_state_est_data(&state_est_data_global, &state_est_state->kf_state);
			ReleaseMutex(&state_est_mutex);
		}

		UsbPrint("[DBG] Height: %d; Velocity: %d; t: %lu\n", state_est_data_global.position_world[2],
			state_est_data_global.velocity_world[2], tick_count);

		/* Update env for FSM */
		if (AcquireMutex(&fsm_mutex) == osOK) {
			global_env = state_est_state->env;
			ReleaseMutex(&fsm_mutex);
		}

		/* Write to logging system */
		logEstimatorVar(osKernelGetTickCount(), state_est_data_global);

		osDelayUntil(tick_count);
	}
}
