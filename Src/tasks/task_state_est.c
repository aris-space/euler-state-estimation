/*
 * task_state_est.c
 *
 *  Created on: Nov 29, 2019
 *      Author: Jonas
 */

#include "tasks/task_state_est.h"

void resetStateEstimation(kf_state_t *kf_state, flight_phase_detection_t *flight_phase_detection,
	env_t *environment, extrapolation_rolling_memory_t *extrapolation_rolling_memory,
	float pressure, float temperature);


void vTaskStateEst(void *argument) {

	/* For periodic update */
	uint32_t tick_count, tick_update;


	/* Initialise Variables */
	env_t env;
	init_env(&env);

	state_est_meas_t state_est_meas = { 0 };
	state_est_meas_t state_est_meas_prior = { 0 };

	kf_state_t kf_state;
	reset_kf_state(&kf_state);

	extrapolation_rolling_memory_t extrapolation_rolling_memory = { 0 };
	extrapolation_rolling_memory.memory_length = 0;

	flight_phase_detection_t flight_phase_detection = { 0 };
	reset_flight_phase_detection(&flight_phase_detection);

	command_e telemetry_command = IDLE_COMMAND;

	select_noise_models(&kf_state, &flight_phase_detection, &env, &extrapolation_rolling_memory);

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
			resetStateEstimation(&kf_state, &flight_phase_detection, &env, &extrapolation_rolling_memory, average_press, average_temp);
		}

		/* Reset the whole thing automatically after 30 Seconds of running */
		if (reset_counter > 30 * STATE_ESTIMATION_FREQUENCY && !was_reset) {
			resetStateEstimation(&kf_state, &flight_phase_detection, &env, &extrapolation_rolling_memory, average_press, average_temp);
			was_reset = true;
		}
		reset_counter++;


		/* Acquire the Sensor data */

		/* Sensor Board 1 */
		ReadMutexStateEst(&sb1_mutex, &sb1_baro, &sb1_imu, &state_est_meas, 1);

		/* Sensor Board 2 */
		ReadMutexStateEst(&sb2_mutex, &sb2_baro, &sb2_imu, &state_est_meas, 2);

		/* Sensor Board 3 */
		ReadMutexStateEst(&sb3_mutex, &sb3_baro, &sb3_imu, &state_est_meas, 3);

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
		ReadMutex(&fsm_mutex, &global_flight_phase_detection, &flight_phase_detection, sizeof(flight_phase_detection));

		/* process measurements */
		process_measurements(tick_count, &kf_state, &state_est_meas, &state_est_meas_prior, &env, &extrapolation_rolling_memory);

		/* select noise models (dependent on detected flight phase and updated temperature in environment) */
		select_noise_models(&kf_state, &flight_phase_detection, &env, &extrapolation_rolling_memory);

		/* Start Kalman Update */

		/* Prediction Step */
		kf_prediction(&kf_state);

		/* update Step */
		if (kf_state.num_z_active > 0) {
			select_kf_observation_matrices(&kf_state);
			kf_update(&kf_state);
		}
		else
		{
			memcpy(kf_state.x_est, kf_state.x_priori, sizeof(kf_state.x_priori));
		}

		/* set measurement prior to measurements from completed state estimation step */
		memcpy(&state_est_meas_prior, &state_est_meas, sizeof(state_est_meas));

		/* Kalman Update Finished */

		/* Update global State Estimation Data */
		if (AcquireMutex(&state_est_mutex) == osOK) {
			update_state_est_data(&state_est_data_global, &kf_state);
			ReleaseMutex(&state_est_mutex);
		}

		UsbPrint("[DBG] Height: %d; Velocity: %d; t: %lu\n", state_est_data_global.position_world[2],
			state_est_data_global.velocity_world[2], tick_count);

		/* Update env for FSM */
		if (AcquireMutex(&fsm_mutex) == osOK) {
			global_env = env;
			ReleaseMutex(&fsm_mutex);
		}

		/* Write to logging system */
		logEstimatorVar(osKernelGetTickCount(), state_est_data_global);

		/* TODO: Check if the state estimation can do this for the given frequency */

		osDelayUntil(tick_count);
	}
}


void resetStateEstimation(kf_state_t *kf_state, flight_phase_detection_t *flight_phase_detection,
	env_t *environment, extrapolation_rolling_memory_t *extrapolation_rolling_memory, float pressure, float temperature) {
	reset_flight_phase_detection(flight_phase_detection);
	calibrate_env(environment, pressure, temperature);
	update_env(environment, temperature);
	reset_kf_state(kf_state);
	*extrapolation_rolling_memory = EMPTY_MEMORY;
	select_noise_models(kf_state, flight_phase_detection, environment, extrapolation_rolling_memory);
}
