/*
 * mutex.c
 *
 *  Created on: Jun 19, 2020
 *      Author: Jonas
 */
#include "Util/mutex.h"

osStatus_t AcquireMutex(custom_mutex_t *custom_mutex){
	osStatus_t status = osOK;
	status = osMutexAcquire(custom_mutex->mutex, 10);
	if(status == osOK){
		custom_mutex->counter++;
	}

	return status;
}

osStatus_t ReleaseMutex(custom_mutex_t *custom_mutex){
	return osMutexRelease(custom_mutex->mutex);
}

osStatus_t ReadMutex(custom_mutex_t *custom_mutex,void const* global_data, void* const local_data, int32_t size){
	uint8_t buffer[100] = { 0 };
	uint32_t counter = custom_mutex->counter;
	osStatus_t status = osError;
	for(int i = 0; i < 5; i++){
		memcpy(&buffer[0], global_data, size);
		if(custom_mutex->counter == counter){
			memcpy(local_data, buffer, size);
			status = osOK;
			break;
		}
		counter = custom_mutex->counter;
	}
	return status;
}

osStatus_t ReadMutexStateEst(custom_mutex_t *custom_mutex, baro_data_t *baro, imu_data_t *imu, state_est_meas_t *state, uint32_t sb_number){
	/* Buffer */
	uint32_t Placeholder_timestamps[2] = { 0 };
	float Placeholder_measurement[3] = { 0 };

	/* Status */
	osStatus_t status = osError;

	/* Counter */
	uint32_t counter = custom_mutex->counter;

	for(int i = 0; i < 5; i++){
		/* Write in Buffer */
		Placeholder_measurement[0] = (float) (baro->pressure);
		Placeholder_timestamps[0] = baro->ts;
		Placeholder_measurement[1] = ((float) (imu->acc_z)) / 1024;
		Placeholder_timestamps[1] = imu->ts;
		Placeholder_measurement[2] = ((float) (baro->temperature)) / 100;

		/* Check if Mutex was aquired */
		if(custom_mutex->counter == counter){
			state->baro_data[sb_number-1].pressure = Placeholder_measurement[0];
			state->baro_data[sb_number-1].temperature = Placeholder_measurement[2];
			state->baro_data[sb_number-1].ts = Placeholder_timestamps[0];

			state->imu_data[sb_number-1].acc_x = Placeholder_measurement[1] * GRAVITATION;
			state->imu_data[sb_number-1].ts = Placeholder_timestamps[1];
			status = osOK;
			break;
		}
		counter = custom_mutex->counter;
	}


	return status;
}
