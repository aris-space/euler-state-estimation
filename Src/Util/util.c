/*
 * util.c
 *
 *  Created on: Feb 24, 2020
 *      Author: stoja
 */

#include "usbd_cdc_if.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <Util/util.h>

/** LOGGING SECTION **/

/*
 *  %c	character
 %d	decimal (integer) number (base 10)
 %e	exponential floating-point number
 %f	floating-point number
 %i	integer (base 10)
 %o	octal number (base 8)
 %s	a string of characters
 %u	unsigned decimal (integer) number
 %x	number in hexadecimal (base 16)
 %%	print a percent sign
 \%	print a percent sign
 */

osStatus_t logSensor(timestamp_t ts, board_id_t sensor_board_id,
		sensor_type_e sensor_type, void *sensor_data) {
	log_entry_t log_entry = { 0 };

	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%hi,%d,", ts, SENSOR,
			sensor_board_id, sensor_type);

	switch (sensor_type) {
	case BARO: {
		baro_data_t *baro_data_ptr = (baro_data_t*) sensor_data;
		snprintf(log_entry.str + strlen(log_entry.str),
		LOG_BUFFER_LEN, "%ld,%ld,%lu\n", baro_data_ptr->pressure,
				baro_data_ptr->temperature, baro_data_ptr->ts);
	}
		break;
	case IMU: {
		imu_data_t *imu_data_ptr = (imu_data_t*) sensor_data;
		snprintf(log_entry.str + strlen(log_entry.str),
		LOG_BUFFER_LEN, "%hd,%hd,%hd,%hd,%hd,%hd,%lu\n", imu_data_ptr->acc_x,
				imu_data_ptr->acc_y, imu_data_ptr->acc_z, imu_data_ptr->gyro_x,
				imu_data_ptr->gyro_y, imu_data_ptr->gyro_z, imu_data_ptr->ts);
	}
		break;
	case GPS: {
		gps_data_t *gps_data = (gps_data_t*) sensor_data;
		snprintf(log_entry.str + strlen(log_entry.str),
		LOG_BUFFER_LEN, "%ld,%ld,%ld,%d,%ld,%d,%ld,%d,%hd,%hd\n",
			gps_data->hour, gps_data->minute, gps_data->second, gps_data->lat_deg,
			gps_data->lat_decimal, gps_data->lon_deg, gps_data->lon_decimal, gps_data->satellite,
			gps_data->altitude, gps_data->HDOP);
	}
		break;
	case BATTERY: {
		battery_data_t *battery_data = (battery_data_t*) sensor_data;
		snprintf(log_entry.str + strlen(log_entry.str),
		LOG_BUFFER_LEN, "%hd,%hd,%hd,%hd\n",
			battery_data->battery, battery_data->consumption,
			battery_data->current, battery_data->supply);
	}
		break;
	default:
		snprintf(log_entry.str + strlen(log_entry.str),
		LOG_BUFFER_LEN, "Unknown sensor type\n");
		break;
	}

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logRocketState(timestamp_t ts, flight_phase_detection_t flight_phase_detection) {
	log_entry_t log_entry = { 0 };

	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%d\n", ts, STATE,
			flight_phase_detection.flight_phase);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logEstimatorVar(timestamp_t ts, state_est_data_t estimator_data) {
	log_entry_t log_entry = { 0 };
	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%ld,%ld,%ld\n", ts, ESTIMATOR_VAR,
			estimator_data.position_world[2], estimator_data.velocity_rocket[0], estimator_data.acceleration_rocket[0]);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logControllerOutput(timestamp_t ts, int32_t controller_output, int32_t reference_error,
		int32_t integrated_error) {
	log_entry_t log_entry = { 0 };
	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%ld,%ld,%ld\n", ts, CONTROLLER_OUTPUT,
			controller_output, reference_error, integrated_error);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logMotor(timestamp_t ts, int32_t desired_position, int32_t actual_position) {
	log_entry_t log_entry = { 0 };
	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%ld,%ld\n", ts, MOTOR_POSITION,
			desired_position, actual_position);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logBattery(timestamp_t ts, int32_t desired_position, int32_t actual_position) {
	log_entry_t log_entry = { 0 };
	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%ld,%ld\n", ts, MOTOR_POSITION,
			desired_position, actual_position);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}

osStatus_t logMsg(timestamp_t ts, char *msg) {
	log_entry_t log_entry = { 0 };

	snprintf(log_entry.str, LOG_BUFFER_LEN, "%lu;%d;%s\n", ts, MSG, msg);

	return osMessageQueuePut(log_queue, &log_entry, 0U, 0U);
}


/** USB DEBUGGING SECTION **/

uint8_t UsbPrint(const char *format, ...) {
	uint8_t ret = 1;
#ifdef DEBUG
	if (osMutexAcquire(print_mutex, 0U) == osOK) {
		va_list argptr;
		va_start(argptr, format);
		vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
		va_end(argptr);
		ret = CDC_Transmit_FS((uint8_t*) print_buffer, strlen(print_buffer));
		osMutexRelease(print_mutex);
	}
#endif
	return ret;
}
