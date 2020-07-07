#include "../Util/util.h"

#ifndef STATE_EST_CONST_H
#define STATE_EST_CONST_H

#define STATE_ESTIMATION_FREQUENCY 100
#define NUM_SENSORBOARDS 3
#define MAX_NUM_SENSORS 3 // max between NUM_IMU and NUM_BAROMETER
#define NUM_IMU 3
#define NUM_BARO 3
#define MAX_LENGTH_ROLLING_MEMORY 18

#define USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION 0 // set to 1 to activate sensor elimination by extrapolation for barometer and temperature
#define EXTRAPOLATION_POLYFIT_DEGREE 2


typedef struct {
	float pressure;
	float temperature;
	timestamp_t ts;
} baro_state_est_t;

typedef struct {
	float gyro_x, gyro_y, gyro_z;
	float acc_x, acc_y, acc_z;
	timestamp_t ts;
} imu_state_est_t;

/* State Estimation combined Data struct */
typedef struct {
    /* pressure in [Pa] and temperature in [°C] */
	baro_state_est_t baro_data[NUM_SENSORBOARDS];
    /* acceleration in [m/s^2] and angular velocity in [rad/s] */
    /* all in rocket frame where x-dir is along length of rocket */
	imu_state_est_t imu_data[NUM_SENSORBOARDS];
} state_est_meas_t;

#endif