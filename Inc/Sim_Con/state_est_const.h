#include "state_est_config.h"

#ifndef STATE_EST_CONST_H
#define STATE_EST_CONST_H

/** BASIC TYPES **/
#ifdef EULER_REC
#include "main.h"
typedef uint32_t timestamp_t;
#endif

#ifdef EULER_AV
#include "../Util/util.h"
#endif

#ifdef EULER_SIMCON
/* as we don't include cmsis_os.h, we need to define some datatypes ourselves */
typedef u_int8_t uint8_t;
typedef u_int16_t uint16_t;
typedef u_int32_t uint32_t;
/* Timestamp */
typedef uint32_t timestamp_t;
#endif

typedef enum {
	IDLE = 1, AIRBRAKE_TEST, THRUSTING, COASTING, DROGUE_DESCENT, BALLISTIC_DESCENT, MAIN_DESCENT, TOUCHDOWN
} flight_phase_e;

/* Mach Regime */
typedef enum {
	SUBSONIC = 1, TRANSONIC, SUPERSONIC
} mach_regime_e;

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

/* State Estimation Output */
typedef struct {
	int32_t position_world[3];
	int32_t velocity_rocket[3];
	int32_t acceleration_rocket[3];
	int32_t velocity_world[3];
	int32_t acceleration_world[3];
	int32_t mach_number;
	int32_t altitude_raw;
	int32_t altitude_raw_active;
} state_est_data_t;

/* FSM States */
typedef struct {
	flight_phase_e flight_phase;
	mach_regime_e mach_regime;
	float mach_number;
	int8_t num_samples_positive;
} flight_phase_detection_t;

#endif