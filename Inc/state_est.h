#include "Util/math_utils.h"
#include "env.h"
#include "state_est_const.h"
#include "kf.h"
#include "flight_phase_detection.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifndef STATE_EST_H_
#define STATE_EST_H_

/* moving average memory */
typedef struct state_est_processed_measurements_t {
    float angular_velocity_world[3];
} state_est_processed_measurements_t;

typedef struct extrapolation_rolling_memory_t {
    int memory_length;
    float timestamps[MAX_LENGTH_ROLLING_MEMORY];
    float measurements[MAX_LENGTH_ROLLING_MEMORY];
    float noise_stdev;
    double polyfit_coeffs[EXTRAPOLATION_POLYFIT_DEGREE+1]; /* array size needs to be the degree of the polyfit plus 1 */
} extrapolation_rolling_memory_t;

/* moving average memory */
typedef struct mav_memory_t {
    int memory_length;
    timestamp_t timestamps[MAX_LENGTH_MOVING_AVERAGE];
    float values[MAX_LENGTH_MOVING_AVERAGE];
    float avg_values[MAX_LENGTH_MOVING_AVERAGE];
} mav_memory_t;

typedef struct state_est_state_t {
    state_est_data_t state_est_data;
    state_est_meas_t state_est_meas;
    state_est_meas_t state_est_meas_prior;
    state_est_processed_measurements_t processed_measurements;
    kf_state_t kf_state;
    env_t env;
    flight_phase_detection_t flight_phase_detection;

    #if defined(USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION) && USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION == true
    extrapolation_rolling_memory_t baro_roll_mem;
    #endif

    #if STATE_ESTIMATION_TYPE == 1 && USE_STATE_EST_DESCENT == false
    mav_memory_t altitude_mav_mem;
    #endif
} state_est_state_t;

void reset_state_est_state(float p_g, float T_g, state_est_state_t *state_est_state);

void state_est_step(timestamp_t t, state_est_state_t *state_est_state, bool bool_detect_flight_phase);

void update_state_est_data(state_est_state_t *state_est_state);

void process_measurements(timestamp_t t, state_est_state_t *state_est_state);
void select_noise_models(state_est_state_t *state_est_state);

void sensor_elimination_by_stdev(int n, float measurements[n], bool measurement_active[n]);
void sensor_elimination_by_extrapolation(timestamp_t t, int n, float measurements[n], bool measurement_active[n], 
						                 extrapolation_rolling_memory_t *extrapolation_rolling_memory);

float update_mav(mav_memory_t *mav_memory, timestamp_t t, float measurement, bool measurement_active);

#if STATE_ESTIMATION_TYPE == 2
void init_sensor_transformation_matrix(state_est_state_t *state_est_state);
#endif

#endif