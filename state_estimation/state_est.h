#include "../math_utils.h"
#include "../environment/env.h"
#include "state_est_const.h"
#include "kf.h"
#include "../flight_phase_detection/flight_phase_detection.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifndef STATE_EST_H_
#define STATE_EST_H_

typedef struct extrapolation_rolling_memory {
    int memory_length;
    float timestamps[MAX_LENGTH_ROLLING_MEMORY];
    float measurements[MAX_LENGTH_ROLLING_MEMORY];
    float noise_stdev;
    double polyfit_coeffs[EXTRAPOLATION_POLYFIT_DEGREE+1]; /* array size needs to be the degree of the polyfit plus 1 */
} extrapolation_rolling_memory;

void calibrate_state_est(float p_g, float T_g, flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data, 
                         env *env, kf_state *kf_state, extrapolation_rolling_memory *baro_roll_mem);

void update_state_est_data(state_est_data_t *state_est_data, kf_state *kf_state);

void process_measurements(timestamp_t t, kf_state *kf_state, state_est_meas_t *state_est_meas, state_est_meas_t *state_est_meas_prior, 
                          env *env, extrapolation_rolling_memory *extrapolation_rolling_memory);

void select_noise_models(kf_state *kf_state, flight_phase_detection_t *flight_phase_detection, env *env,
                         extrapolation_rolling_memory *extrapolation_rolling_memory);

void sensor_elimination_by_stdev(int n, float measurements[n], bool measurement_active[n]);
void sensor_elimination_by_extrapolation(timestamp_t t, int n, float measurements[n], bool measurement_active[n], 
                                         extrapolation_rolling_memory *extrapolation_rolling_memory);

#endif