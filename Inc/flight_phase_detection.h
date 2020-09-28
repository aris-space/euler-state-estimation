#include "state_est_const.h"
#include "env.h"
#include <stdbool.h>

#ifndef FLIGHT_PHASE_DETECTION_H_
#define FLIGHT_PHASE_DETECTION_H_

void detect_flight_phase(timestamp_t t, flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data);

void reset_flight_phase_detection(flight_phase_detection_t *flight_phase_detection);

#endif