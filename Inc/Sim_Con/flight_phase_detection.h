#include "../Util/util.h"
#include "env.h"

#ifndef FLIGHT_PHASE_DETECTION_H_
#define FLIGHT_PHASE_DETECTION_H_

enum flight_phase_e {
	IDLE = 0,
	AIRBRAKE_TEST = 5,
    THRUSTING = 10,
    COASTING = 11,
    DESCENT = 15,
    RECOVERY = 20,
};

enum mach_regime_e {
    SUBSONIC = 0,
    TRANSONIC = 1,
    SUPERSONIC = 2,
};

typedef struct flight_phase_detection_t {
    enum flight_phase_e flight_phase;
    enum mach_regime_e mach_regime;
    float mach_number;
    int num_samples_positive;
} flight_phase_detection_t;


void detect_flight_phase(flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data, env_t *env);

void reset_flight_phase_detection(flight_phase_detection_t *flight_phase_detection);

#endif