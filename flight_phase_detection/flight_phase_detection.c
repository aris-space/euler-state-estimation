#include "flight_phase_detection.h"

void detect_flight_phase(flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data, env *env)
{   


    /* determine state transition events */
    switch (flight_phase_detection->flight_phase) {
        case IDLE:
            if (((float)(state_est_data->acceleration_rocket[0])) / 1000 > 20) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = THRUSTING;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        case THRUSTING:
            if (((float)(state_est_data->acceleration_rocket[0])) / 1000 < 0) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = COASTING;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;
        
        case COASTING:
            if (((float)(state_est_data->velocity_world[2])) / 1000 < 0) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        case DESCENT:
            if (((float)(state_est_data->position_world[2])) / 1000 < 20) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = RECOVERY;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        default:
        break;
    }

    flight_phase_detection->mach_number = mach_number(env, ((float) state_est_data->velocity_rocket[0]) / 1000);

    /* determine the mach regime */
    if (flight_phase_detection->mach_number >= 1.3) {
        flight_phase_detection->mach_regime = SUPERSONIC;
    } else if (flight_phase_detection->mach_number >= 0.8)
    {
        flight_phase_detection->mach_regime = TRANSONIC;
    } else
    {
        flight_phase_detection->mach_regime = SUBSONIC;
    }
    
    
}

void reset_flight_phase_detection(flight_phase_detection_t *flight_phase_detection){
    flight_phase_detection->flight_phase = IDLE;
    flight_phase_detection->mach_regime = SUBSONIC;
    flight_phase_detection->mach_number = 0.0;
    flight_phase_detection->num_samples_positive = 0;
}