#include "../../Inc/Sim_Con/flight_phase_detection.h"

void detect_flight_phase(flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data)
{   


    /* determine state transition events */
    switch (flight_phase_detection->flight_phase) {
        case IDLE:
            if (((float)(state_est_data->acceleration_rocket[0])) / 1000 > FPD_LIFTOFF_ACC_THRESH) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = THRUSTING;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
            else if (((float)(state_est_data->position_world[2])) / 1000 > FPD_LIFTOFF_ALT_THRESH) {
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
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        case DROGUE_DESCENT:
            if (((float)(state_est_data->altitude_raw) / 1000) < FPD_MAIN_DESCENT_ALT_THRESH && state_est_data->altitude_raw_active == true) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = MAIN_DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
            /* we assume a ballistic descent when the absolute velocity of the rocket in vertical direction is larger than 75 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) > FPD_BALLISTIC_VEL_THRESH_HIGH) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = BALLISTIC_DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;
        
        case MAIN_DESCENT:
            /* we assume a touchdown event when the absolute value of the altitude is smaller than 400m 
               and the absolute velocity of the rocket is smaller than 2 m/s */
            if (fabs(((float)(state_est_data->velocity_rocket[0])) / 1000) < FPD_TOUCHDOWN_VEL_THRESH 
                && fabs(((float)(state_est_data->position_world[2])) / 1000) < FPD_TOUCHDOWN_ALT_THRESH) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->num_samples_positive = 0;
                }
            } /* we assume a ballistic descent when the absolute velocity of the rocket in vertical direction is larger than 75 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) > FPD_BALLISTIC_VEL_THRESH_HIGH) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = BALLISTIC_DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        case BALLISTIC_DESCENT:
            /* we assume a touchdown event when the absolute value of the altitude is smaller than 400m 
               and the absolute velocity of the rocket is smaller than 2 m/s */
            if (fabs(((float)(state_est_data->velocity_rocket[0])) / 1000) < FPD_TOUCHDOWN_VEL_THRESH 
                && fabs(((float)(state_est_data->position_world[2])) / 1000) < FPD_TOUCHDOWN_ALT_THRESH) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
            /* we assume a normal descent with parachute when the absolute velocity of the rocket in vertical direction is smaller than 40 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) < FPD_BALLISTIC_VEL_THRESH_LOW) {
                flight_phase_detection->num_samples_positive += 1;
                if (flight_phase_detection->num_samples_positive >= 4) {
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->num_samples_positive = 0;
                }
            }
        break;

        default:
        break;
    }

    flight_phase_detection->mach_number = (float)(state_est_data->mach_number) / 1000000;

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