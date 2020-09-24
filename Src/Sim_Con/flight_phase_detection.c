#include "../../Inc/Sim_Con/flight_phase_detection.h"

void detect_flight_phase(timestamp_t t, flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data)
{   
    /* timestamp_t t needs to be the tick time in [ms] */

    /* determine state transition events */
    switch (flight_phase_detection->flight_phase) {
        case IDLE:
            if (((float)(state_est_data->acceleration_rocket[0])) / 1000 > FPD_LIFTOFF_ACC_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = THRUSTING;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
            else if (((float)(state_est_data->position_world[2])) / 1000 > FPD_LIFTOFF_ALT_THRESH) {
                flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= 4) {
                    flight_phase_detection->flight_phase = THRUSTING;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
        break;

        case THRUSTING:
            if (((float)(state_est_data->acceleration_rocket[0])) / 1000 < 0) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = COASTING;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
        break;
        
        case COASTING:
            #ifdef FPD_CONTROL_ACTIVE
                if (flight_phase_detection->mach_number < FPD_CONTROL_ACTIVATION_MACH_NUMBER) {
                    flight_phase_detection->safety_counter[0] += 1;
                    if (flight_phase_detection->safety_counter[0] >= 4) {
                        flight_phase_detection->flight_phase = CONTROL;
                        flight_phase_detection->safety_counter[0] = 0;
                        flight_phase_detection->safety_counter[1] = 0;
                    }
                }
            #else
                if (((float)(state_est_data->velocity_world[2])) / 1000 < 0) {
                    flight_phase_detection->safety_counter[0] += 1;
                    if (flight_phase_detection->safety_counter[0] >= 4) {
                        flight_phase_detection->flight_phase = DROGUE_DESCENT;
                        flight_phase_detection->safety_counter[0] = 0;
                        flight_phase_detection->safety_counter[1] = 0;
                    }
                }
            #endif
        break;

        case CONTROL:
            #if defined(FPD_CONTROL_ACTIVE) && defined(FPD_BIAS_RESET_ACTIVATION_MACH_NUMBER)
                #if defined(FPD_BIAS_RESET_TIME)
                    if (FPD_BIAS_RESET_TIME > 0) {
                         if (flight_phase_detection->mach_number < FPD_CONTROL_DEACTIVATION_MACH_NUMBER) {
                            flight_phase_detection->safety_counter[0] += 1;
                            if (flight_phase_detection->safety_counter[0] >= 4) {
                                flight_phase_detection->flight_phase = APOGEE_APPROACH;
                                flight_phase_detection->safety_counter[0] = 0;
                                flight_phase_detection->safety_counter[1] = 0;
                            }
                        }
                        else if (flight_phase_detection->mach_number < FPD_BIAS_RESET_ACTIVATION_MACH_NUMBER && 
                            flight_phase_detection->t_bias_reset_start == -1) {
                            flight_phase_detection->safety_counter[1] += 1;
                            if (flight_phase_detection->safety_counter[1] >= 4) {
                                flight_phase_detection->flight_phase = BIAS_RESET;
                                flight_phase_detection->safety_counter[0] = 0;
                                flight_phase_detection->safety_counter[1] = 0;
                                flight_phase_detection->t_bias_reset_start = t;
                            }
                        }
                    } else {
                        if (flight_phase_detection->mach_number < FPD_CONTROL_DEACTIVATION_MACH_NUMBER) {
                            flight_phase_detection->safety_counter[0] += 1;
                            if (flight_phase_detection->safety_counter[0] >= 4) {
                                flight_phase_detection->flight_phase = APOGEE_APPROACH;
                                flight_phase_detection->safety_counter[0] = 0;
                                flight_phase_detection->safety_counter[1] = 0;
                            }
                        }
                    }
                #else
                    if (flight_phase_detection->mach_number < FPD_CONTROL_DEACTIVATION_MACH_NUMBER) {
                        flight_phase_detection->safety_counter[0] += 1;
                        if (flight_phase_detection->safety_counter[0] >= 4) {
                            flight_phase_detection->flight_phase = APOGEE_APPROACH;
                            flight_phase_detection->safety_counter[0] = 0;
                            flight_phase_detection->safety_counter[1] = 0;
                        }
                    }
                #endif
            #endif
        break;

        case BIAS_RESET:
            #ifdef FPD_BIAS_RESET_TIME
                if (t > (flight_phase_detection->t_bias_reset_start + FPD_BIAS_RESET_TIME * 1000)) {
                    flight_phase_detection->safety_counter[0] += 1;
                    if (flight_phase_detection->safety_counter[0] >= 4) {
                        flight_phase_detection->flight_phase = CONTROL;                        
                        flight_phase_detection->safety_counter[0] = 0;
                        flight_phase_detection->safety_counter[1] = 0;
                    }
                }
            #endif
        break;

        case APOGEE_APPROACH:
            if (((float)(state_est_data->velocity_world[2])) / 1000 < 0) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
        break;

        case DROGUE_DESCENT:
            if (((float)(state_est_data->altitude_raw) / 1000) < FPD_MAIN_DESCENT_ALT_THRESH && state_est_data->altitude_raw_active == true) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = MAIN_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
            /* we assume a ballistic descent when the absolute velocity of the rocket in vertical direction is larger than 75 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) > FPD_BALLISTIC_VEL_THRESH_HIGH) {
                flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= 4) {
                    flight_phase_detection->flight_phase = BALLISTIC_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
        break;
        
        case MAIN_DESCENT:
            /* we assume a touchdown event when the absolute value of the altitude is smaller than 400m 
               and the absolute velocity of the rocket is smaller than 2 m/s */
            if (fabs(((float)(state_est_data->velocity_rocket[0])) / 1000) < FPD_TOUCHDOWN_VEL_THRESH 
                && fabs(((float)(state_est_data->position_world[2])) / 1000) < FPD_TOUCHDOWN_ALT_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            } /* we assume a ballistic descent when the absolute velocity of the rocket in vertical direction is larger than 75 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) > FPD_BALLISTIC_VEL_THRESH_HIGH) {
                flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= 4) {
                    flight_phase_detection->flight_phase = BALLISTIC_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
        break;

        case BALLISTIC_DESCENT:
            /* we assume a touchdown event when the absolute value of the altitude is smaller than 400m 
               and the absolute velocity of the rocket is smaller than 2 m/s */
            if (fabs(((float)(state_est_data->velocity_rocket[0])) / 1000) < FPD_TOUCHDOWN_VEL_THRESH 
                && fabs(((float)(state_est_data->position_world[2])) / 1000) < FPD_TOUCHDOWN_ALT_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= 4) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                }
            }
            /* we assume a normal descent with parachute when the absolute velocity of the rocket in vertical direction is smaller than 40 m/s */
            else if (fabs(((float)(state_est_data->velocity_world[2])) / 1000) < FPD_BALLISTIC_VEL_THRESH_LOW) {
                flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= 4) {
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
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
    flight_phase_detection->safety_counter[0] = 0;
    flight_phase_detection->safety_counter[1] = 0;
    flight_phase_detection->t_bias_reset_start = -1;
}