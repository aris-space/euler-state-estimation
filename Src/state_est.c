#include "../Inc/state_est.h"

void reset_state_est_state(float p_g, float T_g, state_est_state_t *state_est_state) {
    reset_flight_phase_detection(&state_est_state->flight_phase_detection);

    memset(&state_est_state->state_est_data, 0, sizeof(state_est_state->state_est_data));
    memset(&state_est_state->state_est_meas, 0, sizeof(state_est_state->state_est_meas));
    memset(&state_est_state->state_est_meas_prior, 0, sizeof(state_est_state->state_est_meas_prior));

    init_env(&state_est_state->env);
    calibrate_env(&state_est_state->env, p_g, T_g);
    update_env(&state_est_state->env, T_g);

	reset_kf_state(&state_est_state->kf_state);
    update_state_est_data(&state_est_state->state_est_data, &state_est_state->kf_state, &state_est_state->env);

    memset(&state_est_state->baro_roll_mem, 0, sizeof(state_est_state->baro_roll_mem));

	select_noise_models(&state_est_state->kf_state, &state_est_state->flight_phase_detection, 
                        &state_est_state->env, &state_est_state->baro_roll_mem);
}

void state_est_step(timestamp_t t, state_est_state_t *state_est_state, bool bool_detect_flight_phase) {
    /* process measurements */
	process_measurements(t, state_est_state);

	/* select noise models (dependent on detected flight phase and updated temperature in environment) */
	select_noise_models(&state_est_state->kf_state, &state_est_state->flight_phase_detection,
                        &state_est_state->env, &state_est_state->baro_roll_mem);
	
	kf_prediction(&state_est_state->kf_state);

	if (state_est_state->kf_state.num_z_active > 0) {
		select_kf_observation_matrices(&state_est_state->kf_state);
		kf_update(&state_est_state->kf_state);
	} else {
		memcpy(&state_est_state->kf_state.x_est, &state_est_state->kf_state.x_priori, sizeof(state_est_state->kf_state.x_priori));
	}

	update_state_est_data(&state_est_state->state_est_data, &state_est_state->kf_state, &state_est_state->env);

    if (EULER_REC == 1){
    	float p_avg, alt, velocity;
    	p_avg = update_mav(state_est_state->state_est_meas.baro_data[0].pressure, state_est_state->state_est_meas.baro_data[1].pressure);

    	if (state_est_state->flight_phase_detection.flight_phase >= DROGUE_DESCENT ){
			float p_a[1];
			p_a[0] = p_avg;
			bool p_ac[1] = {1};
			float h[1];
			printf("\n ------ p_a: %4.2f \n",p_a[0]);
			pressure2altitudeAGL(&state_est_state->env, 1, p_a, p_ac, h);
			printf("\n ------ h: %4.2f \n",h[0]);
			velocity = get_velocity(h[0],state_est_state->state_est_meas.baro_data[0].ts);
			alt = h[0];
			state_est_state->state_est_data.position_world[2] = (int32_t)(alt * 1000);
			state_est_state->state_est_data.velocity_rocket[0] = (int32_t)(velocity * 1000);
			state_est_state->state_est_data.velocity_world[2] = (int32_t)(velocity * 1000);
			state_est_state->state_est_data.acceleration_rocket[0] = 0;
			state_est_state->state_est_data.acceleration_world[2] = 0;
			state_est_state->state_est_data.mach_number = 0;
    	}
    }


    if (bool_detect_flight_phase){
        detect_flight_phase(t, &state_est_state->flight_phase_detection, &state_est_state->state_est_data);
    }

	/* set measurement prior to measurements from completed state estimation step */
	memcpy(&state_est_state->state_est_meas_prior, &state_est_state->state_est_meas, sizeof(state_est_state->state_est_meas));
}

void update_state_est_data(state_est_data_t *state_est_data, kf_state_t *kf_state, env_t *env) {
    state_est_data->position_world[2] = (int32_t)(kf_state->x_est[0] * 1000);
    state_est_data->velocity_rocket[0] = (int32_t)(kf_state->x_est[1] * 1000);
    state_est_data->velocity_world[2] = (int32_t)(kf_state->x_est[1] * 1000);
    state_est_data->acceleration_rocket[0] = (int32_t)(kf_state->u[0] * 1000);
    state_est_data->acceleration_world[2] = (int32_t)(kf_state->u[0] * 1000);
    state_est_data->mach_number = (int32_t)(mach_number(env, kf_state->x_est[1]) * 1000000);

}

void process_measurements(timestamp_t t, state_est_state_t *state_est_state) {
    float temp_meas[NUM_SENSORBOARDS];
    bool temp_meas_active[NUM_SENSORBOARDS];
    float acc_x_meas[NUM_SENSORBOARDS];
    bool acc_x_meas_active[NUM_SENSORBOARDS];

    for (int i = 0; i < NUM_SENSORBOARDS; i++){
        /* barometer */
        if (state_est_state->state_est_meas.baro_data[i].ts > state_est_state->state_est_meas_prior.baro_data[i].ts) {
            state_est_state->kf_state.z[i] = state_est_state->state_est_meas.baro_data[i].pressure;
            state_est_state->kf_state.z_active[i] = true;

            temp_meas[i] = state_est_state->state_est_meas.baro_data[i].temperature;
            temp_meas_active[i] = true;

            /* deactivate all barometer measurements if we are transsonic or supersonic */
            if (state_est_state->flight_phase_detection.mach_regime != SUBSONIC) {
                state_est_state->kf_state.z_active[i] = false;
            }

            /* deactivate all barometer measurements during control phase if required because of dynamic pressure */
            #ifdef USE_BARO_IN_CONTROL_PHASE
                if (USE_BARO_IN_CONTROL_PHASE == false) {
                    if (state_est_state->flight_phase_detection.flight_phase == CONTROL || 
                       (state_est_state->flight_phase_detection.flight_phase == BIAS_RESET && 
                            state_est_state->state_est_meas.airbrake_extension > BIAS_RESET_AIRBRAKE_EXTENSION_THRESH)) {
                        state_est_state->kf_state.z_active[i] = false;
                    }
                }
            #endif
        } else {
            state_est_state->kf_state.z[i] = 0;
            state_est_state->kf_state.z_active[i] = false;

            temp_meas[i] = 0;
            temp_meas_active[i] = false;
        }

        /* imu */
        if (state_est_state->state_est_meas.imu_data[i].ts > state_est_state->state_est_meas_prior.imu_data[i].ts) {
            acc_x_meas[i] = state_est_state->state_est_meas.imu_data[i].acc_x;
            acc_x_meas_active[i] = true;
        } else {
            acc_x_meas[i] = 0;
            acc_x_meas_active[i] = false;
        }
    }

    /* eliminate barometer measurements */
    if (USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION == 1) {
        if (state_est_state->baro_roll_mem.memory_length < MAX_LENGTH_ROLLING_MEMORY) {
            sensor_elimination_by_stdev(NUMBER_MEASUREMENTS, state_est_state->kf_state.z, state_est_state->kf_state.z_active);
        }
        sensor_elimination_by_extrapolation(t, NUMBER_MEASUREMENTS, state_est_state->kf_state.z, state_est_state->kf_state.z_active, &state_est_state->baro_roll_mem);
    } else {
        sensor_elimination_by_stdev(NUMBER_MEASUREMENTS, state_est_state->kf_state.z, state_est_state->kf_state.z_active);
    }

    /* eliminate temperature measurements */
    sensor_elimination_by_stdev(NUMBER_MEASUREMENTS, temp_meas, temp_meas_active);

    /* eliminate accelerometer in rocket x-dir measurements */
    sensor_elimination_by_stdev(NUMBER_MEASUREMENTS, acc_x_meas, acc_x_meas_active);

    /* update num_z_active */
    state_est_state->kf_state.num_z_active = 0;
    /* take the average of the active accelerometers in rocket-x dir as the state estimation input */
    float u = 0;
    int num_acc_x_meas_active = 0;

    /* take the average of the temperature measurement  */
    float temp_meas_mean = 0;
    int num_temp_meas_active = 0;
    
    for (int i = 0; i < NUMBER_MEASUREMENTS; i++){
        if (state_est_state->kf_state.z_active[i]){
            state_est_state->kf_state.num_z_active += 1;
        }
        if (acc_x_meas_active[i]) {
            u += acc_x_meas[i];
            num_acc_x_meas_active += 1;
        }
        if (temp_meas[i]) {
            temp_meas_mean += temp_meas[i];
            num_temp_meas_active += 1;
        }
    }

    pressure2altitudeAGL(&state_est_state->env, NUMBER_MEASUREMENTS, state_est_state->kf_state.z, state_est_state->kf_state.z_active, state_est_state->kf_state.z);

    /* compute the mean raw altitude from all barometer measurements */
    int num_alt_meas_active = 0;
    float alt_mean = 0;
    for (int i = 0; i < NUMBER_MEASUREMENTS; i++){
        if (state_est_state->kf_state.z_active[i]){
            num_alt_meas_active += 1;
            alt_mean += state_est_state->kf_state.z[i];
        }
    }
    if (num_alt_meas_active > 0) {
        alt_mean /= num_alt_meas_active;
        state_est_state->state_est_data.altitude_raw = (int32_t)(alt_mean * 1000);
        state_est_state->state_est_data.altitude_raw_active = true;
    } else {  
        state_est_state->state_est_data.altitude_raw_active = false;
    }

    /* we take the old acceleration from the previous timestep, if no acceleration measurements are active */
    if (num_acc_x_meas_active > 0){
        u /= num_acc_x_meas_active;
        /* gravity compensation for accelerometer */
        state_est_state->kf_state.u[0] = u - GRAVITATION;
    }
    
    if (num_temp_meas_active > 0){
        temp_meas_mean /= num_temp_meas_active;
        update_env(&state_est_state->env, temp_meas_mean);
    }

    /* airbrake extension tracking feedback */
    state_est_state->state_est_data.airbrake_extension = (int32_t)(state_est_state->state_est_meas.airbrake_extension * 1000000);
} 

void select_noise_models(kf_state_t *kf_state, flight_phase_detection_t *flight_phase_detection, env_t *env,
                        extrapolation_rolling_memory_t *baro_roll_mem){
    float accelerometer_x_stdev;
    float barometer_stdev;

    // TODO @maxi: add different noise models for each mach regime
    switch (flight_phase_detection->flight_phase) {
        case AIRBRAKE_TEST:
        case TOUCHDOWN:
        case IDLE:
            accelerometer_x_stdev = 0.0185409;
            barometer_stdev = 1.869;
        break;
        case THRUSTING:
            accelerometer_x_stdev = 1.250775;
            barometer_stdev = 13.000;
        break;
        case BIAS_RESET:
        case APOGEE_APPROACH:
        case CONTROL:
        case COASTING:
            accelerometer_x_stdev = 0.61803;
            barometer_stdev = 7.380;
        break;
        case DROGUE_DESCENT:
        case MAIN_DESCENT:
            accelerometer_x_stdev = 1.955133;
            barometer_stdev = 3.896;
        break;
        case BALLISTIC_DESCENT:
            accelerometer_x_stdev = 0.61803;
            barometer_stdev = 7.380;
        break;
    }

    for(int i = 0; i < NUMBER_PROCESS_NOISE; i++){
        kf_state->Q[i][i] = pow(accelerometer_x_stdev, 2);
    }

    float p[1];
    float h[1] = {kf_state->x_est[0]};
    bool h_active[1] = {true};
    altitudeAGL2pressure(env, 1, h, h_active, p);
    float h_grad = altitude_gradient(env, p[0]);
    float altitude_stdev = fabsf(barometer_stdev * h_grad);

    for(int i = 0; i < NUMBER_MEASUREMENTS; i++){
        kf_state->R[i][i] = pow(altitude_stdev, 2);
    }

    baro_roll_mem->noise_stdev = barometer_stdev;
}

void sensor_elimination_by_stdev(int n, float measurements[n], bool measurement_active[n]) {
    /* calculate mean of the sample */
    int num_active = 0;
    float mean = 0;
    for (int i = 0; i < n; i++){
        if (measurement_active[i]) {
            num_active += 1;
            mean += measurements[i];
        }
    }
    if (num_active > 0){
        mean /= num_active;
    }

    /* calculate the standard deviation of the sample */
    float stdev = 0;
    for (int i = 0; i < n; ++i) {
        if (measurement_active[i]) {
            stdev += pow(measurements[i] - mean, 2);
        }
    }
    if (num_active > 0){
        stdev = sqrt(stdev / num_active);
    }

    /* deactivate measurements if they are too far off the mean */
    for (int i = 0; i < n; ++i) {
        if (measurement_active[i]) {
            if (fabsf(measurements[i] - mean) > 2.0 * stdev) {
                measurement_active[i] = false;
            }
        }
    }
}

void sensor_elimination_by_extrapolation(timestamp_t t, int n, float measurements[n], bool measurement_active[n], 
                                         extrapolation_rolling_memory_t *extrapolation_rolling_memory){
    float x_priori = 0;
    /* we only extrapolate if the memory is fully populated. Otherwise we dont eliminate sensor and fill the memory */
    if (extrapolation_rolling_memory->memory_length >= MAX_LENGTH_ROLLING_MEMORY) {
        /* calculate coefficients for fit */
        polyfit(extrapolation_rolling_memory->timestamps, extrapolation_rolling_memory->measurements, 
                MAX_LENGTH_ROLLING_MEMORY, EXTRAPOLATION_POLYFIT_DEGREE, extrapolation_rolling_memory->polyfit_coeffs);
        /* extrapolate the value of the signal type at the current timestamp */
        for (int i = 0; i <= EXTRAPOLATION_POLYFIT_DEGREE; ++i) {
            x_priori += extrapolation_rolling_memory->polyfit_coeffs[i] * pow(t, i);
        }

        /* comparing and discarding outliers */

        /* deactivate measurements if they are too far off the mean */
        for (int i = 0; i < n; ++i) {
            if (measurement_active[i]) {
                float measurement_multiple = fabsf(measurements[i] - x_priori) / extrapolation_rolling_memory->noise_stdev;
                if (measurement_multiple > 1000.0) {
                    measurement_active[i] = false;
                }
            }
        }
    }
    else {}

    int num_active = 0;
    for (int i = 0; i < n; ++i) {
        if (measurement_active[i]) {
            num_active += 1;
        }
    }

    if (num_active > 0){
        /* shift existing elements in rolling memory back by the number of active new measurements we want to insert */
        for (int i = MAX_LENGTH_ROLLING_MEMORY-1; i >= num_active; --i) {

            extrapolation_rolling_memory->timestamps[i] = extrapolation_rolling_memory->timestamps[i-num_active];
            extrapolation_rolling_memory->measurements[i] = extrapolation_rolling_memory->measurements[i-num_active];
        }

        /* insert new measurements at the beginning of the memory */
        int idx_active = 0;
        for (int i = 0; i < n; ++i) {
            if (measurement_active[i]) {
                extrapolation_rolling_memory->timestamps[idx_active] = (float) t;
                extrapolation_rolling_memory->measurements[idx_active] = measurements[i];
                idx_active += 1;
            }
        }
    }

    if (extrapolation_rolling_memory->memory_length < MAX_LENGTH_ROLLING_MEMORY) {
        extrapolation_rolling_memory->memory_length += num_active;
    }

}
