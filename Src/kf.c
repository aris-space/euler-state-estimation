#include "../Inc/kf.h"

void reset_kf_state(kf_state_t *kf_state){
    #if STATE_ESTIMATION_TYPE == 1
        if (STATE_ESTIMATION_FREQUENCY == 1000) {
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 1.0E-3, 5.0E-7}, {0, 1.0E-0, 1.0E-3}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{5.0E-7}, {1.0E-3}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{5.0E-7}, {1.0E-3}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 200) {
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 5.0E-3, 1.25E-5}, {0, 1.0E-0, 5.0E-3}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{1.25E-5}, {5.0E-3}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{1.25E-5}, {5.0E-3}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 100) { 
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 1.0E-2, 5.0E-5}, {0, 1.0E-0, 1.0E-2}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{5.0E-5}, {1.0E-2}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{5.0E-5}, {1.0E-2}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 50) { 
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 2.0E-2, 2.0E-4}, {0, 1.0E-0, 2.0E-2}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{2.0E-4}, {2.0E-2}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{2.0E-4}, {2.0E-2}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 40) { 
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 2.5E-2, 3.125E-4}, {0, 1.0E-0, 2.5E-2}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{3.125E-4}, {2.5E-2}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{3.125E-4}, {2.5E-2}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 33.33) { 
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 3.003E-2, 4.5009E-4}, {0, 1.0E-0, 3.003E-2}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{4.5009E-4}, {3.003E-2}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{4.5009E-4}, {3.003E-2}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else if (STATE_ESTIMATION_FREQUENCY == 1) { 
            float A_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-0, 1.0E-0, 5.0E-1}, {0, 1.0E-0, 1.0E-0}, {0.0, 0.0, 1.0E-0}};
            float B_init[NUMBER_STATES][NUMBER_INPUTS] = {{5.0E-1}, {1.0E-0}, {0.0}};
            float G_init[NUMBER_STATES][NUMBER_PROCESS_NOISE] = {{5.0E-1}, {1.0E-0}, {0.0}};
            memcpy(kf_state->Ad, A_init, sizeof(kf_state->Ad));
            memcpy(kf_state->Bd, B_init, sizeof(kf_state->Bd));
            memcpy(kf_state->Gd, G_init, sizeof(kf_state->Gd));
        } else {
            float A[NUMBER_STATES][NUMBER_STATES] = {{0, 1, 0}, {0, 0, 1}, {0, 0, 0}};
            float B[NUMBER_STATES][NUMBER_INPUTS] = {{0}, {1}, {0}};
            float G[NUMBER_STATES][NUMBER_INPUTS] = {{0}, {1}, {0}};

            discretize(STATE_ESTIMATION_FREQUENCY, NUMBER_STATES, NUMBER_INPUTS, A, B, kf_state->Ad, kf_state->Bd);
            discretize(STATE_ESTIMATION_FREQUENCY, NUMBER_STATES, NUMBER_INPUTS, A, G, kf_state->Ad, kf_state->Gd);
        }
    #elif STATE_ESTIMATION_TYPE == 2
        float A[NUMBER_STATES][NUMBER_STATES] = {0};
        float B[NUMBER_STATES][NUMBER_INPUTS] = {0};
        float G[NUMBER_STATES][NUMBER_INPUTS] = {0};

        for(int i = 0; i < 3; i++) {
            A[i][2+i] = 1;
        }
        for(int i = 0; i < 6; i++) {
            B[2+i][i] = 1;
        }

        discretize(STATE_ESTIMATION_FREQUENCY, NUMBER_STATES, NUMBER_INPUTS, A, B, kf_state->Ad, kf_state->Bd);
        memcpy(&kf_state->Gd, &kf_state->Bd, sizeof(kf_state->Bd));
    #endif

	float x_est_init[NUMBER_STATES] = {0};
	float P_est_init[NUMBER_STATES][NUMBER_STATES] = {{1.0E-9, 0, 0}, {0, 1.0E-12, 0}, {0, 0, 0}};

    memcpy(kf_state->x_est, x_est_init, sizeof(x_est_init));
    memcpy(kf_state->P_est, P_est_init, sizeof(P_est_init));

    memset(kf_state->Q, 0, NUMBER_PROCESS_NOISE*NUMBER_PROCESS_NOISE*sizeof(kf_state->Q[0][0]));
    memset(kf_state->R, 0, NUMBER_MEASUREMENTS*NUMBER_MEASUREMENTS*sizeof(kf_state->R[0][0]));

    memset(kf_state->z, 0, NUMBER_MEASUREMENTS*sizeof(kf_state->z[0]));
    memset(kf_state->z_active, false, NUMBER_MEASUREMENTS*sizeof(kf_state->z_active[0]));
    kf_state->num_z_active = 0;

    transpose(NUMBER_STATES, NUMBER_STATES, kf_state->Ad, kf_state->Ad_T);
    transpose(NUMBER_STATES, NUMBER_PROCESS_NOISE, kf_state->Gd, kf_state->Gd_T);
}

void kf_prediction(kf_state_t *kf_state){
    /* Prediction Step */
    /* Calculation of x_priori */
    matvecprod(NUMBER_STATES, NUMBER_STATES, kf_state->Ad, kf_state->x_est, kf_state->x_priori, true);
    matvecprod(NUMBER_STATES, NUMBER_INPUTS, kf_state->Bd, kf_state->u, kf_state->x_priori, false);

    /* Calculation of P_priori */
    /* P_priori = Ad * P_est_prior * Ad_T + Gd * Q * Gd_T */
    matmul(NUMBER_STATES, NUMBER_STATES, NUMBER_STATES, kf_state->Ad, kf_state->P_est, kf_state->Placeholder_Ad_mult_P_est, true);
    matmul(NUMBER_STATES, NUMBER_PROCESS_NOISE, NUMBER_PROCESS_NOISE, kf_state->Gd, kf_state->Q, kf_state->Placeholder_Gd_mult_Q, true);

    matmul(NUMBER_STATES, NUMBER_STATES, NUMBER_STATES, kf_state->Placeholder_Ad_mult_P_est, kf_state->Ad_T, kf_state->P_priori, true);
    matmul(NUMBER_STATES, NUMBER_PROCESS_NOISE, NUMBER_STATES, kf_state->Placeholder_Gd_mult_Q, kf_state->Gd_T, kf_state->P_priori, false);
}

void select_kf_observation_matrices(kf_state_t *kf_state){
    memset(kf_state->H, 0, NUMBER_MEASUREMENTS*NUMBER_STATES*sizeof(kf_state->H[0][0]));

    for(int i = 0; i < NUMBER_MEASUREMENTS; i++){
        if (kf_state->z_active[i]) {
            /* activate contribution of measurement in measurement matrix */
            #if STATE_ESTIMATION_TYPE == 1
                kf_state->H[i][0] = 1;
            #elif STATE_ESTIMATION_TYPE == 2
                kf_state->H[i][2] = 1;
            #endif
        } else {
            /* set contributed measurement covariance to zero */
            kf_state->R[i][i] = 0;
        }
    }

    transpose(NUMBER_MEASUREMENTS, NUMBER_STATES, kf_state->H, kf_state->H_T);
}

void kf_update(kf_state_t *kf_state) {
    /* Update Step */
    /* y = z - H * x_priori */
    matvecprod(NUMBER_MEASUREMENTS, NUMBER_STATES, kf_state->H, kf_state->x_priori, kf_state->y, true);
    vecsub(NUMBER_MEASUREMENTS, kf_state->z, kf_state->y, kf_state->y);

    /* S = H * P_priori * H_T + R */
    matmul(NUMBER_MEASUREMENTS, NUMBER_STATES, NUMBER_STATES, kf_state->H, kf_state->P_priori, kf_state->Placeholder_H_mult_P_priori, true);
    matmul(NUMBER_MEASUREMENTS, NUMBER_STATES, NUMBER_MEASUREMENTS, kf_state->Placeholder_H_mult_P_priori, kf_state->H_T, kf_state->S, true);
    matadd(NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, kf_state->S, kf_state->R, kf_state->S);

    /* Calculate Pseudoinverse of covariance innovation */
    memset(kf_state->S_inv, 0, NUMBER_MEASUREMENTS*NUMBER_MEASUREMENTS*sizeof(kf_state->S_inv[0][0]));
    /* the cholvesky inverse has a big O complexity of n^3 */
    if (powf(NUMBER_MEASUREMENTS, 3) > (4 * powf(NUMBER_STATES, 3))) {
        /* we use the Woodbury matrix identity in case the number of measurements is bigger than the state dimension */
        /* S_inv = R_inv - R_inv * H * (P_priori_inv + H_T * R_inv * H)^(-1) * H_T * R_inv */
        diag_inverse(NUMBER_MEASUREMENTS, kf_state->R, kf_state->R_inv, LAMBDA);
        cholesky_inverse(NUMBER_STATES, kf_state->P_priori, kf_state->P_priori_inv, LAMBDA);

        matmul(NUMBER_STATES, NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, kf_state->H_T, kf_state->R_inv, kf_state->Placeholder_H_T_mult_R_inv, true);
        matmul(NUMBER_STATES, NUMBER_MEASUREMENTS, NUMBER_STATES, kf_state->Placeholder_H_T_mult_R_inv, kf_state->H, kf_state->Placeholder_S_inv_1, true);
        matadd(NUMBER_STATES, NUMBER_STATES, kf_state->P_priori_inv, kf_state->Placeholder_S_inv_1, kf_state->Placeholder_S_inv_1);
        cholesky_inverse(NUMBER_STATES, kf_state->Placeholder_S_inv_1, kf_state->Placeholder_S_inv_2, LAMBDA);

        matmul(NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, NUMBER_STATES, kf_state->R_inv, kf_state->H, kf_state->Placeholder_R_inv_mult_H, true);
        matmul(NUMBER_MEASUREMENTS, NUMBER_STATES, NUMBER_STATES, kf_state->Placeholder_R_inv_mult_H, kf_state->Placeholder_S_inv_2, kf_state->Placeholder_S_inv_3, true);
        matmul(NUMBER_MEASUREMENTS, NUMBER_STATES, NUMBER_MEASUREMENTS, kf_state->Placeholder_S_inv_3, kf_state->Placeholder_H_T_mult_R_inv, kf_state->S_inv, true);
        matsub(NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, kf_state->R_inv, kf_state->S_inv, kf_state->S_inv);
    } else {
        /* we use the regular inverse when the state dimension is larger than the number of measurements */
        /* the cholvesky inverse has a big O complexity of n^3 */
        cholesky_inverse(NUMBER_MEASUREMENTS, kf_state->S, kf_state->S_inv, LAMBDA);
    }

    /* K  = P_priori * H_T * S_inv */
    matmul(NUMBER_STATES, NUMBER_STATES, NUMBER_MEASUREMENTS, kf_state->P_priori, kf_state->H_T, kf_state->Placeholder_P_priori_mult_H_T, true);
    matmul(NUMBER_STATES, NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, kf_state->Placeholder_P_priori_mult_H_T, kf_state->S_inv, kf_state->K, true);

    /* x_est = x_priori + K*y */
    matvecprod(NUMBER_STATES, NUMBER_MEASUREMENTS, kf_state->K, kf_state->y, kf_state->x_est, true);
    vecadd(NUMBER_STATES, kf_state->x_priori, kf_state->x_est, kf_state->x_est);


    /* P_est = (eye(NUMBER_STATES) - K*H)*P_priori */
    eye(NUMBER_STATES, kf_state->Placeholder_eye);
    matmul(NUMBER_STATES, NUMBER_MEASUREMENTS, NUMBER_STATES, kf_state->K, kf_state->H, kf_state->Placeholder_K_mult_H, true);
    matsub(NUMBER_STATES, NUMBER_STATES, kf_state->Placeholder_eye, kf_state->Placeholder_K_mult_H, kf_state->Placeholder_P_est);
    matmul(NUMBER_STATES, NUMBER_STATES,  NUMBER_STATES, kf_state->Placeholder_P_est, kf_state->P_priori, kf_state->P_est, true);
}