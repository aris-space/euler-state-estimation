#include "Util/math_utils.h"
#include "state_est_const.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#ifndef EKF_H_
#define EKF_H_

/* Matrix Sizes */
#if STATE_ESTIMATION_TYPE == 1
	#define NUMBER_STATES 3	/* NUMBER_STATES x NUMBER_STATES -> A Matrix */
	#define NUMBER_INPUTS 1	/* NUMBER_STATES x NUMBER_INPUTS -> B Matrix */
	#define NUMBER_PROCESS_NOISE 1	/* NUMBER_STATES x NUMBER_PROCESS_NOISE -> G Matrix */
#elif STATE_ESTIMATION_TYPE == 2
	#define NUMBER_STATES 9	/* NUMBER_STATES x NUMBER_STATES -> A Matrix */
	#define NUMBER_INPUTS 6	/* NUMBER_STATES x NUMBER_INPUTS -> B Matrix */
	#define NUMBER_PROCESS_NOISE 6	/* NUMBER_STATES x NUMBER_PROCESS_NOISE -> G Matrix */
#endif

#define NUMBER_MEASUREMENTS NUM_BARO /* NUMBER_MEASUREMENTS x NUMBER_STATES -> H Matrix */
#define LAMBDA 0.0001		/* Lambda for Moore Penrose Pseudoinverse */

typedef struct kf_state_t{
	/* Fixed Variables */
    float Ad[NUMBER_STATES][NUMBER_STATES];
	float Ad_T[NUMBER_STATES][NUMBER_STATES];
	float Bd[NUMBER_STATES][NUMBER_INPUTS];
	float Gd[NUMBER_STATES][NUMBER_PROCESS_NOISE];
	float Gd_T[NUMBER_PROCESS_NOISE][NUMBER_STATES];
	float H[NUMBER_MEASUREMENTS][NUMBER_STATES];
	float H_T[NUMBER_STATES][NUMBER_MEASUREMENTS];
	float Q[NUMBER_PROCESS_NOISE][NUMBER_PROCESS_NOISE];
	float R[NUMBER_MEASUREMENTS][NUMBER_MEASUREMENTS];
	float R_inv[NUMBER_MEASUREMENTS][NUMBER_MEASUREMENTS];

	/* State Variables */
    float u[NUMBER_INPUTS]; // inputs
	float x_est[NUMBER_STATES]; // estimated state (posteriori)
	float P_est[NUMBER_STATES][NUMBER_STATES]; // estimated covariance (posteriori)
	float x_priori[NUMBER_STATES]; // priori state (priori)
	float P_priori[NUMBER_STATES][NUMBER_STATES]; // priori covariance (priori)
	float P_priori_inv[NUMBER_STATES][NUMBER_STATES]; // inverted priori covariance
    float z[NUMBER_MEASUREMENTS]; // measurements
	float y[NUMBER_MEASUREMENTS]; // state innovation
	float S[NUMBER_MEASUREMENTS][NUMBER_MEASUREMENTS]; // covariance innovation
	float S_inv[NUMBER_MEASUREMENTS][NUMBER_MEASUREMENTS];
	float K[NUMBER_STATES][NUMBER_MEASUREMENTS];

	/* specifies which measurements should be included in the Kalman update */
	bool z_active[NUMBER_MEASUREMENTS];
	int num_z_active;

    /* Placeholder Variables for increased speed */
	float Placeholder_Ad_mult_P_est[NUMBER_STATES][NUMBER_STATES];
    float Placeholder_Gd_mult_Q[NUMBER_STATES][NUMBER_PROCESS_NOISE];
	float Placeholder_H_mult_P_priori[NUMBER_MEASUREMENTS][NUMBER_MEASUREMENTS];
	float Placeholder_P_priori_mult_H_T[NUMBER_STATES][NUMBER_MEASUREMENTS];
	float Placeholder_P_est[NUMBER_STATES][NUMBER_STATES];
    float Placeholder_eye[NUMBER_STATES][NUMBER_STATES];
    float Placeholder_K_mult_H[NUMBER_STATES][NUMBER_STATES];
	float Placeholder_S_inv_1[NUMBER_STATES][NUMBER_STATES];
	float Placeholder_S_inv_2[NUMBER_STATES][NUMBER_STATES];
	float Placeholder_S_inv_3[NUMBER_MEASUREMENTS][NUMBER_STATES];
	float Placeholder_R_inv_mult_H[NUMBER_MEASUREMENTS][NUMBER_STATES];
	float Placeholder_H_T_mult_R_inv[NUMBER_STATES][NUMBER_MEASUREMENTS];
} kf_state_t;

/* Here, the Matrices are declared */
void reset_kf_state(kf_state_t *kf_state);
void kf_prediction(kf_state_t *kf_state);
void select_kf_observation_matrices(kf_state_t *kf_state);
void kf_update(kf_state_t *kf_state);

#endif
