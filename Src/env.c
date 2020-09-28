#include "../../Inc/Sim_Con/env.h"

void init_env(env_t *env) {
	/* init constants */
	calibrate_env(env, PRESSURE_REFERENCE, TEMPERATURE_REFERENCE);
	update_env(env, TEMPERATURE_REFERENCE);
}

void calibrate_env(env_t *env, float p_g, float T_g) {
	env->T_g = T_g + T_0; // input is temperature in °C
	env->p_g = p_g; //
}

void update_env(env_t *env, float T) {
	env->T = T + T_0; // input is temperature in °C and property is temperature in °K
	env->C = powf(GAMMA * R_0 * env->T, 0.5);
}

float mach_number(env_t *env, float V_x) {
	float mach_number = fabsf(V_x) / env->C;
	return mach_number;
}

void pressure2altitudeAGL(env_t *env, int n, float p[n], bool p_active[n], float h[n]) {
	for (int i = 0; i < n; i++) {
		if (p_active[i]) {
			/* original implementation */
			h[i] = env->T_g / T_GRAD * (1 - powf(p[i] / env->p_g, R_0 * T_GRAD / GRAVITATION));
			/* adapted implementation which can possibly speed up calculation and should have the same results */
			// h[i] = env->T_g / T_GRAD * (1 - expf(logf(p[i] / env->p_g) * R_0 * env->T_grad / GRAVITATION);
		}
	}
}

void altitudeAGL2pressure(env_t *env, int n, float h[n], bool h_active[n], float p[n]) {
	for (int i = 0; i < n; i++) {
		if (h_active[i]) {
			/* original implementation */
			p[i] = env->p_g * powf((1 - T_GRAD * h[i] / env->T_g), GRAVITATION / (R_0 * T_GRAD));
			/* adapted implementation which can possibly speed up calculation and should have the same results */
			// p[i] = env->p_g * expf(logf(1 - env->T_grad * h[i] / env->T_g) * GRAVITATION / (R_0 * env->T_grad));
		}
	}
}

float altitude_gradient(env_t *env, float p) {
	/* computes the altitude gradient per infitesimal change in pressure (dh/dp) at a specified pressure */
	/* original implementation */
	float h_grad = -R_0 * env->T_g / (GRAVITATION * env->p_g) * powf(p / env->p_g, R_0 * T_GRAD / GRAVITATION - 1);
	/* adapted implementation which can possibly speed up calculation and should have the same results */
	// float h_grad = - R_0 * env->T_g / (GRAVITATION * env->p_g) * expf(logf(p / env->p_g) * (R_0 * env->T_grad / GRAVITATION - 1));
	return h_grad;
}
