# aris-euler-state-estimation
## Usage
The following steps need to be taken to use the state estimation:

1. Initialisation and calibration for ground temperature and pressure:
```
state_est_state_t state_est_state = { 0 };
reset_state_est_state(ground_pressure, ground_temperature, &state_est_state);
```
2. Update of state estimation measurements
```
state_est_state->state_est_meas = xy;
```
3. Update of state estimation measurements (with included flight detection step)
```
state_est_step(tick_count, &state_est_state, true);
```