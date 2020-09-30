# aris-euler-state-estimation
## Usage
The following steps need to be taken to use the state estimation:

1. Include headers
Define either `EULER_SIMCON` for Simulations & Control, `EULER_AV` or `EULER_REC` to tune the includes and typedefinitions:
```
#define EULER_SIMCON 1
#include "euler_state_estimation/Inc/state_est_const.h"
#include "euler_state_estimation/Src/state_est.h"
```

2. Initialisation and calibration for ground temperature and pressure:
```
state_est_state_t state_est_state = { 0 };
reset_state_est_state(ground_pressure, ground_temperature, &state_est_state);
```
3. Update of state estimation measurements
```
state_est_state->state_est_meas = xy;
```
4. Update of state estimation measurements (with included flight detection step)
```
state_est_step(tick_count, &state_est_state, true);
```