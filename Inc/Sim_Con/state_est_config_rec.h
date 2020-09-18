#define STATE_ESTIMATION_FREQUENCY 100
#define NUM_SENSORBOARDS 2
#define MAX_NUM_SENSORS 2 // max between NUM_IMU and NUM_BAROMETER
#define NUM_IMU 2
#define NUM_BARO 2

#define MAX_LENGTH_ROLLING_MEMORY 18
#define USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION 0 // set to 1 to activate sensor elimination by extrapolation for barometer and temperature
#define EXTRAPOLATION_POLYFIT_DEGREE 2