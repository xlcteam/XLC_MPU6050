#ifndef _GYRO_H

#define _GYRO_H

#include <Arduino.h>

/* definitions for function MPU6050_calibrate */
#define VALUE_RANGE (0.01f)
#define COUNTER_RANGE 60
#define DEFAULT_CALIB_MAX_TIME 1000
/* offsets definitions */
#define X_GYRO_OFFSET 17
#define Y_GYRO_OFFSET -60
#define Z_GYRO_OFFSET 25
#define X_ACCEL_OFFSET -2859
#define Y_ACCEL_OFFSET -1297
#define Z_ACCEL_OFFSET 2121

/* ISR for MPU6050 interupt */
void dmpDataReady(void);

/* initial function for connecting with device
   true -> succesful, false -> error */
bool Gyro_init(void);

/* it's not at all calibration, but only waiting for COUNTER_RANGE values by
   values, which difference is lower than VALUE_RANGE. If this won't happen
   in max_time miliseconds, it will return false, else true.
   Please don't move with MPU until this function return something */
bool Gyro_calibrate(uint16_t max_time=DEFAULT_CALIB_MAX_TIME);

/* reading the newst data (not the fastest function, but easy) */
void Gyro_update(void);

/* just getters for euler angles */
float Gyro_get_yaw(void);
float Gyro_get_pitch(void);
float Gyro_get_roll(void);

#endif
