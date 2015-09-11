#include <Arduino.h>
#include "Gyro.h"

/* include libraries needed to communicate with MPU6050 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* object for getting data */
MPU6050 mpu;

/* MPU control/status vars */
bool dmpReady = false;  // true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // status of device (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/* orientation/motion vars */
struct Quaternion q;               // [w, x, y, z]         quaternion container
struct VectorFloat gravity;        // [x, y, z]            gravity vector
float ypr[3] = { 0, 0, 0 }; // [yaw, pitch, roll]   
float last_ypr[3] = { 0, 0, 0 };

/* offsets after MPU6050_calibrate */
float ypr_offsets[3] = { 0, 0, 0 };

/* indicates interrupt */
volatile bool mpu_interrupt = false;

void dmp_data_ready(void)
{
    mpu_interrupt = true;
}

bool Gyro_init(void)
{  
    Wire.begin();
    mpu.initialize();

    if (mpu.testConnection() == false) {
      return false;
    }
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {
        mpu.setXGyroOffset(X_GYRO_OFFSET);
        mpu.setYGyroOffset(Y_GYRO_OFFSET);
        mpu.setZGyroOffset(Z_GYRO_OFFSET);
        mpu.setXAccelOffset(X_ACCEL_OFFSET);
        mpu.setYAccelOffset(Y_ACCEL_OFFSET);
        mpu.setZAccelOffset(Z_ACCEL_OFFSET);
    
        mpu.setDMPEnabled(true);
        dmpReady = true;

        attachInterrupt(0, dmp_data_ready, RISING);

        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        return true;
    }
    else {
        return false;
    }
}

bool Gyro_calibrate(uint16_t max_time)
{
    uint32_t start_time = millis();
    /* how many value differences were in range VALUE_RANGE  */
    uint16_t counter[3] = { 0, 0, 0 };

    while (true) {
        /* waiting for interrupt */
        while (!mpu_interrupt) {
            /* checking if function doesn't take longer than it should */
            if (millis() - start_time > max_time) {
                return false;
            }
        }

        mpu_interrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        /* owerflowed FIFO buffer (this should happen never) */
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
        }
        /* we can read data */
        else if (mpuIntStatus & 0x02) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            /* loop control variable */
            uint8_t i;
            /* true if all indexes of counter[3] are >= COUNTER_RANGE */
            bool calibrated;

            /* storing old data */
            for (i = 0; i < 3; i++) {
                last_ypr[i] = ypr[i];
            }

            /* getting new data */
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            /* finding out, if we are "calibrated" */
            calibrated = true;
            for (i = 0; i < 3; i++) {
                /* convert radians to degrees */
                ypr[i] = ypr[i] * 180 / M_PI;
                
                if (abs(ypr[i] - last_ypr[i]) < VALUE_RANGE) {
                    counter[i]++;
                    if (counter[i] < COUNTER_RANGE) {
                        calibrated = false;
                    }
                }
                else {
                    counter[i] = 0;
                }
            }
            /* if we are calibrate, set offsets and return true */
            if (calibrated == true) {
                for (i = 0; i < 0; i++) {
                    ypr_offsets[i] = ypr[i] * -1;
                }
                return true;
            }
        }
    }
}
    
void Gyro_update(void)
{
    /* reset FIFO buffer and wait for first data */
    mpu.resetFIFO();
    mpu_interrupt = false;
    do {
        while (!mpu_interrupt) {
            ;
        }
        mpu_interrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
    } while (!(mpuIntStatus & 0x02));
    
    /* reading data */
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    /* converting to degrees and using offsets */
    for (uint8_t i = 0; i < 3; i++) {
        ypr[i] = (ypr[i] * 180 / M_PI) + ypr_offsets[i];
    }
}

float Gyro_get_yaw(void)
{
    return ypr[0];
}

float Gyro_get_pitch(void)
{
    return ypr[1];
}

float Gyro_get_roll(void)
{
    return ypr[2];
}
