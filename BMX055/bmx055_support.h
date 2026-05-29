#ifndef __BMX055_H__
#define __BMX055_H__

#include <stdbool.h>
/**
 * @brief TWIM initialization.
 */
void nrf_twi_init (void);
void nrf_send_eMPLPack(void);
void nrf_timer_SampleBMX055_init(void);
void nrf_SampleBMX055_loopRoutine(void);

char BMX055_init(void);
char BMX055_axisCompensation(void);
char BMX055_calibrateMag(char bConfigMag);
void BMX055_updateFusion(void);

float getPitch(float q0, float q1, float q2, float q3);
float getYaw(float q0, float q1, float q2, float q3); 
float getRoll(float q0, float q1, float q2, float q3);

void BMX055_setMagCalib(float xMagS, float xMagB, float yMagS, float yMagB, float zMagS, float zMagB);
void BMX055_getMagCalib(float* xMagS, float* xMagB, float* yMagS, float* yMagB, float* zMagS, float* zMagB);

float getScreenAxisX(void);
float getScreenAxisY(void);
#endif
