#ifndef __BMX055_H__
#define __BMX055_H__


char BMX055_config(void);
char BMX055_CalOrientation(void);
char BMX055_CalibrateMag(char bConfigMag);
void BMX055_updateAHRS(void);
void BMX055_updateIMU(void);

float getPitch(float q0, float q1, float q2, float q3);
float getYaw(float q0, float q1, float q2, float q3); 
float getRoll(float q0, float q1, float q2, float q3);

#endif
