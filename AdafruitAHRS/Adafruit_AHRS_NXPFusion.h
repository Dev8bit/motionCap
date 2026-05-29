/*!
 * @file Adafruit_AHRS_NXPFusion.h
 *
 * @section license License
 *
 * This is a modification of
 * https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/blob/master/Sources/tasks.h
 * by PJRC / Paul Stoffregen https://github.com/PaulStoffregen/NXPMotionSense
 *
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __Adafruit_Nxp_Fusion_h_
#define __Adafruit_Nxp_Fusion_h_


/*!
* @brief Kalman/NXP Fusion algorithm.
*/
/**************************************************************************/
/*!
* @brief Initializes the 9DOF Kalman filter.
*
* @param sampleFrequency The sensor sample rate in herz(samples per second).
*/
/**************************************************************************/
void Adafruit_NXPSensorFusion_Init(void);

/**************************************************************************/
/*!
* @brief Updates the filter with new gyroscope, accelerometer, and
* magnetometer data. For roll, pitch, and yaw the accelerometer values can be
* either m/s^2 or g, but for linear acceleration they have to be in g.
*
* 9DOF orientation function implemented using a 12 element Kalman filter
*
* void fRun_9DOF_GBY_KALMAN(SV_9DOF_GBY_KALMAN_t *SV,
* const AccelSensor_t *Accel, const MagSensor_t *Mag,
* const GyroSensor_t *Gyro, const MagCalibration_t *MagCal)
*
* @param gx The gyroscope x axis. In DPS.
* @param gy The gyroscope y axis. In DPS.
* @param gz The gyroscope z axis. In DPS.
* @param ax The accelerometer x axis. In g.
* @param ay The accelerometer y axis. In g.
* @param az The accelerometer z axis. In g.
* @param mx The magnetometer x axis. In uT.
* @param my The magnetometer y axis. In uT.
* @param mz The magnetometer z axis. In uT.
*/
/**************************************************************************/
void Adafruit_NXPSensorFusion_update(float gx, float gy, float gz, float ax, float ay, float az,
		  float mx, float my, float mz);

float Adafruit_NXPSensorFusion_getRoll(void);
float Adafruit_NXPSensorFusion_getPitch(void);
float Adafruit_NXPSensorFusion_getYaw(void);
void Adafruit_NXPSensorFusion_getQuat(float* fq);

#endif
