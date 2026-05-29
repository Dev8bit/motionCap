//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "Adafruit_AHRS_Madgwick.h"
#include <math.h>
//MadgwickAHRS右手系下的四元数为：RQ=(w, x, y, z)

//那么RQ在左手坐标系下的表示为：
//１.当X轴朝向取反
//LQ=(w, x, -y, -z)
//2.当Y轴朝向取反
//LQ=(w, -x, y, -z)
//3.当Z轴朝向取反
//LQ=(w, -x, -y, z)
//---------------------------------------------------------------------------------------------------
// Definitions
#define sampleFreq		100.0f		// sample frequency in Hz
#define betaDef			0.1f		// 2 * proportional gain

#define sampleFreqAHRS	100.0f		// sample frequency in Hz
#define sampleFreqIMU	100.0f		// sample frequency in Hz

#define betaDefAHRS		0.1f		// 2 * proportional gain
#define betaDefIMU		0.1f		// 2 * proportional gain


//---------------------------------------------------------------------------------------------------
// Variable definitions

//volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

#define Kp 10.0f
#define Ki 0.0f
static float ix = 0.0f, iy = 0.0f, iz = 0.0f;  //integral feedback terms
void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // short name local variable for readability
    float norm;
    float hx, hy, hz;  //observed West vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Compute feedback only if magnetometer measurement valid (avoids NaN in magnetometer normalisation)
    float tmp = mx * mx + my * my + mz * mz;
    // Normalise magnetometer
    norm = invSqrt(tmp);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0f)
    {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        norm = invSqrt(tmp);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Measured horizon vector = a x m (in body frame)
        hx = ay * mz - az * my;
        hy = az * mx - ax * mz;
        hz = ax * my - ay * mx;

        // Normalise horizon vector
        norm = invSqrt(hx * hx + hy * hy + hz * hz);
        hx *= norm;
        hy *= norm;
        hz *= norm;

        // Estimated direction of Up reference vector
        ux = 2.0f * (q1q3 - q0q2);
        uy = 2.0f * (q0q1 + q2q3);
        uz = q0q0 - q1q1 - q2q2 + q3q3;

        // estimated direction of horizon (West) reference vector
        wx = 2.0f * (q1q2 + q0q3);
        wy = q0q0 - q1q1 + q2q2 - q3q3;
        wz = 2.0f * (q2q3 - q0q1);

        // Error is cross product between estimated direction and measured direction of the reference vectors
        ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
        ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
        ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
            ix += Ki * ex * deltat;  // integral error scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix;  // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    deltat *= 0.5f;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
	  _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
	  q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
	// magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in
	// accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	// Normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Normalise magnetometer measurement
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Reference direction of Earth's magnetic field
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
		 _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
		 my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
		   _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
		 _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
		 _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
		 (-_2bx * q3 + _2bz * q1) *
			 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
		 _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
		 _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
		 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
		 _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
		 (_2bx * q2 + _2bz * q0) *
			 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
		 (_2bx * q3 - _4bz * q1) *
			 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
		 _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
		 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
		 (-_4bx * q2 - _2bz * q0) *
			 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
		 (_2bx * q1 + _2bz * q3) *
			 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
		 (_2bx * q0 - _4bz * q2) *
			 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
		 _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
		 (-_4bx * q3 + _2bz * q1) *
			 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
		 (-_2bx * q0 + _2bz * q2) *
			 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
		 _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
						s3 * s3); // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 -= betaDefAHRS * s0;
	qDot2 -= betaDefAHRS * s1;
	qDot3 -= betaDefAHRS * s2;
	qDot4 -= betaDefAHRS * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * 1.0f / sampleFreqAHRS;
	q1 += qDot2 * 1.0f / sampleFreqAHRS;
	q2 += qDot3 * 1.0f / sampleFreqAHRS;
	q3 += qDot4 * 1.0f / sampleFreqAHRS;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaDefIMU * s0;
    qDot2 -= betaDefIMU * s1;
    qDot3 -= betaDefIMU * s2;
    qDot4 -= betaDefIMU * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * 1.0f / sampleFreqIMU;
  q1 += qDot2 * 1.0f / sampleFreqIMU;
  q2 += qDot3 * 1.0f / sampleFreqIMU;
  q3 += qDot4 * 1.0f / sampleFreqIMU;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
