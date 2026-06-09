#include "filter.h"

/* Control loop period: 5ms (200 Hz) */
float dt = 0.005;

/**************************************************************************
Function: Kalman filter for the X-axis angle
          Fuses accelerometer angle and gyroscope angular rate to produce
          a smooth, drift-corrected angle estimate.
Input   : Accel - angle derived from accelerometer (degrees)
          Gyro  - angular rate from gyroscope (degrees/s)
Output  : filtered angle estimate (degrees)
**************************************************************************/
float Kalman_Filter_x(float Accel, float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle = 0.001;  // Process noise covariance for angle
	float Q_gyro  = 0.003;  // Process noise covariance for gyro bias
	float R_angle = 0.5;    // Measurement noise covariance (sensor noise)
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4]    = {0, 0, 0, 0};
	static float PP[2][2]   = {{1, 0}, {0, 1}};

	/* Prediction step: integrate gyro rate to predict angle */
	angle += (Gyro - Q_bias) * dt;

	/* Update error covariance matrix (prediction) */
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	/* Innovation: difference between accelerometer angle and predicted angle */
	Angle_err = Accel - angle;

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E     = R_angle + C_0 * PCt_0;

	/* Kalman gain */
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	/* Update covariance matrix */
	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	/* Correction step */
	angle    += K_0 * Angle_err;   // Correct angle estimate
	Q_bias   += K_1 * Angle_err;   // Correct gyro bias estimate
	angle_dot = Gyro - Q_bias;     // Best estimate of true angular rate

	return angle;
}

/**************************************************************************
Function: First-order complementary filter for the X-axis
          Blends accelerometer angle (K1) with integrated gyro angle (1-K1)
Input   : angle_m - angle from accelerometer (degrees)
          gyro_m  - angular rate from gyroscope (degrees/s)
Output  : filtered angle estimate (degrees)
**************************************************************************/
float First_order_filter_x(float angle_m, float gyro_m)
{
	static float angle;
	float K1 = 0.02;  // Trust factor for accelerometer (higher = more accel)
	angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
	return angle;
}

/**************************************************************************
Function: Kalman filter for the Y-axis angle (identical algorithm to X-axis)
Input   : Accel - angle derived from accelerometer (degrees)
          Gyro  - angular rate from gyroscope (degrees/s)
Output  : filtered angle estimate (degrees)
**************************************************************************/
float Kalman_Filter_y(float Accel, float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle = 0.001;
	float Q_gyro  = 0.003;
	float R_angle = 0.5;
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4]    = {0, 0, 0, 0};
	static float PP[2][2]   = {{1, 0}, {0, 1}};

	angle += (Gyro - Q_bias) * dt;

	Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle;

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E     = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle    += K_0 * Angle_err;
	Q_bias   += K_1 * Angle_err;
	angle_dot = Gyro - Q_bias;

	return angle;
}

/**************************************************************************
Function: First-order complementary filter for the Y-axis
Input   : angle_m - angle from accelerometer (degrees)
          gyro_m  - angular rate from gyroscope (degrees/s)
Output  : filtered angle estimate (degrees)
**************************************************************************/
float First_order_filter_y(float angle_m, float gyro_m)
{
	static float angle;
	float K1 = 0.02;
	angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
	return angle;
}
