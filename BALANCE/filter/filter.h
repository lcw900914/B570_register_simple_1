#ifndef __FILTER_H
#define __FILTER_H

/* Kalman filter for the X-axis (roll/pitch from accelerometer + gyroscope) */
float Kalman_Filter_x(float Accel, float Gyro);

/* First-order complementary filter for the X-axis */
float First_order_filter_x(float angle_m, float gyro_m);

/* Kalman filter for the Y-axis */
float Kalman_Filter_y(float Accel, float Gyro);

/* First-order complementary filter for the Y-axis */
float First_order_filter_y(float angle_m, float gyro_m);

#endif
