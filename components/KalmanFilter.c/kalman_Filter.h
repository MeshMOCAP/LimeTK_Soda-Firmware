/*  Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
    Copyright © 2024 SourLemonJuice.
    
    SPDX-license-identifier: GPL-3.0-or-later
    本文件是 kalman_Filter.c 的一部分。
    kalman_Filter.c 是自由软件：你可以再分发之和/或依照由自由软件基金会发布的 GNU 通用公共许可证修改之，无论是版本 3 许可证，还是（按你的决定）任何以后版都可以。
    发布 kalman_Filter.c 是希望它能有用，但是并无保障;甚至连可销售和符合某个特定的目的都不保证。请参看 GNU 通用公共许可证，了解详情。
    你应该随程序获得一份 GNU 通用公共许可证的复本。如果没有，请看 <https://www.gnu.org/licenses/>。

    Original author contact information
    -------------------
    Kristian Lauszus, TKJ Electronics
    Web      :  http://www.tkjelectronics.com
    e-mail   :  kristianl@tkjelectronics.com
*/

// define data struct
typedef struct
{
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalman_Filter_t;

// function prototypes
// init
void kalman_init(kalman_Filter_t *filterD);

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_getAngle(kalman_Filter_t *filterD, float newAngle, float newRate, float dt);

void kalman_setAngle(kalman_Filter_t *filterD, float angle); // Used to set angle, this should be set as the starting angle
float kalman_getRate(kalman_Filter_t *filterD); // Return the unbiased rate

/* These are used to tune the Kalman filter */
void kalman_setQangle(kalman_Filter_t *filterD, float Q_angle);
/**
 * setQbias(float Q_bias)
 * Default value (0.003f) is in Kalman.cpp. 
 * Raise this to follow input more closely,
 * lower this to smooth result of kalman filter.
 */
void kalman_setQbias(kalman_Filter_t *filterD, float Q_bias);
void kalman_setRmeasure(kalman_Filter_t *filterD, float R_measure);

float kalman_getQangle(kalman_Filter_t *filterD);
float kalman_getQbias(kalman_Filter_t *filterD);
float kalman_getRmeasure(kalman_Filter_t *filterD);
