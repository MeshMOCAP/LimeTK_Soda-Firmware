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

#include "kalman_Filter.h"

void kalman_init(kalman_Filter_t *filterD)
{
    /* We will set the variables like so, these can also be tuned by the user */
    filterD->Q_angle = 0.001f;
    filterD->Q_bias = 0.003f;
    filterD->R_measure = 0.03f;

    filterD->angle = 0.0f; // Reset the angle
    filterD->bias = 0.0f; // Reset bias

    filterD->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    filterD->P[0][1] = 0.0f;
    filterD->P[1][0] = 0.0f;
    filterD->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_getAngle(kalman_Filter_t *filterD, float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    filterD->rate = newRate - filterD->bias;
    filterD->angle += dt * filterD->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    filterD->P[0][0] += dt * (dt*filterD->P[1][1] - filterD->P[0][1] - filterD->P[1][0] + filterD->Q_angle);
    filterD->P[0][1] -= dt * filterD->P[1][1];
    filterD->P[1][0] -= dt * filterD->P[1][1];
    filterD->P[1][1] += filterD->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = filterD->P[0][0] + filterD->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = filterD->P[0][0] / S;
    K[1] = filterD->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - filterD->angle; // Angle difference
    /* Step 6 */
    filterD->angle += K[0] * y;
    filterD->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = filterD->P[0][0];
    float P01_temp = filterD->P[0][1];

    filterD->P[0][0] -= K[0] * P00_temp;
    filterD->P[0][1] -= K[0] * P01_temp;
    filterD->P[1][0] -= K[1] * P00_temp;
    filterD->P[1][1] -= K[1] * P01_temp;

    return filterD->angle;
};

void kalman_setAngle(kalman_Filter_t *filterD, float angle) { filterD->angle = angle; }; // Used to set angle, this should be set as the starting angle
float kalman_getRate(kalman_Filter_t *filterD) { return filterD->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void kalman_setQangle(kalman_Filter_t *filterD, float Q_angle) { filterD->Q_angle = Q_angle; };
void kalman_setQbias(kalman_Filter_t *filterD, float Q_bias) { filterD->Q_bias = Q_bias; };
void kalman_setRmeasure(kalman_Filter_t *filterD, float R_measure) { filterD->R_measure = R_measure; };

float kalman_getQangle(kalman_Filter_t *filterD) { return filterD->Q_angle; };
float kalman_getQbias(kalman_Filter_t *filterD) { return filterD->Q_bias; };
float kalman_getRmeasure(kalman_Filter_t *filterD) { return filterD->R_measure; };
