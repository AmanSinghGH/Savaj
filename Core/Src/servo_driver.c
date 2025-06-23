/*
 * servo_driver.c
 *
 *  Created on: Feb 2, 2025
 *      Author: PARTH
 */

#include "servo_driver.h"

// Initialize Servo with Timer and Channel
void Servo_Init(Servo_t* servo, TIM_HandleTypeDef* htim, uint32_t Channel) {
    servo->htim = htim;
    servo->Channel = Channel;
    HAL_TIM_PWM_Start(htim, Channel);  // Start PWM
}

// Set Servo Angle (0° - 180°)
void Servo_SetAngle(Servo_t* servo, uint8_t angle) {
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Convert angle to CCR using linear mapping
    uint16_t ccr_value = SERVO_MIN_CCR + ((angle * (SERVO_MAX_CCR - SERVO_MIN_CCR)) / SERVO_MAX_ANGLE);

    // Set PWM Compare Register
    __HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, ccr_value);
}

