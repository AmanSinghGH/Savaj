/*
 * servo_driver.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PARTH
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include "stm32f7xx_hal.h"

// Define servo limits
#define SERVO_MIN_CCR 250   // Corresponds to 0°
#define SERVO_MAX_CCR 1250  // Corresponds to 180°
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Structure to hold servo details
typedef struct {
    TIM_HandleTypeDef* htim;  // Timer handler
    uint32_t Channel;         // PWM Channel
} Servo_t;

// Function Prototypes
void Servo_Init(Servo_t* servo, TIM_HandleTypeDef* htim, uint32_t Channel);
void Servo_SetAngle(Servo_t* servo, uint8_t angle);


#endif /* INC_SERVO_DRIVER_H_ */
