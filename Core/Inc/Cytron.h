/*
 * Author: Parth
 */
#ifndef CYTRON_MOTOR_H
#define CYTRON_MOTOR_H

#include "main.h"
typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;

}CytronMotor_t;

/* Function Prototype */
void CytronMotor_Init(CytronMotor_t *motor,TIM_HandleTypeDef *htim, uint32_t channel,GPIO_TypeDef *dirPort, uint16_t dirPin);
void CytronMotor_Speed(CytronMotor_t *motor, int speed);

#endif
