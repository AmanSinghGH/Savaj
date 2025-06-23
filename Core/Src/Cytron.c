/*
 * Author:Parth
 * */
#include "Cytron.h"


/*Function to initalize motor*/
void CytronMotor_Init(CytronMotor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort, uint16_t dirPin)
{
	motor->htim =htim; //Assign Timer
	motor->channel=channel; //Assign channel
	motor->dirPort = dirPort; //Assign Port
	motor->dirPin = dirPin; //Assign Pin

	HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_RESET);
	if (HAL_TIM_PWM_Start(motor->htim, motor->channel) != HAL_OK) {
	    Error_Handler();
	}
}
/* Function to set speed*/
void CytronMotor_Speed(CytronMotor_t *motor, int speed)
{
	if(speed>0)
	{
		HAL_GPIO_WritePin(motor->dirPort,motor->dirPin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(motor->dirPort,motor->dirPin, GPIO_PIN_RESET);
	speed=-speed;
	}
	if(speed>1000) speed=1000;
	__HAL_TIM_SET_COMPARE(motor->htim,motor->channel,speed);
}
