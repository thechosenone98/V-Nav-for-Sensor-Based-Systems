/*
 * File: HCSR04_cfg.c
 * Driver Name: [[ HC-SR04 Ultrasonic Sensor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "../HCSR04/HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOB,
		GPIO_PIN_0,
		TIM2,
		TIM_CHANNEL_1,
		90
	},
	// HC-SR04 Sensor Unit 2 Configurations
	{
		GPIOB,
		GPIO_PIN_1,
		TIM2,
		TIM_CHANNEL_2,
		90
	},
	// HC-SR04 Sensor Unit 2 Configurations
	{
		GPIOC,
		GPIO_PIN_4,
		TIM2,
		TIM_CHANNEL_3,
		90
	},
	// HC-SR04 Sensor Unit 2 Configurations
	{
		GPIOC,
		GPIO_PIN_5,
		TIM2,
		TIM_CHANNEL_4,
		90
	}
};
