/*
 * motor_encoder.c
 *
 *  Created on: Jun 8, 2023
 *      Author: syh
 */
#include "motor_encoder.h"
#include <math.h>

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
{
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;

	if(!first_time)
	{
		encoder_value -> velocity = 0;
		first_time = 1;
	}

	else
	{
		if(temp_counter == encoder_value ->last_counter_value)
		{
			encoder_value -> velocity = 0;
		}
		else if(temp_counter > encoder_value ->last_counter_value)
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value ->velocity = (-encoder_value ->last_counter_value -
						(__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter));
			}
			else
			{
				encoder_value ->velocity = temp_counter - encoder_value ->last_counter_value;
			}
		}
		else
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value ->velocity = temp_counter - encoder_value -> last_counter_value;
			}
			else
			{
				encoder_value ->velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) -
						encoder_value ->last_counter_value);
			}
		}
	}
	// velocity [mm/s], D=80mm, 1320CPR, timer_period = 5ms
	encoder_value -> velocity = encoder_value -> velocity * M_PI * 80.0 / 5.0 * 1000.0 / 1320.0;

	encoder_value -> position += encoder_value -> velocity * 5 / 1000; // 5ms
	encoder_value -> last_counter_value = temp_counter;
}
void reset_encoder(encoder_instance *encoder_value)
{
	encoder_value -> velocity = 0;
	encoder_value -> position = 0;
	encoder_value -> last_counter_value =0;
}

