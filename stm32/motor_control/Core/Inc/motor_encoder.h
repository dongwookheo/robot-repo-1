/*
 * motor_encoder.h
 *
 *  Created on: Jun 8, 2023
 *      Author: syh
 */

#ifndef INC_MOTOR_ENCODER_H_
#define INC_MOTOR_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "main.h"

typedef struct {
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
}encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);

#endif /* INC_MOTOR_ENCODER_H_ */
