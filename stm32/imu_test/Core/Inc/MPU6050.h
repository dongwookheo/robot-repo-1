#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_



#endif /* INC_MPU6050_H_ */

#include <stdint.h>
#include "i2c.h"

// MPU6050 structure
typedef struct
{
	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;
	double ax;
	double ay;
	double az;

	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
	double gx;
	double gy;
	double gz;

	float temperature;

	double kalman_ang_x;
	double kalman_ang_y;
	double ang_z;
} MPU6050_t;

// Kalman structure
typedef struct
{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_Get_Angle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
