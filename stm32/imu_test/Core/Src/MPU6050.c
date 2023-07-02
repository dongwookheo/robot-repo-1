/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "MPU6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105


#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_XOUT_H 0x43

// Setup MPU6050
#define DEVADDR 0xD0
const uint16_t i2c_timeout = 100;
const double accel_z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};

Kalman_t KalmanY = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};



uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t check;
	uint8_t data;

	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(I2Cx, DEVADDR, MPU6050_WHO_AM_I, 1, &check, 1, i2c_timeout);
	// 0x68 will be returned by the sensor if everything goes well
	if(check == 104)
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		data = 0;
		HAL_I2C_Mem_Write(I2Cx, DEVADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, i2c_timeout);

		// Set data rate of 1khz by writing SMPLRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(I2Cx, DEVADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, i2c_timeout);

		// Set accelerometer configuration in ACCEL_CONFIG register
		// XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> +- 2g
		data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, DEVADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, i2c_timeout);

		// Set gyroscope configuration in GYRO_CONFIG register
		// XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> +- 250 deg/s
		data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, DEVADDR, MPU6050_GYRO_CONFIG, 1 , &data, 1, i2c_timeout);
		return 0;
	}
	return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];

	// Read 6 byte of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(I2Cx, DEVADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

	DataStruct->accel_x_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->accel_y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->accel_z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	/* convert the raw values into acceleration in 'g'
	 * we have to divide according to the full scale value set in FS_SEL
	 * I have configured FS_SEL = 0. So I an dividing by 16384.0
	 * for more details check ACCEL_CONFIG register.
	 * */
	DataStruct->ax = DataStruct->accel_x_raw / 16384.0;
	DataStruct->ay = DataStruct->accel_y_raw / 16384.0;
	DataStruct->az = DataStruct->accel_z_raw / accel_z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_data[6];

	// Read 6 bytes of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read(I2Cx, DEVADDR, MPU6050_GYRO_XOUT_H, 1, Rec_data, 1, i2c_timeout);

	DataStruct->gyro_x_raw = (int16_t)(Rec_data[0] << 8 | Rec_data[1]);
	DataStruct->gyro_y_raw = (int16_t)(Rec_data[2] << 8 | Rec_data[3]);
	DataStruct->gyro_z_raw = (int16_t)(Rec_data[4] << 8 | Rec_data[5]);

	/* convert the raw values into dps
	 * we have to divide according to the full scale value set in FS_SEL
	 * I have configured FS_SEL = 0. So I am dividing by 131.0
	 * for more details check GYRO_CONFIG register
	 * */

	DataStruct->gx = DataStruct->gyro_x_raw / 131.0;
	DataStruct->gy = DataStruct->gyro_y_raw / 131.0;
	DataStruct->gz = DataStruct->gyro_z_raw / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_data[2];
	int16_t temp;

	// Read 2 bytes of data starting from TEMP_OUT_H register

	HAL_I2C_Mem_Write(I2Cx, DEVADDR, MPU6050_TEMP_OUT_H, 1, Rec_data, 2, i2c_timeout);

	temp = (int16_t)(Rec_data[0] << 8 | Rec_data[1]);
	DataStruct->temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[14];
	int16_t temp;

	// Read 14 bytes of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(I2Cx, DEVADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 14, i2c_timeout);

	DataStruct->accel_x_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->accel_y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->accel_z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
	DataStruct->gyro_x_raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	DataStruct->gyro_y_raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	DataStruct->gyro_z_raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	DataStruct->ax = DataStruct->accel_x_raw / 16384.0;
	DataStruct->ay = DataStruct->accel_y_raw / 16384.0;
	DataStruct->az = DataStruct->accel_z_raw / accel_z_corrector;
	DataStruct->temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	DataStruct->gx = DataStruct->gyro_x_raw / 131.0;
	DataStruct->gy = DataStruct->gyro_y_raw / 131.0;
	DataStruct->gz = DataStruct->gyro_z_raw / 131.0;

	// Kalman angle solve
	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	double roll;
	double roll_sqrt = sqrt(
			DataStruct->accel_x_raw * DataStruct->accel_x_raw + DataStruct->accel_z_raw * DataStruct->accel_z_raw);
	if(roll_sqrt != 0.0)
	{
		roll = atan(DataStruct->accel_y_raw / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0;
	}

	double pitch = atan2(-DataStruct->accel_x_raw, DataStruct->accel_z_raw) * RAD_TO_DEG;
	if((pitch < -90 && DataStruct->kalman_ang_y > 90) || (pitch > 90 && DataStruct->kalman_ang_y < -90))
	{
		KalmanY.angle = pitch;
		DataStruct->kalman_ang_y = pitch;
	}
	else
	{
		DataStruct->kalman_ang_y = Kalman_Get_Angle(&KalmanY, pitch, DataStruct->gy, dt);
	}

	if (fabs(DataStruct->kalman_ang_y)>90)
		DataStruct->gx = -DataStruct->gx;
	DataStruct->kalman_ang_x = Kalman_Get_Angle(&KalmanX, roll, DataStruct->gx, dt);

	// Calculate yaw with complementary filter
	DataStruct->ang_z += DataStruct->gz * dt;
}

double Kalman_Get_Angle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
	double rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	double S = Kalman->P[0][0] + Kalman->R_measure;
	double K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	double y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	double P00_temp = Kalman->P[0][0];
	double P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}



