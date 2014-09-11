/********************************************************
 Source code .c file Library for DCM matrix attitude 
 - Based on William Premerlani paper and Jose Julio code for DCM matrix
 	
    Copyright (C) 2014  Juan Lopez Medina

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

* You can contact me by mail : julome21@gmail.com
*********************************************************/


// Mode 0 for out angle only gyro 
// Mode 1 for out angles gyro + accel
// Mode 2 for out angles only accelerometer
// Any mode time execution 2 ms therefore time_sample must be less more than 2 ms
#define F_CPU           16000000UL			// Frequency XTAL 16MHz
#define OUTPUTMODE	1

#include "DCM.h"
#include "MPU6050.h"
#include <math.h>
#include "util/delay.h"
//#include <stdlib.h>
#include <avr/io.h>

#define rad_grad		180.0/M_PI	
#define grad_rad		M_PI/180.0
#define scale_gyro2000	grad_rad/16.4		// Scale gyro +/- 2000º/s
#define scale_gyro1000	grad_rad/32.8		// Scale gyro +/- 1000º/s
#define scale_gyro500	grad_rad/65.5		// Scale gyro +/- 500º/s
#define scale_gyro250	grad_rad/131.0		// Scale gyro +/- 250º/s
#define Gain_accel_z      1.0/1.042			// 1.042 Gravity Z raw reader
#define Gain_accel_x      1.0/1.007			// 1.007 Gravity X raw reader
#define Gain_accel_y      1.0				// 1.000 Gravity Y raw reader
#define scale_accel16	2048				// Scale Accel +/- 16 G
#define scale_accel8	4096				// Scale Accel +/- 8 G
#define scale_accel4	8192				// Scale Accel +/- 4 G

const double Gain_accel[3] = {Gain_accel_x, Gain_accel_y, Gain_accel_z};
double a_result[3];				// Roll, pitch, yaw
double gyro;					// Move reference for Yaw control

static double DCM_matrix[3][3]= {{1,0,0},{0,1,0},{0,0,1}};
static int a_gyro[3];
static int accel_correct[3];
static int offset_gyro[3];
static double omega[3];	

// Computes the dot product of two vectors
double vector_dot_product(double *vector1, double *vector2){
	double aux = 0;
	
	for(int i = 0; i < 3; i++){
		aux += vector1[i] * vector2[i];
	}	
	return aux; 
}

// Computes the cross product of two vectors
void vector_cross_product(double v1[3], double v2[3], double out[3]){
	out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar.
void vector_scalar(double vectorin[3], double scalar, double *vectorout){
	for(int i = 0; i < 3; i++){
		vectorout[i] = vectorin[i] * scalar;
	}
}

// Add two vectors
void vector_add(double *vectorin, double vector_added[3]){
	for(int i = 0; i < 3; i++){
		vector_added[i] += vectorin[i];
	}
}

// Offset Gyro
void gyro_offset(void){
	int gyro[3];
		
	_delay_ms(500);	
	// Begin Read		
	for (int i = 0; i < 32; i++){
		TWI_Read(MPU6050_RA_GYRO_XOUT_H, gyro, 6);		// Gyro meters		
		offset_gyro[0] += gyro[0];
		offset_gyro[1] += gyro[1];
		offset_gyro[2] += gyro[2];		
		_delay_ms(30);
	}
	offset_gyro[0] = (offset_gyro[0] >> 5);
	offset_gyro[1] = (offset_gyro[1] >> 5);
	offset_gyro[2] = (offset_gyro[2] >> 5);		
	PORTB |= (1 << LED_READY); 													// Ready Led Pin13 On																	
	_delay_ms(200);																// Delay for ready
}

//Low Pass Filter
int lpf(int pv_sf, int pv_fil_ant, double kf){
	double pv_fil;
	pv_fil = (kf * (double)pv_sf + (1 - kf) * (double)pv_fil_ant);		//pv_fil = kf * pv_sf + (1 - kf) * pv_fil_ant;
	return (int)pv_fil;
}

// Samples Components from Gyro and accelerometer
void sample_meters(void){
	int gyro[3] = {0, 0, 0};						// Meters axis X, Y, Z Gyroscope
	int gyro_correct[3] = {0, 0, 0};				// Gyroscope components corrected
	static long gyro_correct_ant[3] = {0, 0, 0};	// Gyroscope components corrected k-1		
	int accel[3] = {0, 0, 0};						// Meters axis X, Y, Z accelerometer
								
	TWI_Read(MPU6050_RA_GYRO_XOUT_H, gyro, 6);						// Gyro meters			
	for (int i = 0; i < 3; i++){
		gyro_correct[i] = gyro[i] - offset_gyro[i];						// X, Y, Z Axis			
	}
	for (int i = 0; i < 3; i++){										// Trapezoidal integer
		a_gyro[i] = ((gyro_correct[i] + gyro_correct_ant[i]) >> 1);		// Angle axis X, Y, Z
	}				
	for (int i = 0; i < 3; i++){									// Save k gyro
		gyro_correct_ant[i] = gyro_correct[i];
	}				
	TWI_Read(MPU6050_RA_ACCEL_XOUT_H, accel, 6);					//Accelerometer meters			
	for (int i = 0; i < 3; i++){
		accel_correct[i] = lpf(accel[i], accel_correct[i], 0.1);		// Low pass filter x, y, z axis										
	}													
}

void normalize(void)
{
	double error = 0;				// Orthogonal error
	double aux[3];					// Auxiliary result error/2 * (row)
	double renorm = 0;				// Auxiliary for renormalized
	
	error = -vector_dot_product(&DCM_matrix[0][0], &DCM_matrix[1][0]) * 0.5;			// Orthogonal -error/2	

	vector_scalar(&DCM_matrix[1][0], error, aux);										// -error/2 * Y
	vector_add(aux, &DCM_matrix[0][0]);													// X(orthogonal) = x - error/2 * y
	
	vector_scalar(&DCM_matrix[0][0], error, aux);										// -error/2 * x
	vector_add(aux, &DCM_matrix[1][0]);													// Y(orthogonal) = Y - error/2 * x
	
	vector_cross_product(&DCM_matrix[0][0], &DCM_matrix[1][0], &DCM_matrix[2][0]);		// Z(orthogonal) = X x Y
		
	for (int i = 0; i < 3; i++){														// For each row
		renorm = 0.5 * (3 - vector_dot_product(&DCM_matrix[i][0], &DCM_matrix[i][0]));	// (3 - X * X)/2
		vector_scalar(&DCM_matrix[i][0], renorm, &DCM_matrix[i][0]);					// X(nomralied) = x(otrhogonal) * (3 - X * X)/2
	}
}


void drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift. 
	double omega_P[3];						// Proportional error
	static double omega_I[3];				// Integer error
	double errorRollPitch[3]= {0,0,0};		// Error from earth orientation
	double accel_vector[3];					// Accelerometer vector
  
	//***** Roll and Pitch drift correction *************** 		
	for (int i = 0; i < 3; i++){
		accel_vector[i] = (double)accel_correct[i] * Gain_accel[i] / scale_accel8;	// Accel vector scaled to G				
	}			
	vector_cross_product(accel_vector, &DCM_matrix[2][0], errorRollPitch);		// Error from accelerometer	
	for (int i = 0; i < 3; i++){
		omega_P[i] = errorRollPitch[i] * Kp_ROLLPITCH;		// error proportional	
		omega_I[i] += errorRollPitch[i] * Ki_ROLLPITCH;		// error integer	
		omega[i] = omega_P[i] + omega_I[i];					// PI control
		//omega[2] = 0;										// Only if only have accelerometer and gyro		
	}	
}

// Multiply two 3x3 matrix 
void matrix_multiply(double a[3][3], double b[3][3], double mat[3][3])
{
	double op[3];	
	
	for(int x = 0; x < 3; x++){
		for(int y = 0; y < 3; y++){
			for(int w = 0; w < 3; w++){
				op[w] = a[x][w] * b[w][y];
			}			
			mat[x][y] = op[0] + op[1] + op[2];						
		}
	}
}

void matrix_update(unsigned char t, double gyro)
{		
	double update_matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}};		// Matrices for dq = w * dt
	double aux_matrix[3][3]={{0,0,0},{0,0,0},{0,0,0}};			// Auxiliary matrices
	double gyro_vector[3];										// Gyro vector scaled
	double dt;													// Incremental time
	double omega_vector[3];										// Drift correction to Gyro vector
		
	dt = 64E-6 * (double)t;												// Take dt in us	
	for (int i = 0; i < 3; i++){								// Gyro vector scaled = (gyro_vector / scale_config) to rad
	gyro_vector[i] = a_gyro[i] * scale_gyro1000;		
	}		
	for (int i = 0; i < 3; i++){								// Add drift correction
	omega_vector[i] = gyro_vector[i] + omega[i];	
	}
	
	omega_vector[2] += gyro;									// Move zero grades reference for Yaw Set point
					
	#if OUTPUTMODE == 1											// Matrix w * dt with drift correction (Accelerometer)
	update_matrix[0][0] = 0;
	update_matrix[0][1] =-dt * omega_vector[2];
	update_matrix[0][2] = dt * omega_vector[1];
	update_matrix[1][0] = dt * omega_vector[2];
	update_matrix[1][1] = 0;
	update_matrix[1][2] =-dt * omega_vector[0];
	update_matrix[2][0] =-dt * omega_vector[1];
	update_matrix[2][1] = dt * omega_vector[0];
	update_matrix[2][2] = 0;
	#endif
	#if OUTPUTMODE == 0											// Matrix w * dt without drift correction (Only gyros)	
	update_matrix[0][0] = 0;
	update_matrix[0][1] =-dt * gyro_vector[2];
	update_matrix[0][2] = dt * gyro_vector[1];
	update_matrix[1][0] = dt * gyro_vector[2];
	update_matrix[1][1] = 0;
	update_matrix[1][2] =-dt * gyro_vector[0];
	update_matrix[2][0] =-dt * gyro_vector[1];
	update_matrix[2][1] = dt * gyro_vector[0];
	update_matrix[2][2] = 0;
	#endif
	
	matrix_multiply(DCM_matrix, update_matrix, aux_matrix);		// Incremental dq body to earth
	for(int x = 0; x < 3; x++){									// R(t) Matrix Addition (update)	
		for(int y = 0; y < 3; y++){
			DCM_matrix[x][y] += aux_matrix[x][y];
		}
	}
}

void euler_angles(void)
{	
	#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)	
	a_result[1] = (atan2(-accel_correct[0],accel_correct[2])) * rad_grad;     // Pitch atan2(acc_y, acc_z)
	a_result[0] = (atan2((accel_correct[1]),accel_correct[2])) * rad_grad;	// Roll atan2(acc_x, acc_z)
	a_result[2] = 0;															// Yaw		
	#else // Euler angles from DCM matrix
	a_result[0] = (atan2(DCM_matrix[2][1],DCM_matrix[2][2])) * rad_grad;		// Roll
	a_result[1] = (asin(-DCM_matrix[2][0])) * rad_grad;						// Pitch
	a_result[2] = (atan2(DCM_matrix[1][0],DCM_matrix[0][0])) * rad_grad;		// Yaw		
	#endif
}
