/********************************************************
  Header file Library for DCM matrix attitude 
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

extern double a_result[3];			// Roll, pitch, yaw
extern double gyro;					// Move reference for Yaw control


#ifndef DCM_H_
#define DCM_H_

#define Kp_ROLLPITCH 0.1		// Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.00001	// Pitch&Roll Integrator Gain

#define LED_READY	PINB7		// LED On for Ready only for ATmega168

void matrix_update(unsigned char t, double gyro);
void sample_meters(void);
void matrix_multiply(double a[3][3], double b[3][3], double mat[3][3]);
void euler_angles(void);
void normalize(void);
double vector_dot_product(double *vector1, double *vector2);
void vector_cross_product(double v1[3], double v2[3], double out[3]);
void vector_scalar(double vectorin[3], double scalar, double *vectorout);
void vector_add(double *vectorin, double vector_added[3]);
void drift_correction(void);
void gyro_offset(void);
int lpf(int pv_sf, int pv_fil_ant, double kf);

#endif /* DCM_H_ */
