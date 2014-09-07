/********************************************************
	        Library for PID Control	
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


extern double KPr, KIr, KDr;
extern double KPp, KIp, KDp;
extern double KPy, KIy, KDy;


#ifndef PID_H_
#define PID_H_


// Configure PID
#define OUT_MAX_RP		 150			// Out PID maximum
#define OUT_MAX_Y		 150			// Out PID maximum
#define I_MAX_RP		 80			// Out I_term maximum
#define I_MAX_Y			 80			// Out I_term maximum
#define DIR				 1				// Direct PID Direction
//#define DIR			-1				// Inverse PID Direction


// Define functios
int PID(double setpoint, double *yt, double *iterm, double kp, double ki, double kd, int out_max, int i_max);		// PID process

#endif /* PID_H_ */
