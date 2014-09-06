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


#include "PID.h"

double KPr = (double)DIR * 3.5;
double KIr = (double)DIR * 0.2;
double KDr = (double)DIR * 50.0;

double KPp = (double)DIR * 3.5;
double KIp = (double)DIR * 0.2;
double KDp = (double)DIR * 50.0;

double KPy = (double)DIR * 8.0;
double KIy = (double)DIR * 0.1;
double KDy = (double)DIR * 35.0;

int PID(double setpoint, double *yt, double *iterm, double kp, double ki, double kd, int out_max, int i_max)
{
	//Compute all the working error variables
	double error, dyt;
	double ut;
	
	error = setpoint - yt[0];
	*iterm += ki * error;
	if (*iterm > i_max) *iterm = i_max;
	else if (*iterm < -i_max) *iterm = -i_max;	
	dyt = (yt[0] - yt[1]);	
		
	//Compute PID Output (Input Process)
	ut = kp * error + *iterm - kd * dyt;
	
	if (ut > out_max) ut = out_max;
	else if (ut < -out_max) ut = -out_max;		
	// Save state actual
	yt[1] = yt[0];	
	return (int)ut;
}

