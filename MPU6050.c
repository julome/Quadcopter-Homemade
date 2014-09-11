/*********************************************************************
					Library for IMU MPU6050
			Copyright (C) 2014  Juan Lopez Medina

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	You can contact me in julome21@gmail.com

    Library for IMU MPU6050  Copyright (C) 2013  Juan Lopez Medina
	
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
 ****************************************************************************/ 


/**************************************************************************
	Examples:
			- To initialize: void imu_init();  // Put in your setup code after initialize TWI and interrupts are enabled
			- To write:		 TWI_Write(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);				// Registry, data to write
			- To read:		 TWI_Read(MPU6050_RA_GYRO_XOUT_H, gyro, 6);								// Registry, array read, bytes to read

****************************************************************************/

#include "MPU6050.h"
#include "twi_master.h"
#include "util/delay.h"

#define F_CPU           16000000UL			// Frequency XTAL 16MHz

// Write byte to IMU through TWI
void TWI_Write(unsigned char reg, unsigned char data){  // reg= Direction de registro, data= data to write

unsigned char messageBuf[8] = {0};					// Buffer for TX through TWI

messageBuf[0] = MPU6050_DEFAULT_ADDRESS;			// TWI slave address (IMU) + Write.
messageBuf[1] = reg;								// Registry Address to write.
messageBuf[2] = data;								// Data to Write to IMU.
TWI_Start_Transceiver_With_Data( messageBuf, 3 );	// TX Reg+Data to Write IMU
return;
}

// Read measurements of IMU Through TWI
void TWI_Read(unsigned char reg, int *result, unsigned char size){
	
	unsigned char messageBuf[8] = {0};					// Buffer for TX through TWI
	
	messageBuf[0] = MPU6050_DEFAULT_ADDRESS;			// TWI slave address (IMU) + Write.
	messageBuf[1] = reg;								// Registry Address to write.
	TWI_Start_Transceiver_With_Data(messageBuf, 2);		// TX Reg to Write IMU
	while (TWI_Transceiver_Busy());						// Wait until TWI is ready for next transmission.
	if (TWI_statusReg.lastTransOK){						// Check if the last operation was successful
		// Request/collect the data from the Slave
		messageBuf[0] = (MPU6050_DEFAULT_ADDRESS | 0x01);			// TWI slave address (IMU) + Read.
		TWI_Start_Transceiver_With_Data(messageBuf, size + 1);
	} else return;													// Out of function
	while (TWI_Transceiver_Busy());									// Wait until TWI is ready for next transmission.
	if (TWI_statusReg.lastTransOK){									// Check if the last operation was successful
		TWI_Get_Data_From_Transceiver(messageBuf, size + 1);
		if (size > 1){		
			for (int i = 0; i < (size / 2); i++)						// Get reads 16 bit on array result
			{
				result[i] = (((int)messageBuf[i * 2]) << 8) | (int)messageBuf[(i * 2) + 1];
			}
		} else result[0] = messageBuf[0];
	} else return; 										// Out of function
}

// Write bit to IMU
void TWI_WriteBit(unsigned char reg, unsigned char bit_num, unsigned char data ){
	int read_aux[1];	
	unsigned char data_add = 0;
	
	TWI_Read(reg, read_aux, 1);							// Read byte of a registry of IMU
	if (data == True) data_add |= (1 << bit_num);		// Select false or true for bit to write
		else data_add &= ~(1 << bit_num);				
	read_aux[0] |= data_add;							// Compound new data with bit for write	
	TWI_Write(reg, read_aux[0]);						// Write byte actualized	
}

// Initialize IMU MPU-6050
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their scale +/- xg and +/- x degrees/sec, and sets the clock source to use the
 * X Gyro for reference, which is slightly better than the default internal clock source.
 */

void imu_init(){
	TWI_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, True);	// Device Reset
	_delay_ms(150);
	TWI_Write(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);					// Clock selection PLL Z_GYRO temp disables
	_delay_ms(2);
	TWI_Write(MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);						// Sample rate divider at 19 => Sample = 1KHz/(1+19) = 50 Hz with 1 KHz sample if DLPF ON
	_delay_ms(2);
	TWI_Write(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_1000);					// Gyro Scale +/- 1000 º/s
	_delay_ms(2);
	TWI_Write(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_8);						// Accel Scale at +/- 8 G
	_delay_ms(2);
	TWI_Write(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_98);							// Config DLPF Low Pass Filter IMU Off
	_delay_ms(2);
	TWI_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, True);		// Disable Temperature
	_delay_ms(2);
	TWI_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, False);			// Disable sleep mode for wake up IMU
	_delay_ms(2);
}
