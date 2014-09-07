/*********************************************************************

			Quadcopter with PID control and MPU6050
	Created: 21/01/2014
	
	Improve - DCM IMU instead Complementary Filter
			- Without interrupts to temporize samples
			- ESC at 333 Hz
			- Time sample IMU (Calculate DCM) at 10 ms. Using timer 2 without interrupts and step by 64us.
			- Time control at 2 * Time sapmple (20ms)
			- Commands KPr, KIr, KDr and KPy, KIy, KDy for set PID through USART
			- Readers servo Receiver RX with an Multiplexer and the ICP timer5
			
			
	Based on William Premerlani paper and Jose Julio code for DCM matrix
	
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

    Quadcopter Adaptive  Copyright (C) 2014  Juan Lopez Medina
	
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.

 ****************************************************************************/  


/*****************************************************************************

	- Min_Throttle Futaba 10CP 1108
	- Max_Throttle Futaba 10CP 1932
	- For calibrate ESC Off all control, Disable calibrate_radio and gyro_offset. Go Throttle max...
	- You must write Flash memory and EEPROM when you programming device

******************************************************************************/

#define F_CPU           16000000UL			// Frequency XTAL 16MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "usart.h"
#include "twi_master.h"
#include "MPU6050.h"
#include "DCM.h"
#include "PID.h"
#include "stdlib.h"
#include <util/delay.h>
#include <avr/eeprom.h>

// Configuration
#define T_SAMPLE		10					// Time sample Calculate attitude T = T_SAMPLE * 1ms (max 14 ms).
#define T_CONTROL		2					// CP = T_SAMPLE * T_CONTROL in ms
const int T_CNT = T_SAMPLE * 1000L/64.0;	// Number count temp2.  1 Count Temp1 64us => T_sample = Value_CNT1 = ms/64us
#define CUT_OFF_PWM		120					// Cut Off Out PWM to min from Channel Throttle.
#define RX_TIME_OUT		5					// Time out channel read receiver = T_SAMPLE * RX_TIME_OUT

// Define Out PWMs
#define RB PINB5					// Out PWM control Right-bottom Motor Timer1 OC1A DPIN 11 Arduino Mega
#define RF PINB6					// Out PWM control Right-Front Motor Timer1 OC1B DPIN 12 Arduino Mega
#define LB PINE3					// Out PWM control Left-Bottom Timer3 OC3A DPIN 5 Arduino Mega
#define LF PINE4					// Out PWM control Left-Front Timer3 OC3B DPIN 2 Arduino Mega

//#define mux_c PINC2				// Input Data select C MUX Not Need
#define mux_b PINC1					// Input Data select B MUX DPIN 36
#define mux_a PINC0					// Input Data select A MUX DPIN 37

// Globals Variables								
unsigned char t_sample;									// Save TCNT2 for sample timer
unsigned char t_control = 0;							// Time control actuation
volatile int vch[5] = {1500, 0, 1500, 1500, 1500};		// Array for save width channel signal PPM receiver
volatile unsigned char c = 0;							// Channel read receiver
unsigned char c_ant = 0;								// Past Channel read receiver
unsigned char rx_time_out;								// Time out channel read receiver
volatile unsigned char edge = 1;						// Select rising edge to PPM detect
volatile unsigned int ppm_start = 0;					// Save start value PPM
int vch4_zero = 0;										// Channel Rudder zero set
int vch3_zero = 0;										// Channel Throttle zero set
volatile unsigned char m = 0;							// Index vector Get char USART for commands
char data[20];											// Vector buffer for Get char
volatile unsigned char command;							// Command selected for configure PID

// You must write EEPROM when you programming device
// PID EEPROM values set
float EEMEM kpr = 3.0;
float EEMEM kir = 0.02;
float EEMEM kdr = 75;

float EEMEM kpy = 7.0;
float EEMEM kiy = 0.003;
float EEMEM kdy = 55;

// Interrupt Input Capture Pulse for catch Signal PPM
ISR(TIMER5_CAPT_vect){
	unsigned int ppm;	
	if (edge){								// Rising edge default
		ppm_start = ICR5;				    // Save start PPM
		TCCR5B &= ~(1 << ICES5);		    // Select falling edge
		edge = !edge;						
	} else {		
		ppm = ((ICR5 - ppm_start) >> 1);	// TCNT5 count 1/2 us.  TCNT5/2 = us
		if (ppm > 2050) ppm = 2050;			// Control limits. 1000 = 1 ms to 2000 = 2 ms
		else if (ppm > 900){
			vch[c] = ppm;			
			c ++;
			}					
		if (c >= 4)	c = 0;			        // Limit 4 channel read
		PORTC = (c << mux_a);		        // Select next channel to read
		TCCR5B |= (1 << ICES5);		        // Select rising edge
		edge = !edge;
		TCNT5 = 0;							// Reset counter
		}	
}

// Define functions
void timer2_init();
void timer1_init();
void timer3_init();
void timer5_init();
void calibrate_radio();						// Calibrate TX Radio (Throttle must be at down)
void clearbuffer1(void);					// Clear buffer for Get char
void clearbuffer2(void);					// Clear buffer for Get char and Reset command

			
int main(void)
{										
	// Initialize
	double vch1, vch2, vch4;					// Remapped channels from volatile int	
	int vch3;									// Remapped channel form volatile int	
	int right_bottom;							// Motor RB
	int right_front;							// Motor RF
	int left_bottom;							// Motor LB
	int left_front;								// Motor LF
	int outpitch = 0;							// Out regulator PID control Motor Pitch		
	int outroll = 0;							// Out regulator PID control Motor Roll
	int outyaw = 0;								// Out regulator PID control Motor Yaw
	
	// Pitch (Inclination)											
	double up = 0;								// Array process input u(k)
	double spp = 0;								// Set point process sp(k)
	double ypp[2] = {0};						// Array process out yp(k), yp(k-1)
	double iterm_p = 0;							// Integer terminus balancer	
			
	// Roll (Alabeo)	
	double ur = 0;								// Array process input u(k)
	double spr = 0;								// Set point process sp(k)
	double yrp[2] = {0};						// Array process out yp(k), yp(k-1)
	double iterm_r = 0;							// Integer terminus balancer
	
	// Yaw
	double uy = 0;								// Array process input u(k)
	double spy = 0;								// Set point process sp(k)
	double yyp[2] = {0};						// Array process out yp(k), yp(k-1)
	double iterm_y = 0;							// Integer terminus balancer

	//Configuration output/input pins
	DDRC = (1 << mux_b) | (1 << mux_a);			// Out to Input Data select MUX
	DDRB = (1 << RB) | (1 << RF);				// Out Right-Bottom and Right-Front Motors	
	DDRE = (1 << LB) | (1 << LF);				// Out Left-Bottom and Left-Front Motors			
	PORTC &= ~(1 << mux_a) & ~(1 << mux_b);		// Reset PORTD (Out for MUX Data select)
	DDRB |= (1 << LED_READY);					// Out Led Ready	
	PORTB |= (1 << RB) | (1 << RF);				// Hold output PWM ESC high until initialize timer
	PORTE |= (1 << LB) | (1 << LF);				// Hold output PWM ESC high until initialize timer
	
	//Initialization EEPROM PID SET
	KPr = eeprom_read_float(&kpr);
	KIr = eeprom_read_float(&kir);
	KDr = eeprom_read_float(&kdr);
	KPp = KPr;
	KIp = KIr;
	KDp = KDr;
	KPy = eeprom_read_float(&kpy);
	KIy = eeprom_read_float(&kiy);
	KDy = eeprom_read_float(&kdy);
			
	cli();								// Disable all interrupts
	wdt_disable();						// Disable watchdog				
	usart_init();						// Initialize serial port		
	UCSR0B |= (1 << RXCIE0);			// Enable interrupt RX for commands to set PID (If enabled do not use functions get_xx)
	TWI_Master_Initialise();			// Initialize TWI Port					
	timer2_init();						// Initialize timer2 for time system by 64us	
	timer5_init();						// Initialize Timer5 For ICP radio read servos RX out					
	sei();								// Enable global interrupts			
	imu_init();							// Initialize IMU MPU-6050
	calibrate_radio();					// Calibrate radio TX
	timer1_init();						// Initialize timer 1 for PWM OC1A Right-Bottom Motor, OC1B Right-Front motor
	timer3_init();						// Initialize timer 3 for PWM OC3A Left-Bottom Motor, OC3B Right-Front motor
	gyro_offset();						// Offset gyro	
	wdt_enable(WDTO_30MS);				// Enable watchdog for 30 ms
			  
	// Loop
	while(1){						 										
		if (TCNT2 >= T_CNT){								// Attitude calculates Read IMU (Accel and Gyro) Execute in 2.3ms							
			t_sample = TCNT2;								// Catch sample time for integer angle gyro	
			TCNT2 = 0;										// Restart sample time																	
			wdt_reset();									// Reset watchdog
			sample_meters();								// Samples read IMU
			matrix_update(t_sample, gyro);					// Send t_sample and command yaw for DCM matrix
			normalize();
			drift_correction();									
			t_control++;	
			
			if (c_ant != c) {								// Time out pass channel receiver
				rx_time_out = 0;
				c_ant = c;
			} else {
				rx_time_out ++;
				if (rx_time_out > RX_TIME_OUT) {
					if (c == 1) vch[c] = 1000;		// Throttle down
					else vch[c] = 1500;				// Channel missed centered
					c++;							// Channel passed
					if (c >= 4) c = 0;
					PORTC = (c << mux_a);			// Select next channel to read
					TCCR5B |= (1 << ICES5);			//Select rising edge
					edge = 1;
				}
			}
		
		}											
		
		if (t_control >= T_CONTROL){			// Control action Roll, Pitch, Yaw 						
			t_control = 0;									
			euler_angles();							
						
			vch1 = ((double)vch[0] / 12) - 125;				// Remap vch[0]. Roll (1). Mapped from 1000 - 2000 to (-42º) - (42º) Centered in (0º)
			vch3 = vch[1];									// Remap vch[1]. Throttle (3)
			vch2 = ((double)vch[2] / 12) - 125;				// Remap vch[2]. Pitch (2). Mapped from 1000 - 2000 to (-42º) - (42º) Centered in (0º) 
			vch4 = ((double)vch[3] - 1500 + vch4_zero);		// Remap vch[3]. Yaw (4). Mapped from 1000 - 2000 to -500 - +500
			
			//vch2 = 0;
			//vch3 = 1500;
			//vch4 = 0;
			//vch1 = 0;
			
			// PID process Roll process
			spr = vch1;														// Set point
			yrp[0] = a_result[1];											// Process out y(k).
			if (vch3 > (CUT_OFF_PWM + vch3_zero)){
				ur = PID(spr, yrp, &iterm_r, KPr, KIr, KDr, OUT_MAX_RP, I_MAX_RP);
			}			
			outroll = ur;
			//outroll = 0;
			
					
			// PID process Pitch process
			spp = vch2;														// Set point
			ypp[0] = a_result[0];											// Process out y(k).
			if (vch3 > (CUT_OFF_PWM + vch3_zero)){
				up = PID(spp, ypp, &iterm_p, KPp, KIp, KDp, OUT_MAX_RP, I_MAX_RP);
			}
			outpitch = up;
			//outpitch = 0;			
																
						
			// PID  Yaw process (Only Giro)						
			if ( (a_result[2] < -30) && (vch4 > 15)) vch4 = 0;					// Limit Command Set Point Yaw
				else if ( (a_result[2] > 30) && (vch4 < 15)) vch4 = 0;	
			if ((vch4 > 15) | (vch4 < -15)) gyro = -(vch4 / 250.0);				// Move direction reference 0 for Set point Yaw Process. 			 									
				else gyro = 0;													// Gyro select rate yaw. Reduce constant value for increase rate

			spy = 0;															// Set point
			yyp[0] = a_result[2];												// Process out y(k).
			if (vch3 > (CUT_OFF_PWM + vch3_zero)){
				uy = PID(spy, yyp, &iterm_y, KPy, KIy, KDy, OUT_MAX_Y, I_MAX_Y);
			}
			outyaw = uy;
			//outyaw = 0;												
		}												
		
		// Off control
		//outpitch = 0;
		//outroll = 0;
		//outyaw = 0;
						
		// Adding control pitch, yaw, roll		
		vch3 = vch[1];							// Remap vch[1]. Throttle (3)
		if (vch3 > (CUT_OFF_PWM + vch3_zero)){
			right_bottom = (vch3 << 1) - outpitch - outroll - outyaw;		// Throttle (2000 - 4000) => Centred in 3000 + PID Control 
			left_bottom = (vch3 << 1) - outpitch + outroll + outyaw;													
			right_front = (vch3 << 1) + outpitch - outroll + outyaw;						
			left_front = (vch3 << 1) + outpitch + outroll - outyaw;	
		}else{
			ur = 0; up = 0; uy = 0;			
			right_bottom = vch3 << 1;
			right_front = right_bottom;
			left_bottom = right_bottom;
			left_front = right_bottom;												
			gyro = 0;														// Gyro reference Off											
			iterm_y = 0;
			iterm_p = 0;
			iterm_r = 0;
		}			
		//Limits out motors			
		if (right_bottom > 4000) right_bottom = 4000;
			else if (right_bottom < 2000) right_bottom = 2000;									
		if (right_front > 4000) right_front = 4000;
			else if (right_front < 2000) right_front = 2000;			
		if (left_bottom > 4000) left_bottom = 4000;
			else if (left_bottom < 2000) left_bottom = 2000;						
		if (left_front > 4000) left_front = 4000;
			else if (left_front < 2000) left_front = 2000;

		//Assignments PWMs
		OCR1A = right_bottom;
		OCR1B = right_front;
		OCR3A = left_bottom;
		OCR3B = left_front;			
																
										
	}	
}

// Timer count for samples. 1 count = 64us. Max = 256 * 64us = 16ms
void timer2_init(){
	TCCR2A = 0;												// Normal Mode
	TCCR2B = (1 << CS22) | (1 << CS20) | (1 << CS21);		// Prescaler = 1024 Normal Mode
}

// Initialize Timer1 PWM Fast Mode for control ESC Right motors
void timer1_init(){
	//Fast PWM Mode Fpwm = F_cpu / (N * (End + 1))
	TCCR1B = (2 << CS10);			// Configuration prescaler to 1/8 N = 8
	ICR1 = 5999;					// End count for Fpwm = 50Hz End = 39999 (only for Servos) Fpwm = 333Hz (3.0ms) End = 5999 (only for ESC)
	TCCR1A = (2 << COM1A0);			// No inverter mode PIN OC1A
	TCCR1A |= (2 << COM1B0);		// No inverter Mode PIN OC1B
	// Enable FAST PWM Mode
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	OCR1A = (vch[1] << 1);					// Initialize comparison registry Timer1 PIN A for PWM servo 2000 = 1ms 4000 = 2ms
	OCR1B = (vch[1] << 1);					// Initialize comparison registry Timer1 PIN B for PWM servo 2000 = 1ms 4000 = 2ms
	return;
}

// Initialize Timer3 PWM Fast Mode for control ESC Left motors
void timer3_init(){
	//Fast PWM Mode Fpwm = F_cpu / (N * (End + 1))
	TCCR3B = (2 << CS30);		// Configuration del prescaler a 1/8 N=8
	ICR3 = 5999;				// End count for Fpwm = 50Hz End = 39999 (only for Servos) Fpwm = 333Hz (3.0ms) End = 5999 (only for ESC)
	TCCR3A = (2 << COM3A0);		// No inverter Mode PIN OC3A
	TCCR3A |= (2 << COM3B0);	// No inverter Mode PIN OC3B
	// Enable FAST PWM Mode
	TCCR3A |= (1 << WGM31);
	TCCR3B |= (1 << WGM32) | (1 << WGM33);
	OCR3A = (vch[1] << 1);				//Initialize comparison registry Timer3 PINA for PWM servo 2000=1ms 4000=2ms
	OCR3B = (vch[1] << 1);				//Initialize comparison registry Timer3 PINB for PWM servo 2000=1ms 4000=2ms
	return;
}

// Routine initialize ICP timer5 for catch Signal PPM
void timer5_init(){
	TCCR5A = 0;									// Normal Mode
	TCCR5B |= (2 << CS50);						// Timer prescaler = 8
	TCCR5B |= (1 << ICNC5) | (1 << ICES5);		// Noise Canceler and Rising edge enabled ICP5 D_PIN = 48
	TIMSK5 |= (1 << ICIE5);						// Active interrupt ICP5
}

// Calibrate readers radio for Throttle and Rudder
void calibrate_radio(){
	while ((vch[1] < 1000) || (vch[1] > 1200));					// Wait always if Throttle is not down
	_delay_ms(100);
	for (int i = 0; i < 16; i++){
		_delay_ms(40);
		vch3_zero += vch[1];
		vch4_zero += vch[3];
	}
	vch3_zero = (vch3_zero >> 4);			// Channel 3 Throttle at down
	vch4_zero = 1500 - (vch4_zero >> 4);	// Channel 4 Rudder centered
}
// Commands for SET PID throught USART
ISR(USART0_RX_vect){
	data[m++] = UDR0;
	//UDR0 = data[m-1];																	// Get char
	if ((data[0] == 'K') & (data[1] == 'P') & (data[2] == 'r')) {command = 1;			// Command for KP roll pitch
		clearbuffer1(); put_string("KPr: ");}
	if ((data[0] == 'K') & (data[1] == 'I') & (data[2] == 'r')) {command = 2;			// Command for KI roll pitch
		clearbuffer1(); put_string("KIr: ");}	
	if ((data[0] == 'K') & (data[1] == 'D') & (data[2] == 'r')) {command = 3;			// Command for KD roll pitch
		clearbuffer1(); put_string("KDr: ");}
	if ((data[0] == 'K') & (data[1] == 'P') & (data[2] == 'y')) {command = 4;			// Command for KP yaw
		clearbuffer1(); put_string("KPy: ");}
	if ((data[0] == 'K') & (data[1] == 'I') & (data[2] == 'y')) {command = 5;			// Command for KI yaw
		clearbuffer1(); put_string("KIy: ");}	
	if ((data[0] == 'K') & (data[1] == 'D') & (data[2] == 'y')) {command = 6;			// Command for KD yaw
		clearbuffer1(); put_string("KDy: ");}		
	if ((command == 0) & (m > 2) ) clearbuffer1();										// Reset if wrong command												
	if ((command == 1) & (data[m-1] == '\n') & (atof(data) != 0)) {KPr = atof(data);					// Get Data KP																			
		KPp = KPr; eeprom_update_float(&kpr, KPr); clearbuffer2(); put_float(KPr); put_string("\n");}	
	if ((command == 2) & (data[m-1] == '\n') & (atof(data) != 0)) {KIr = atof(data);					// Get Data KI			
		KIp = KIr; eeprom_update_float(&kir, KIr); clearbuffer2(); put_float(KIr); put_string("\n");}	
	if (command == 3 & data[m-1] == '\n' & atof(data) != 0) {KDr = atof(data);							// Get Data KD						
		KDp = KDr; eeprom_update_float(&kdr, KDr); clearbuffer2(); put_float(KDr); put_string("\n");}			
	if (command == 4 & data[m-1] == '\n' & atof(data) != 0) {KPy = atof(data);							// Get Data KP							
		eeprom_update_float(&kpy, KPy); clearbuffer2(); put_float(KPy); put_string("\n");}		
	if (command == 5 & data[m-1] == '\n' & atof(data) != 0) {KIy = atof(data);							// Get Data KI						
		eeprom_update_float(&kiy, KIy); clearbuffer2(); put_float(KIy); put_string("\n");}
	if (command == 6 & data[m-1] == '\n' & atof(data) != 0) {KDy = atof(data);							// Get Data KD			
		eeprom_update_float(&kdy, KDy); clearbuffer2(); put_float(KDy); put_string("\n");}
	
}

void clearbuffer1(void){			// Reset index vector Get char USART
	m = 0;								
	for (int i = 0; i < 20; i++) data[i] = 0;	
}

void clearbuffer2(void){			// Reset index vector Get char USART and Reset command
	m = 0;
	for (int i = 0; i < 20; i++) data[i] = 0;
	command = 0;
}
