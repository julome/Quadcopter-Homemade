/********************************************************
 Source code .c file Library for send data through USART AVR	
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

/************** Intructions ********
To use this library you must initilize in you header project by the function: usart_init();

Then you can to use five functions as follow:

put_float(variable);        // Send float data
put_string(" ");            // Send character space
put_int(variable);          // Send int data
put_string(" ");
put_long(variable);         // Send long data
put_string("\n");			// Send newline character

void get_float(void);			// Rx float data
void get_int(void);				// Rx int data

Also you should define the CPU frequency for your board and the baud
rate in this code and in your Terminal Window of Atmel Studio 6 project.

For further information contact with me please. julome21@gmail.com

************************************/
#include "avr/io.h"
#include "stdlib.h"
#include "usart.h"
//#include <stdio.h>								// For use with printf and scanf

// Initialize USART
void usart_init(void){	
	UCSR0A |= (1 << U2X0);							// Config BAUDRATE
	UBRR0 = F_CPU / (8 * USART_BAUD) - 1;	
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);			// Format 8N1 Asynchronous	
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);			// Enable module TX adn RX	
	//fdevopen((int (*)(char, FILE*))put_char, (int (*)(FILE*))get_char);		// For use with printf and scanf	
}

// TX data char through USART
int put_char (int dato){
	while ((UCSR0A & (1 << UDRE0)) == 0);	// Wait for empty buffer
	UDR0 = dato;
	return dato;
}

// TX data string ASCII through USART
void put_string(char *s){
	while (*s){
		put_char(*s);
		s++;
	}
}

// TX integer variable through USART
void put_int (int dato){
	char s[20];
	itoa(dato,s,10);	// Converting data integer to ASCII
	put_string(s);
}

// TX long variable through USART
void put_long (long dato){
	char s[20];
	ltoa(dato,s,10);	// Converting data integer to ASCII
	put_string(s);
}

// Tx float variable through USART
void put_float (float dato){
	char s[20];
	dtostrf(dato,8,3,s);	// Converting data integer to ASCII. 3 Decimals
	put_string(s);
}

// Rx data char through USART 
int get_char(void){
	int dato;	
	while ((UCSR0A & (1<<RXC0)) == 0 );		// Wait for data in buffer	
	dato = UDR0;
	return dato;
}

// Rx string  through USART
float get_float(void){
	char k[20];
	float f;
	int i = 0;
	while (1) {
		k[i] = get_char();
		if (k[i] == '\n') break;
		i++;		
	}	
	f = atof(k);
	put_float(f);		// Echo print
	put_string("\n");
	return f;		
}

// Rx string  through USART
int get_int(void){
	char k[20];
	int data;
	int i = 0;
	while (1) {
		k[i] = get_char();
		if (k[i] == '\n') break;
		i++;
	}
	data = atoi(k);			// ASCII to data int
	put_int(data);			// Echo print
	put_string("\n");
	return data;
}




	

