/*********************************************************************
					Library for TWI AVR
		Based on AVR315 - TWI Master Implementation Atmel Corporation
		
		Improved:
				- Time out for not hold in loop
				- Function TWI_Get_Data_From_Transceiver modified
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

    Library for TWI AVR  Copyright (C) 2014  Juan Lopez Medina
	
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
 ****************************************************************************/ 

#include "avr/io.h"
#include "avr/interrupt.h"
#include "twi_master.h"


static unsigned int time_out = 0;					// Time out exit Loop
static unsigned char TWI_buf[ TWI_BUFFER_SIZE ];    // Transceiver buffer
static unsigned char TWI_msgSize = 0;               // Number of bytes to be transmitted.
static unsigned char TWI_state = TWI_NO_STATE;      // State byte. Default set to TWI_NO_STATE.
union TWI_statusReg TWI_statusReg = {0};            // TWI_statusReg is defined in TWI_Master.h

// Call this function to set up the TWI master to its initial standby state.
// Remember to enable interrupts from the main application after initializing the TWI.
void TWI_Master_Initialise(void){
	TWSR &= ~((1 << TWPS1) | (1 << TWPS0));			   // Prescaler TWI to 0
	TWBR = ((F_CPU / I2C_BAUD) - 16) / 2;			   // SCL Frequency
	TWDR = 0xFF;                                       // Default content = SDA released.
	TWCR = (1 << TWEN)|                                // Enable TWI-interface and release TWI pins.
	(0 << TWIE)|(0 << TWINT)|                          // Disable Interrupt.
	(0 << TWEA)|(0 << TWSTA)|(0 << TWSTO)|             // No Signal requests.
	(0 << TWWC);                                
}        
// Call this function to test if the TWI_ISR is busy transmitting.
unsigned char TWI_Transceiver_Busy( void ){       
  time_out++;
  if (time_out > 1000) {
	  time_out = 0;
	  return 0;				  // Exit loop if time out true	
  }	  
  return ( TWCR & (1 << TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}
/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function 
will return the TWI State code. 
****************************************************************************/
unsigned char TWI_Get_State_Info( void ){
  time_out = 0;
  while ( TWI_Transceiver_Busy() );            // Wait until TWI has completed the transmission.
  return ( TWI_state );                        // Return error state.
}
/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize ){
  unsigned char temp;
  
  time_out = 0;
  while ( TWI_Transceiver_Busy());				// Wait until TWI is ready for next transmission.    
  TWI_msgSize = msgSize;                        // Number of data to transmit.
  TWI_buf[0]  = msg[0];                         // Store slave address with R/W setting.
  if (!( msg[0] & (TRUE << TWI_READ_BIT) ))     // If it is a write operation, then also copy data.
  {
    for ( temp = 1; temp < msgSize; temp++ )
      TWI_buf[ temp ] = msg[ temp ];
  }
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE;
  TWCR = (1 << TWEN)|                               // TWI Interface enabled.
		 (1 << TWIE)|(1 << TWINT)|                  // Enable TWI Interrupt and clear the flag.
		 (0 << TWEA)|(1 << TWSTA)|(0 << TWSTO)|     // Initiate a START condition.
		 (0 << TWWC);                             
}
/****************************************************************************
Call this function to resend the last message. The driver will reuse the data previously put in the transceiver buffers.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver( void ){  	
  time_out = 0;
  while ( TWI_Transceiver_Busy());                   // Wait until TWI is ready for next transmission.
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE ;
  TWCR = (1 << TWEN)|                               // TWI Interface enabled.
         (1 << TWIE)|(1 << TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (0 << TWEA)|(1 << TWSTA)|(0 << TWSTO)|     // Initiate a START condition.
         (0 << TWWC);                             
}
/****************************************************************************
Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
TWI_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
requested (including the address field) in the function call. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI error code.
****************************************************************************/
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize ){
  unsigned char i;
  
  time_out = 0;
  while ( TWI_Transceiver_Busy());              // Wait until TWI is ready for next transmission.

  if( TWI_statusReg.lastTransOK )               // Last transmission competed successfully.              
  {                                             
    for ( i = 0; i < msgSize; i++ )             // Copy data from Transceiver buffer.
    {
      msg[ i ] = TWI_buf[ i + 1 ];
    }
  }
  return( TWI_statusReg.lastTransOK );                                   
}

// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
ISR(TWI_vect){	
  static unsigned char TWI_bufPtr;    
  
  switch (TWSR){
    case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      TWI_bufPtr = 0;           // Set buffer pointer to the TWI Address location    
	case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
      if (TWI_bufPtr < TWI_msgSize){
        TWDR = TWI_buf[TWI_bufPtr++];
        TWCR = (1 << TWEN)|                                 // TWI Interface enabled
               (1 << TWIE)|(1 << TWINT)|                    // Enable TWI Interrupt and clear the flag to send byte
               (0 << TWEA)|(0 << TWSTA)|(0 << TWSTO)|         
               (0 << TWWC);                                   
      }else{                    // Send STOP after last byte      
        TWI_statusReg.lastTransOK = TRUE;                   // Set status bits to completed successfully. 
        TWCR = (1 << TWEN)|                                 // TWI Interface enabled
               (0 << TWIE)|(1 << TWINT)|                    // Disable TWI Interrupt and clear the flag
               (0 << TWEA)|(0 << TWSTA)|(1 << TWSTO)|       // Initiate a STOP condition.
               (0 << TWWC);                               
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
      TWI_buf[TWI_bufPtr++] = TWDR;
    case TWI_MRX_ADR_ACK:       // SLA+R has been transmitted and ACK received
      if (TWI_bufPtr < (TWI_msgSize-1) )					// Detect the last byte to NACK it.
      {
        TWCR = (1 << TWEN)|                                 // TWI Interface enabled
               (1 << TWIE)|(1 << TWINT)|                    // Enable TWI Interrupt and clear the flag to read next byte
               (1 << TWEA)|(0 << TWSTA)|(0 << TWSTO)|       // Send ACK after reception
               (0 << TWWC);                                   
      }else                    // Send NACK after next reception
      {
        TWCR = (1 << TWEN)|                                 // TWI Interface enabled
               (1 << TWIE)|(1 << TWINT)|                    // Enable TWI Interrupt and clear the flag to read next byte
               (0 << TWEA)|(0 << TWSTA)|(0 << TWSTO)|       // Send NACK after reception
               (0 << TWWC);                                  
      }    
      break; 
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
      TWI_buf[TWI_bufPtr] = TWDR;
      TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
      TWCR = (1 << TWEN)|                               // TWI Interface enabled
             (0 << TWIE)|(1 << TWINT)|                  // Disable TWI Interrupt and clear the flag
             (0 << TWEA)|(0 << TWSTA)|(1 << TWSTO)|     // Initiate a STOP condition.
             (0 << TWWC);                              
      break;      
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1 << TWEN)|                                 // TWI Interface enabled
             (1 << TWIE)|(1 << TWINT)|                    // Enable TWI Interrupt and clear the flag
             (0 << TWEA)|(1 << TWSTA)|(0 << TWSTO)|       // Initiate a (RE)START condition.
             (0 << TWWC);                                 
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received    
    case TWI_MTX_DATA_NACK:     // Data byte has been transmitted and NACK received
	//case TWI_NO_STATE              // No relevant state information available; TWINT = “0”
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWI_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.      
	  // Reset TWI Interface      
	  TWCR = (1 << TWEN)|                               // TWI Interface enabled
	  	     (0 << TWIE)|(1 << TWINT)|                  // Disable TWI Interrupt and clear the flag
	  	     (0 << TWEA)|(0 << TWSTA)|(1 << TWSTO)|     // Initiate a STOP condition.
	  	     (0 << TWWC);
	  	  
	  TWCR = (1 << TWEN)|                               // Enable TWI-interface and release TWI pins
             (0 << TWIE)|(0 << TWINT)|                  // Disable Interrupt
             (0 << TWEA)|(0 << TWSTA)|(0 << TWSTO)|     // No Signal requests
             (0 << TWWC);                               	  
  }
}
