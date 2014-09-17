Quadcopter-Homemade
===================

Quadcopter homemade with an Atmega2560, IMU MPU6050, PID control and attitude calculate DCM matrix. Programmed in AVR-GCC with Atmel Studio 6.

Components:

Frame: Q450
Motors: Turnigy D2836/9 950Kv
Propeller: 10x4.5
ESC: Turnigy Plus 30A
Microcontroller: Atmega2560 (ArduinoMega)
IMU: MPU6050
Level shifter for TWI 


Features:

- First of all, I have developed all this project in AVR-GCC code. I realized that arduino is useful and very easy but in C code you can do all that you want with the AVRs.




- I have calculated attitude based on William Premerlani and Jose Julio code using the DCM matrix. I have to say that the drift that I have obtained is almost zero. I actualize the DCM matrix by 10ms using the timer2 without interrupts and with a timer set by 64us step for improve results.




- The main control PID is executed by 20ms, therefore I read IMU by 10ms and I control pitch, roll and yaw for two IMU readers.




- I have used two timers of 16 bit for control PWM ESC. The frecuency of PWM for ESC is 333Hz.

- For read the servos command from receiver I have used a basic multiplexer 74LS151 and the ICP timer 5 for better performance.

- Finally, I have implemented the USART sending commands for set PID parameters by interrupts. Furthermore PID paremeters are saved in EEPROM, so you must write Flash and EEPROM memories.

https://www.youtube.com/watch?v=f1ZoPM6jPvY

Link Frame: https://drive.google.com/file/d/0B_yc0RUsZBjoSEFaMG1KZ1FvVlU/edit?usp=sharing

Link schematic: https://drive.google.com/file/d/0B_yc0RUsZBjobVZTVU9adEtLTjA/edit?usp=sharing

