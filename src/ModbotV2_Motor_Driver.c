/**
 * File Name: ModbotV2_Motor_Driver.c
 * Description: Test controlling 2 Kangaroos using the Edison
 */

#include "Kangaroo_Driver_Lib.h"
#include <pthread.h>
#include <time.h>

// Threads for reading commands and reading encoders
pthread_t thread_command;
pthread_t thread_encoder;

// Kangaroos parameters
uint8_t address1 = 128;
uint8_t channelName1_1 = '1';
uint8_t channelName1_2 = '2';
uint8_t address2 = 129;
uint8_t channelName2_1 = '1';
uint8_t channelName2_2 = '2';

// Uart connections
mraa_uart_context uartKang;
mraa_uart_context uartEdi;

void delay(int milliseconds);
void* receive_command();
void* read_encoder();

int main(){

	//Set up the uart connection
	uartKang = uart_setup();
	uartEdi = uartgs0_setup();

    //Clear the Read buffers
    clearRead(uartKang);
    clearRead(uartEdi);

    //Start the Kangaroos channels
    start_channel(uartKang, address1, channelName1_1);
    start_channel(uartKang, address1, channelName1_2);
    start_channel(uartKang, address2, channelName2_1);
    start_channel(uartKang, address2, channelName2_2);

	pthread_create(&thread_command,NULL,receive_command,NULL);

	pthread_create(&thread_encoder,NULL,read_encoder,NULL);

	while(1);

    //Destroy the uart context
    uart_destroy(uartKang);
    //uart_destroy(uartEdi);  //We get a memory problem if we destroy, so it's not required

	return 0;
}

void* receive_command(){

	while(1){

		//fprintf(stdout,"Reading command!\n");

		//While data is not available, do nothing
		while(!mraa_uart_data_available(uartEdi, 1000)) {
			//fprintf(stdout, "Waiting for data...\n");
		}

		//When data becomes available, store it into readBuffer and print speed values
		int32_t speeds[4] = {0};
		readMotors(uartEdi, speeds);
		//fprintf(stdout, "\nWrite speeds: %d %d %d %d\n", speeds[0], speeds[1], speeds[2], speeds[3]);

		//Set speeds on motors
		writeMoveSpeed(uartKang, address1, channelName1_1, speeds[0]);
		writeMoveSpeed(uartKang, address1, channelName1_2, speeds[1]);
		writeMoveSpeed(uartKang, address2, channelName2_1, speeds[2]);
		writeMoveSpeed(uartKang, address2, channelName2_2, speeds[3]);
	}
    pthread_exit(NULL);
}

void* read_encoder(){

	while(1) {
		//Clear the Read buffer
		clearRead(uartKang);

		//Read speeds
		readMoveSpeed(uartKang, address1, channelName1_1);
		readMoveSpeed(uartKang, address1, channelName1_2);
		readMoveSpeed(uartKang, address2, channelName2_1);
		readMoveSpeed(uartKang, address2, channelName2_2);

		delay(17);
	}

	pthread_exit(NULL);
}

void delay(int milliseconds)
{
	long pause;
	clock_t now,then;

	pause = milliseconds*(CLOCKS_PER_SEC/1000);
	now = then = clock();
	while( (now-then) < pause )
		now = clock();
}
