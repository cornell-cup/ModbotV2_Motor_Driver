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
uint8_t address_front = 128;
uint8_t channel_left = '1';
uint8_t channel_right = '2';
uint8_t address_back = 129;

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
    start_channel(uartKang, address_front, channel_left);
    start_channel(uartKang, address_front, channel_right);
    start_channel(uartKang, address_back, channel_left);
    start_channel(uartKang, address_back, channel_right);

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
		fprintf(stdout, "\nWrite speeds: %d %d %d %d\n", speeds[0], speeds[1], speeds[2], speeds[3]);

		//Set speeds on motors
		writeMoveSpeed(uartKang, address_front, channel_left, speeds[LEFT_FRONT]);
		writeMoveSpeed(uartKang, address_front, channel_right, speeds[RIGHT_FRONT]);
		writeMoveSpeed(uartKang, address_back, channel_left, speeds[LEFT_BACK]);
		writeMoveSpeed(uartKang, address_back, channel_right, speeds[RIGHT_BACK]);
	}
    pthread_exit(NULL);
}

void* read_encoder(){

	while(1) {
		//Clear the Read buffer
		clearRead(uartKang);

		int32_t motor_vels[4] = {0}; //Array to hold speeds of motors
		//Read speeds
		velocity_t motor1_vel = readMoveSpeed(uartKang, address_back, channel_right);
		velocity_t motor2_vel = readMoveSpeed(uartKang, address_front, channel_right);
		velocity_t motor3_vel = readMoveSpeed(uartKang, address_back, channel_left);
		velocity_t motor4_vel = readMoveSpeed(uartKang, address_front, channel_left);

		motor_vels[0] = motor1_vel.value;
		motor_vels[1] = motor2_vel.value;
		motor_vels[2] = motor3_vel.value;
		motor_vels[3] = motor4_vel.value;

		//Write the motor velocities to the edison. Commented out until we have functionality to write data.
		//mraa_uart_write(uartEdi, motor_vels, sizeof(motor_vels));
		fprintf(stdout, "\nREAD speeds: %d %d %d %d\n", motor_vels[0], motor_vels[1], motor_vels[2], motor_vels[3]);

		delay(250);
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
