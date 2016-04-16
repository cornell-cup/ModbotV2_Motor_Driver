/*
 * Kangaroo_Driver_Lib.c
 *
 *  Created on: Feb 27, 2016
 *      Author: Yuqi (Mark) Zhao and Gustavo Oliveira
 */

#include "Kangaroo_Driver_Lib.h"
#include <stdio.h>
#include <stdlib.h>

#include "COMMAND_LIST.h"
#include "mraa.h"
#include <unistd.h>


/* This function should be called at the initialization of the setup.
 * This configures the Intel Edison to communicate with sabertooth through uart (8N1, BAUDRATE baud).
 * Returns the uart context that is setup.
 */
mraa_uart_context uart_setup() {

    mraa_uart_context uart0 = mraa_uart_init(0);
    if (uart0 == NULL) {
    	fprintf(stderr, "UART failed to setup\n");
    	return EXIT_FAILURE;
        }
    else{
    	printf("UART initialized\n");
    }
    mraa_uart_set_mode(uart0, 8,MRAA_UART_PARITY_NONE , 1);
  	mraa_uart_set_baudrate(uart0, BAUDRATEKANG);

  	//wait for 3 seconds to allow time for things to settle down
  	sleep(3);
	return uart0;
}
/* Sets up the uart2 GS0 serial terminal. Disconnects debug console
 */
mraa_uart_context uartgs0_setup() {
	 mraa_uart_context uart2;
	 /*
	    if (detach_console()) {
	        fprintf(stdout, "Failed to detach system console.\n");
	        fflush(stdout);
	        return EXIT_FAILURE;
	    }
	    */

	    uart2 = mraa_uart_init_raw("/dev/ttyGS0");
	    //If this doesn't work type in "systemctl stop clloader"
	    if (uart2 == NULL) {
	        fprintf(stdout, "UART2 failed to setup\n");
	        return EXIT_FAILURE;
	    }
	    else{
	        printf("UART2 initialized\n");
	    }
	    mraa_uart_set_mode(uart2, 8,MRAA_UART_PARITY_NONE , 1);
	    mraa_uart_set_baudrate(uart2, BAUDRATEMOBO);

	    //wait for 3 seconds to allow time for things to settle down
	    sleep(3);
	    return uart2;
}
/**
 * Stops the uart connection and returns EXIT_SUCCESS if successful
 */
int uart_destroy(mraa_uart_context uart){
    mraa_uart_stop(uart);
    mraa_deinit();
    fprintf(stdout, "UART DESTROYED\n");
    return EXIT_SUCCESS;
}

/**
 * Stops the uart2 connection and returns EXIT_SUCCESS if successful. Reconnects debug console
 * WE DON'T USE THIS NOW
 */
int uart2_destroy(mraa_uart_context uart){
    mraa_uart_stop(uart);
    mraa_deinit();
    reattach_console();
    fprintf(stdout, "Console reattached.\n");
    return EXIT_SUCCESS;
}

/*
 * Generates the crc from the array of commands to be sent. The crc must be generated
 * and sent in order to confirm the command. The Kangaroo will not work without reading crc.
 * Input: data - array of uint8_t that contains the command to be sent
 * 		   length - length of command buffer
 * Output: uint16_t corresponding to the generated crc.
 */
uint16_t crc14(const uint8_t* data, size_t length) {
	uint16_t crc = 0x3fff; size_t i, bit;
	for (i = 0; i < length; i ++){
		crc ^= data[i] & 0x7f;
		for (bit = 0; bit < 7; bit ++){
			if (crc & 1) {
				crc >>= 1; crc ^= 0x22f0;
			}
			else{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0x3fff;
}

/*
 * ! Bit-packs a number.
 *param buffer The buffer to write into.
 *param number The number to bit-pack. Should be between -(2^29-1) and 2^29-1.
 *return How many bytes were written (1 to 5).
 */
size_t bitpackNumber(uint8_t* buffer, int32_t number)
{
 size_t i = 0;

 if (number < 0) { number = -number; number <<= 1; number |= 1; }
 else { number <<= 1; }

 while (i < 5)
 {
 buffer[i ++] = (number & 0x3f) | (number >= 0x40 ? 0x40 : 0x00);
 number >>= 6; if (!number) { break; }
 }

 return i;
}

int32_t unpackNumber(uint8_t* buffer, uint8_t dataLength){

	uint32_t encodedNumber = 0;
	uint8_t shift = 0;
	int i;
	for (i = 6; i < (6 + dataLength - 3); i++) {
		encodedNumber |= (uint32_t)(buffer[i] & 0x3f) << shift;
		shift +=6;
	}
	return (encodedNumber & 1) ? -(int32_t)(encodedNumber >> 1) : (int32_t)(encodedNumber >> 1);
}

/**********************************************************************************************
 * Function:        write_kangaroo_command(uint8_t address, uint8_t command, const uint8_t* data,
                                           uint8_t length, uint8_t* buffer)
 * Input:           address: the address of the Kangaroo.
 *                  command: the command number.
 *                  data: the command data.
 *                  length: the number of bytes of the data.
 *                  buffer: the buffer to write into.
 * Output:          How many bytes were written.
 * Notes:           Use this function to write a command into a buffer. The buffer can be
 *                  used to send the command to the Kangaroo using the mraa_uart_write function.
 *********************************************************************************************/

size_t write_kangaroo_command(uint8_t address, uint8_t command, const uint8_t* data,
                              uint8_t length, uint8_t* buffer){

    size_t i;
    uint16_t crc;

    buffer[0] = address;
    buffer[1] = command;
    buffer[2] = length;
    for (i = 0; i < length; i++) {
        buffer[3 + i] = data[i];
    }
    crc = crc14(buffer, length + 3);
    buffer[3 + length] = crc & 0x7F;		// crc low byte
    buffer[4 + length] = (crc >> 7) & 0x7F; // crc high byte

    return 5 + length;
}

/**********************************************************************************************
 * Function:        start_channel(uint8_t address, uint8_t channel_name)
 * Input:           address: the address of the Kangaroo.
 *                  channel_name: the name of the channel that will be initialized.
 *                  uart: the uart context to be written to.
 * Output:          None.
 * Notes:           Starts a channel. The Kangaroo LED will shine brightly for a third of a
 *                  second. You must call this before Units, Home or Move commands will work.
 *********************************************************************************************/
void start_channel(mraa_uart_context uart, uint8_t address, uint8_t channel_name){
	fprintf(stdout, "Initializing channel!\n");

    uint8_t buffer[7];
    uint8_t data[2];
    // data for start command
    data[0] = channel_name;         // channel name, usually "1" or "2"
    data[1] = 0;                    // flags
    struct velocity_Data test;

    test.readFlag = -1;

    do {

    	write_kangaroo_command(address, CMD_START, data, 2, buffer);

    	mraa_uart_write(uart, buffer, 7);

    	test = readMoveSpeed(uart, address, channel_name);

    	//Clear the Read buffer from the Kangaroos
    	clearRead(uart);
    } while(test.readFlag != 0);

    fprintf(stdout, "Channel Initialized!\n");
}

/**********************************************************************************************
 * Function:        writeMoveSpeed (mraa_uart_context uart, uint8_t address, uint8_t channel_name, 
 * 				    int32_t velocity)
 * Input:           address: the address of the Kangaroo.
 *                  channel_name: the name of the channel that will be initialized.
 *                  uart: the uart context to be written to.
 *		    velocity: the desired motor speed.
 * Output:          None.
 * Notes:           Moves the motor at a speed (in mm/s), given our motor setup.
 *********************************************************************************************/
void writeMoveSpeed(mraa_uart_context uart, uint8_t address, uint8_t channel_name, int32_t velocity){
	uint8_t flag = 0; // No options
	uint8_t type = 2; // Speed Control
	uint8_t data[8];
	uint8_t length = 0;
	data[length ++] = channel_name;
	data[length ++] = flag;
	data[length ++] = type;
	length += bitpackNumber(&data[length], velocity);
	uint8_t buffer[length+5];

	write_kangaroo_command(address, CMD_MOVE, data, length, buffer);
	mraa_uart_write(uart, buffer, length+5);

}

/**********************************************************************************************
 * Function:        power_down_channel(mraa_uart_context uart, uint8_t address, uint8_t channel_name)
 * Input:           address: the address of the Kangaroo.
 *                  channel_name: the name of the channel that will be initialized.
 *                  uart: the uart context to be written to.
 * Output:          None.
 * Notes:           Powers down the channel.
 *********************************************************************************************/
void power_down_channel(mraa_uart_context uart, uint8_t address, uint8_t channel_name){
	uint8_t flag = 0; // No options
	uint8_t sys_cmd_num = SYS_CMD_PWRDOWN; // power down channel
	uint8_t length = 0;
	uint8_t data[3];
	
	data[length++] = channel_name;
	data[length++] = flag;
	data[length++] = sys_cmd_num;
	
	uint8_t buffer[length+5];
	
	write_kangaroo_command(address, CMD_SYSTEM, data, length, buffer);
	mraa_uart_write(uart, buffer, length+5);
}

/**********************************************************************************************
 * Function:        readMoveSpeed (mraa_uart_context uart, uint8_t address, uint8_t channel_name)
 * Input:           address: the address of the Kangaroo.
 *                  channel_name: the name of the channel that will be initialized.
 *                  uart: the uart context to be written to.
 * Output:          A struct containing the velocity and error codes of the read data
 * Notes:
 *********************************************************************************************/
struct velocity_Data readMoveSpeed(mraa_uart_context uart, uint8_t address, uint8_t channel_name){
	uint8_t flag = 0; // No options
	uint8_t parameters = 2; //Current Speed
	uint8_t data[3];
	uint8_t length = 0;
	data[length ++] = channel_name;
	data[length ++] = flag;
	data[length ++] = parameters;
	uint8_t buffer[length+5];

	//First send the "GET" command
	write_kangaroo_command(address, CMD_GET, data, length, buffer);
	mraa_uart_write(uart, buffer, length+5);
	int maxLength = 13;
	uint8_t dataBuffer[maxLength]; //Maximum size of return data

	mraa_uart_read(uart, dataBuffer, 13);
	fprintf(stdout,"\nNEW CMD:");

	//Decode the data
	struct velocity_Data returnData;
	if((dataBuffer[0] == address) && (dataBuffer[1] == CMD_REPLY) && (dataBuffer[3] == channel_name)){
		//The data buffer is correct
		returnData.readFlag = dataBuffer[4];
		uint8_t dataLength = dataBuffer[2];
		int32_t velocity = unpackNumber(dataBuffer, dataLength);
		returnData.value = velocity;
	}
	else{
		returnData.readFlag = -1;
		returnData.value = 0;
		fprintf(stdout, "Read speed unsuccessful!");
	}
	fprintf(stdout, "readFlag: %d, value: %d\n", returnData.readFlag, returnData.value);

	return returnData;
}


/**********************************************************************************************
 * Function:        clearRead(mraa_uart_context uart)
 * Input:           uart: the uart context to be cleared
 * Output:          None.
 * Notes:           Clears the read buffer of the uart context
 *********************************************************************************************/
void clearRead(mraa_uart_context uart){
	while(mraa_uart_data_available(uart, 0)){
		uint8_t clearBuffer[1];
		mraa_uart_read(uart, clearBuffer, 1);
		fprintf(stdout, "Cleared one byte\n");
	}
}

/**********************************************************************************************
 * Function:        readAndSetMotors
 * Input:           buffer: Pointer to the buffer of speeds Size should be 4. (LF RF LB RB)
 * 					uart: The uart context to be read.
 * 					uint8_t adrFront: address of front motors
 * 					uint8_t adrBack: address of rear motors
 * 					uint8_t chLeft: channel of the left motors
 * 					uint8_t chRight: channel of the right motors
 * Output:
 * Notes:			Reads motor speeds from the motherboard. Formatted according to ModBot base station code
 * This function assumes that there is
 *********************************************************************************************/
void readMotors(mraa_uart_context uart, int32_t* buf){
	char firstByte[1] = {0}; //First byte of the array.
	char directions[1] = {0};
	uint8_t speeds[4] = {0};
	char endChar[1] = {0};

	//int32_t kangarooSpeeds[4] = {0};

	float speedFactor = MAX_SPEED / 255;

	mraa_uart_read(uart, firstByte, 1);

	//If the first byte is M, then do the processing
	if(firstByte[0] == 'M'){
		mraa_uart_read(uart, directions, 1);
		mraa_uart_read(uart, speeds, 4);
		mraa_uart_read(uart, endChar, 1);

		if(endChar[0] != '#'){
			fprintf(stdout,"Bad Ending Char");
			return;
		}

		int32_t LF = speeds[0] * speedFactor;
		int32_t RF = speeds[1] * speedFactor;
		int32_t LB = speeds[2] * speedFactor;
		int32_t RB = speeds[3] * speedFactor;
		fprintf(stdout, "INCOMING SPEEDS: %d %d %d %d\n",speeds[0], speeds[1], speeds[2], speeds[3]);

		char directionChar = directions[0];

		char LFDir = (0x08 & directionChar);
		if(LFDir == 0x00){
			LF = LF * -1;
		}
		char RFDir = (0x04 & directionChar);
		if(RFDir == 0x00){
			RF = RF * -1;
		}
		char LBDir = (0x02 & directionChar);
		if(LBDir == 0x00){
			LB = LB * -1;
		}
		char RBDir = (0x01 & directionChar);
		if(RBDir == 0x00){
			RB = RB * -1;
		}

		//If the speeds are within 1% of 0 - MAX_SPEED, then disregard and set equal to 0.
		int32_t deadband = MAX_SPEED * DEADBAND;
		if(abs(LF) < deadband ){
			LF = 0;
		}
		if(abs(RF) < deadband){
			RF = 0;
		}
		if(abs(LB) < deadband){
			LB = 0;
		}
		if(abs(RB) < deadband){
			RB = 0;
		}
		//Write the speeds
		buf[0] = LF;
		buf[1] = RF;
		buf[2] = LB;
		buf[3] = RB;
	}
	else{
		//Garbage
		clearRead(uart);
		fprintf(stdout,"CLEARED GARB");
	}
	return;
}

