/*
 * Kangaroo_Driver_Lib.h
 *
 *  Created on: Feb 27, 2016
 *      Author: MarkZhao
 */

#ifndef KANGAROO_DRIVER_LIB_H_
#define KANGAROO_DRIVER_LIB_H_
#include <stdio.h>
#include <stdlib.h>
#include "COMMAND_LIST.h"
#include "mraa.h"

#define MAX_SPEED		4000
#define DEADBAND		0.01
#define BAUDRATEKANG		19200
#define BAUDRATEMOBO			115200

	struct velocity_Data {
		// readFlag contains the flag of the reply command.
		// readFlag = 0 indicates no error in the reading.
		// readFlag = 1 indicates the reply is an error. The value in value
		// will then represent the error code.
		// readFlag = 2 indicates a pending state. If the controller is in motion,
		// the motion is not finished. For an error, the error
		// (such as "not homed") should self-clear.
		char readFlag;
		// if readFlag = 0 or 2, value contains the current speed.
		// if readFlag = 1, value contains the error code.
		int32_t value;
	};

    mraa_uart_context uart_setup();

    int uart_destroy(mraa_uart_context uart);

    mraa_uart_context uart2_setup();

    int uart2_destroy(mraa_uart_context uart);

    uint16_t crc14(const uint8_t* data, size_t length);

    size_t bitpackNumber(uint8_t* buffer, int32_t number);

    int32_t unpackNumber(uint8_t* buffer, uint8_t dataLength);

    size_t write_kangaroo_command(uint8_t address, uint8_t command, const uint8_t* data,
                                  uint8_t length, uint8_t* buffer);

    void start_channel(mraa_uart_context uart, uint8_t address, uint8_t channel_name);

    void writeMoveSpeed(mraa_uart_context uart, uint8_t address, uint8_t channel_name, int32_t velocity);
    
    void power_down_channel(mraa_uart_context uart, uint8_t address, uint8_t channel_name);

    struct velocity_Data readMoveSpeed(mraa_uart_context uart, uint8_t address, uint8_t channel_name);

    void clearRead(mraa_uart_context uart);

    void readMotors(mraa_uart_context uart, int32_t* buf);
#endif /* KANGAROO_DRIVER_LIB_H_ */


