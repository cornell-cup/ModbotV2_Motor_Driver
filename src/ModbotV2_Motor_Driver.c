/**
 * File Name: ModbotV2_Motor_Driver.c
 * Description: Test controlling 2 Kangaroos using the Edison
 */

#include <stdio.h>
#include <stdlib.h>
#include "COMMAND_LIST.h"
#include "Kangaroo_Driver_Lib.h"
#include "mraa.h"
#include <unistd.h>
#include <time.h>       //for time tracking
#include "uart2.h"

//void delay(int milliseconds);

int main()
{
    //Set up the uart connection
    mraa_uart_context uartKang = uart_setup();
    mraa_uart_context uartEdi = uartgs0_setup();

    //Set up Kangaroos parameters
    uint8_t address1 = 128;
    uint8_t channelName1_1 = '1';
    uint8_t channelName1_2 = '2';
    uint8_t address2 = 129;
    uint8_t channelName2_1 = '1';
    uint8_t channelName2_2 = '2';

    //Clear the Read buffers
    clearRead(uartKang);
    clearRead(uartEdi);
    //Start the Kangaroos channels
    start_channel(uartKang, address1, channelName1_1);
    start_channel(uartKang, address1, channelName1_2);
    start_channel(uartKang, address2, channelName2_1);
    start_channel(uartKang, address2, channelName2_2);

    while(1){
    	char readbuffer[5];
    	if(mraa_uart_data_available(uartEdi,0) != 0){
    		mraa_uart_read(uartEdi, readbuffer, 5);
    		printf(readbuffer);
    	}
    	char printbuffer[] = "Hello!";
    	mraa_uart_write(uartEdi, printbuffer, sizeof(printbuffer));
    }

    while(0){

        //Read data from computer
        //While data is not available, do nothing
        while(!mraa_uart_data_available(uartEdi, 0)) {
            //fprintf(stdout, "Waiting for data...\n");
        }

        //When data becomes available, store it into readBuffer and print speed values
        int32_t speeds[4] = {0};
        readMotors(uartEdi, speeds);
        fprintf(stdout, "Write speeds %d %d %d %d:", speeds[0], speeds[1], speeds[2], speeds[3]);

        //Set speeds on motors
        writeMoveSpeed(uartKang, address1, channelName1_1, speeds[0]);
        writeMoveSpeed(uartKang, address1, channelName1_2, speeds[1]);
        writeMoveSpeed(uartKang, address2, channelName2_1, speeds[2]);
        writeMoveSpeed(uartKang, address2, channelName2_2, speeds[3]);

        //Clear the Read buffer
        clearRead(uartKang);

        //Read speeds
        readMoveSpeed(uartKang, address1, channelName1_1);
        readMoveSpeed(uartKang, address1, channelName1_2);
        readMoveSpeed(uartKang, address2, channelName2_1);
        readMoveSpeed(uartKang, address2, channelName2_2);
    }


    //Destroy the uart context
    uart_destroy(uartKang);
    //uart_destroy(uartEdi);  //We get a memory problem if we destroy, so it's not required
    return 0;
}

    /*
    //Code to measure time to set and read speeds on motors

    int32_t speeds[4] = {4000,-4000,-4000,4000};
    clock_t timeWrite;
    timeWrite = clock();
    writeMoveSpeed(uartKang, address1, channelName1_1, speeds[0]);
    writeMoveSpeed(uartKang, address1, channelName1_2, speeds[1]);
    writeMoveSpeed(uartKang, address2, channelName2_1, speeds[2]);
    writeMoveSpeed(uartKang, address2, channelName2_2, speeds[3]);
    timeWrite = clock() - timeWrite;
    double time_taken_write = ((double)timeWrite)/CLOCKS_PER_SEC; // in seconds

    printf("Took %f seconds to set 4 speeds \n", time_taken_write);
    // Took 0.000152 seconds to set 4 speeds
    // Took 0.000099 seconds to set 4 speeds
    // Took 0.000095 seconds to set 4 speeds
    // Took 0.000157 seconds to set 4 speeds

    delay(250);

    clock_t timeRead;
    timeRead = clock();
    readMoveSpeed(uartKang, address1, channelName1_1);
    readMoveSpeed(uartKang, address1, channelName1_2);
    readMoveSpeed(uartKang, address2, channelName2_1);
    readMoveSpeed(uartKang, address2, channelName2_2);
    timeRead = clock() - timeRead;
    double time_taken_read = ((double)timeRead)/CLOCKS_PER_SEC; // in seconds

    printf("Took %f seconds to read 4 speeds \n", time_taken_read);
    // Took 0.001289 seconds to read 4 speeds
    // Took 0.001397 seconds to read 4 speeds
    // Took 0.001263 seconds to read 4 speeds
    // Took 0.001342 seconds to read 4 speeds

    // Code to test uart2

    mraa_uart_context uart1;

    if (detach_console()) {
        fprintf(stdout, "Failed to detach system console.\n");
        fflush(stdout);
        return EXIT_FAILURE;
    }

    uart1 = mraa_uart_init_raw("/dev/ttyMFD2");
    if (uart1 == NULL) {
        fprintf(stdout, "UART2 failed to setup\n");
        return EXIT_FAILURE;
    }
    else{
        printf("UART2 initialized\n");
    }
    mraa_uart_set_mode(uart1, 8,MRAA_UART_PARITY_NONE , 1);
    mraa_uart_set_baudrate(uart1, 9600);

    char buffer[] = "\nHello Mraa!\n";
    mraa_uart_write(uart1, buffer, sizeof(buffer));

    //clearRead(uart1);


   // while(mraa_uart_data_available(uart1, 0) == 0){
        //do nothing
        printf("waiting for data\n");
    //}

    char stop = 0;
    //while(!stop){
        char read[4];
        if(mraa_uart_data_available(uart1, 0) == 0){
            printf("No Data\n");
        }
        else{
            printf("DATA READ");
            stop = 1;
        }
  //  }

    reattach_console();
    fprintf(stdout, "Console reattached.\n");
void delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}*/
