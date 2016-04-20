/**
 * File Name: ModbotV2_Motor_Driver.c
 * Description: Test controlling 2 Kangaroos using the Edison
 */

#include "Kangaroo_Driver_Lib.h"
//#include <time.h>       //for time tracking

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

        //While data is not available, do nothing
        while(!mraa_uart_data_available(uartEdi, 0)) {
            //fprintf(stdout, "Waiting for data...\n");
        }

        //When data becomes available, store it into readBuffer and print speed values
        int32_t speeds[4] = {0};
        readMotors(uartEdi, speeds);
        fprintf(stdout, "\nWrite speeds: %d %d %d %d\n", speeds[0], speeds[1], speeds[2], speeds[3]);

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
*/
