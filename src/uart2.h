#ifndef UART2_H_
#define UART2_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int detach_console(void);
int reattach_console(void);

#endif /* UART2_H_ */
