#ifndef _SERIAL_COMUNICATION_h_
#define _SERIAL_COMUNICATION_h_


#define BAUDRATE115200 	B115200
#define BAUDRATE38400	B38400
#define BAUDRATE19200	B19200
#define BAUDRATE9600 	B9600
#define BAUDRATE4800    B4800

#include <unistd.h>
#include <stdio.h>		
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdio.h> 
#include <termios.h>
#include <string.h>
#include "comunication_descriptor_data.h"

int open_serial_port_com(char serial_port[20], long int badu);
int serial_send(int fd,data_AHRS_Struct buffer);

#endif
