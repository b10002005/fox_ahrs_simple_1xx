#include"seriallib.h"

int open_serial_port_com(char serial_port[20], long int badu)
  {
  //serial port opening
	int fd;
	struct termios oldtio, newtio;
  	fd=open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

  	if (fd<0) {
  	perror ("FD opened\n");
	     perror (serial_port);
	   //exit(-1);
  	}
	if (fd>0){
  	tcgetattr(fd,&oldtio);
  	bzero(&newtio,sizeof(newtio));
	if(badu==115200){newtio.c_cflag=(BAUDRATE115200 | CS8 | CLOCAL | CREAD);}
	if(badu==38400){newtio.c_cflag=(BAUDRATE38400 | CS8 | CLOCAL | CREAD);}
	if(badu==19200){newtio.c_cflag=(BAUDRATE19200 | CS8 | CLOCAL | CREAD);}
	if(badu==9600){newtio.c_cflag=(BAUDRATE9600 | CS8 | CLOCAL | CREAD);}
	if(badu==4800){newtio.c_cflag=(BAUDRATE4800 | CS8 | CLOCAL | CREAD);}
	if((badu!=4800)&&(badu!=9600)&&(badu!=19200)&&(badu!=38400)&&(badu!=115200))
		{
		perror("worning error on badurate serial device set slowat 4800");
		newtio.c_cflag=(BAUDRATE4800 | CS8 | CLOCAL | CREAD);
		}
  	newtio.c_iflag= IGNPAR; //| ICRNL;
  	newtio.c_oflag=0;
  	newtio.c_lflag=ICANON;
  
  	tcflush(fd,TCIFLUSH);
  	tcsetattr(fd,TCSANOW,&newtio);
	}

	return fd;
	//end setting serial port
  } 


int serial_send (int fd,data_AHRS_Struct buffer)
{
	int writed;
	writed=write(fd,(void*)&buffer, sizeof(buffer));
	tcflush(fd,TCIFLUSH);
	//printf("writed %d \n",writed);
 	 if(writed==sizeof(buffer))
     	{
		return 1;
	}
 	 else
  	{
		return 0;
	}
}
