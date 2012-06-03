#ifndef __GLOBALDATA_H__
#define __GLOBALDATA_H__


#define LATITUDINE_FAKE   44.4939
#define LONGITUDINE_FAKE  11.3718
#define ALTITUDINE_FAKE  15
#define INTERNALUSEPORT 5003


char ip_dest_addrs[20] ="192.168.0.3";
char ip_local_addrs[20]="192.168.0.2";
char imu_port[20]="/dev/i2c-0";
char gps_port[20]="/dev/ttyS3";


  int sk;
  int fd_iduec, fd_gps ; 
  int dim;
  int fd_serial_send=0;
  int fd_serial_gadget=0;

//time
/*timer constant*/
  double time_divisor_clk=1600000;
  double fake_deltat=0.0120;
//kalman
  float dev_std_accell=0.0001;
  float dev_std_mag=0.1;
  #define GPSTIMEOUTCOUNT 75
  #define TMP_PRESS_TIMEOUTCOUNT 30
  #define WMM_TIMEOUTCOUNT 60000
  #define GADGETPORT "/dev/ttyGS0"


#endif
