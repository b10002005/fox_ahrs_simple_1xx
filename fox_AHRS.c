/*
 * File: fox_ahrs.c
 * Autor: Federico LOlli
 *  *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * @author Federico Lolli <federicololli@hotmail.com>
 * @version 0.1
 * @date 21/01/2011
 *
 * Model version                        : 0.1
 * C/C++ source on      	        : 06 01 2011
 *
 * 
 * 
 * 
 * 
 */


#include<math.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdio.h> 
#include <termios.h> 
#include <sys/ioctl.h>
#include <stdlib.h>
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "IMU/iduec.h"
#include "IMU/daisysette.h"

#include "IMU/global_sensor.h"
#include "comunication_descriptor_data.h"
#include "udpsocketlib.h"
#include "globaldata.h"
#include "config/loadconf.h"

/*gps*/
#include "gps/gps.h"

#include "seriallib.h"

void sighandlersigterm(int sig);
datasensincomp correct_whith_calib_uno(datasensincomp data_row,load_data_struct calibr_data);

clock_t BeginTimer();

int crccal(data_AHRS_Struct a);
/*global data*/
load_data_struct load_data;

     
int main(int argc, char *argv[])
{ 

  /*************load global data*****************/
  load_data=load_configuration();
  /*end load data*/
  char ethconfig_comand[30]="ifconfig eth0 ";
  char serial_port_send[30]="";
  strcpy(serial_port_send, load_data.serial_port);
  printf("inizialize control serial port send  %s \n",serial_port_send);
  printf("inizialize control gadget serial port send  %s \n",GADGETPORT);

  strcpy(ip_dest_addrs, load_data.ip_dest_addrs);
  printf("inizialize control ip dest %s \n",ip_dest_addrs);
  strcpy(ip_local_addrs, load_data.ip_local_addrs);
  printf("inizialize control ip local %s \n",ip_local_addrs);
  strcpy(imu_port, load_data.imu_port);
  printf("inizialize control imu port %s \n",imu_port);
  strcpy(gps_port, load_data.gps_port);
  printf("inizialize control gps port %s \n",gps_port);
  //configuro il sitema
  strcat(ethconfig_comand,ip_local_addrs);
  printf("execute command: %s \n",ethconfig_comand);
  system(ethconfig_comand);
  printf("configure gadget\n");
  system("modprobe g_serial");

  //time
  double elapTicks,elapTicks_prec, deltatime;
  //tempo inizio processo
  double elapTicks_init=BeginTimer();
  //dati comunicazione
  data_AHRS_Struct send ;//data to send
  memset(&send,0,sizeof(send));
  NavInputs_t gpsData1;
  int cnt_gps=0;
  int cnt_baro=0;
  int statusbaro=0; 
  int update_tmp_cnt=0;
  long int cnt_WMM=0;
  float tmp_altitude=0,altitude=0;
  /*global data and inizializzation*/
  /*inizialize kalman*/

  double sqw=0;
  double sqx=0;
  double sqy=0;
  double sqz=0;
  double unit=0; // if normalised is one, otherwise is correction factor
  double test=0;
  //quaternione temporaneo

  float q0_t , q1_t , q2_t , q3_t;

  q0=(float)load_data.Q_quat0;  q1=(float)load_data.Q_quat1;  q2=(float)load_data.Q_quat2; q3=(float)load_data.Q_quat3;
  q0_m=(float)load_data.Q_quat0;  q1_m=(float)load_data.Q_quat1;  q2_m=(float)load_data.Q_quat2;  q3_m=(float)load_data.Q_quat3;
  beta=load_data.KQ_bias0;
  if(beta==0)
	  beta=0.1;
  twoKp=load_data.KQ_bias1; 
    if(twoKp==0)
	  twoKp=1;
  twoKi=load_data.KQ_bias2;
      if(twoKi==0)
	  twoKi=1;

  printf("open serial port send dtata \n");
  fd_serial_send=open_serial_port_com(serial_port_send,(int)load_data.serial_port_badu);
  if( fd_serial_send<0)
	{
	printf("error to open send serial port\n");
	}
  printf("open serial gadget port send \n");
  fd_serial_gadget=open_serial_port_com(GADGETPORT, 115200);
  if( fd_serial_send<0)
	{
	printf("error to open send serial gadget port\n");
	}


  /*fine inizializzazione kalman*/
  //structure data
  datasensincomp datasensor,datasensor1,datasensor2;
  /*opendevice esterni*/
  /*apertura i2c*/
  if((int)load_data.imu_type==1)
  {
		fd_iduec=openi2c();
  }
  if((int)load_data.enable_GPS>0)
  {
	fd_gps= open_gps(gps_port,load_data.badu_gps);
  }
 //create soket udp stream
   if ((sk = create_udp_server (ip_local_addrs, (int)load_data.internal_port)) == 0)
   {
      fprintf (stderr, "cannot open server socket\n");
      //exit (EXIT_FAILURE);
   }
   set_d7_initialize(fd_iduec);

  /*collego il segnale di terminazione*/
  signal(SIGTERM,sighandlersigterm);

 /*main loop*/
 while (1) 
  {
	
	/*****************time************************/
    	//elapTicks = (BeginTimer()-elapTicks_init)/time_divisor_clk;
	//if(elapTicks<=elapTicks_prec)
	//	{
	//	elapTicks=elapTicks_prec+fake_deltat;
	//	}
	//deltatime=elapTicks-elapTicks_prec;
	//elapTicks_prec=elapTicks;
	deltatime=fake_deltat;

	if((int)load_data.debug_mode==1)	{printf("time %f deltatime %f \n ",elapTicks,deltatime);}

	/*****************************GPS enable*********************************/
	if((int)load_data.enable_GPS==1)
	{
		cnt_gps ++;
		if(cnt_gps>GPSTIMEOUTCOUNT+100)
		{
			cnt_gps=0;
			send.gps=1;
			if(getUserPosition(fd_gps)<0)//salva i dati in varibile globale gpsData
			{printf("parser nemea string problem");}
			gpsData1=read_nav_data();
		}		
	}
	if((int)load_data.enable_GPS==0)
	{
		//fake gps data if not avable		
		gpsData1.lat=LATITUDINE_FAKE ;
		gpsData1.lon=LONGITUDINE_FAKE;
		gpsData1.heigth=ALTITUDINE_FAKE;
		gpsData1.speed=0.0;
		gpsData1.gpsFixQuality=0;
		send.gps=0;
	}
	if(gpsData1.lat==0||gpsData1.lon==0)
	{
		gpsData1.lat=LATITUDINE_FAKE ;
		gpsData1.lon=LONGITUDINE_FAKE;
		gpsData1.heigth=ALTITUDINE_FAKE;
		gpsData1.speed=0.0;
		gpsData1.gpsFixQuality=0;
		send.gps=0;
	}
	send.latitude=gpsData1.lat;
	send.longitude=gpsData1.lon;
	send.vel_gps=gpsData1.speed;
	send.altitudine=gpsData1.heigth;
	send.gps=gpsData1.gpsFixQuality;
	if((int)load_data.debug_mode==1)
	{
		printf("lat %f lon %f vel %f  altitude %f \n",send.latitude,send.longitude,send.vel_gps,send.altitudine);
	}
	//update altitude
	cnt_baro ++;
	if(cnt_baro>TMP_PRESS_TIMEOUTCOUNT)
	{
		cnt_baro=0;
		tmp_altitude=readaltitude_press(fd_iduec ,statusbaro);

		if((int)load_data.debug_mode==1)
		{
			printf("altitude %f \n",altitude);
		}
		statusbaro ++;
		if(statusbaro>3)
		{
			statusbaro=0;
		}
	}	
	//update altitude if data is true
	if(tmp_altitude>-500)
	{
		altitude=tmp_altitude;
	}	
	send.altitudine=altitude;
	/******************IMU D7*************************/
	if((int)load_data.imu_type==1)
	{

		if((int)load_data.kalman_enable>0)
		{
			datasensor = myupdatesensor_uno(fd_iduec, 1);
			
			datasensor=correct_whith_calib_uno(datasensor,load_data);
			if((int)load_data.kalman_enable==1)
			{
				MadgwickAHRSupdate(-datasensor.gyro_Y[0]/(DegToRadIMU), -datasensor.gyro_X[0]/(DegToRadIMU), -datasensor.gyro_Z[0]/(DegToRadIMU), datasensor.acc_Y[0], datasensor.acc_X[0], datasensor.acc_Z[0], -0*datasensor.mag_Z[0],-0*datasensor.mag_X[0],- 0*datasensor.mag_Y[0]);
				q0_t=q0;
				q1_t=q1;
				q2_t=q2;
				q3_t=q3;
			}
			if((int)load_data.kalman_enable==2)
			{
				MahonyAHRSupdate(-datasensor.gyro_Y[0]/(DegToRadIMU), -datasensor.gyro_X[0]/(DegToRadIMU), -datasensor.gyro_Z[0]/(DegToRadIMU), datasensor.acc_Y[0], datasensor.acc_X[0], datasensor.acc_Z[0], -datasensor.mag_Z[0],-datasensor.mag_X[0],- datasensor.mag_Y[0]);
				q0_t=q0_m;
				q1_t=q1_m;
				q2_t=q2_m;
				q3_t=q3_m;
			}

			datasensor1 = myupdatesensor_uno(fd_iduec, 0);
			datasensor1=correct_whith_calib_uno(datasensor1,load_data);
			if((int)load_data.kalman_enable==1)
			{
				MadgwickAHRSupdate(-datasensor1.gyro_Y[0]/(DegToRadIMU), -datasensor1.gyro_X[0]/(DegToRadIMU), -datasensor1.gyro_Z[0]/(DegToRadIMU), datasensor1.acc_Y[0], datasensor1.acc_X[0], datasensor1.acc_Z[0], -0*datasensor.mag_Z[0],-0*datasensor.mag_X[0],- 0*datasensor.mag_Y[0]);
				q0_t=q0_t+q0;
				q1_t=q1_t+q1;
				q2_t=q2_t+q2;
				q3_t=q3_t+q3;
			}
			if((int)load_data.kalman_enable==2)
			{
				MahonyAHRSupdate(-datasensor1.gyro_Y[0]/(DegToRadIMU), -datasensor1.gyro_X[0]/(DegToRadIMU), -datasensor1.gyro_Z[0]/(DegToRadIMU), datasensor1.acc_Y[0], datasensor1.acc_X[0], datasensor1.acc_Z[0], -datasensor.mag_Z[0],-datasensor.mag_X[0],- datasensor.mag_Y[0]);
				q0_t=q0_t+q0_m;
				q1_t=q1_t+q1_m;
				q2_t=q2_t+q2_m;
				q3_t=q3_t+q3_m;
			}
			datasensor2 = myupdatesensor_uno(fd_iduec, 1);
			datasensor2=correct_whith_calib_uno(datasensor2,load_data);
			if((int)load_data.kalman_enable==1)
			{
				MadgwickAHRSupdate(-datasensor2.gyro_Y[0]/(DegToRadIMU), -datasensor2.gyro_X[0]/(DegToRadIMU), -datasensor2.gyro_Z[0]/(DegToRadIMU), datasensor2.acc_Y[0], datasensor2.acc_X[0], datasensor2.acc_Z[0], -0*datasensor2.mag_Z[0],-0*datasensor2.mag_X[0],- 0*datasensor2.mag_Y[0]);
				q0_t=q0_t+q0;
				q1_t=q1_t+q1;
				q2_t=q2_t+q2;
				q3_t=q3_t+q3;
			}
			if((int)load_data.kalman_enable==2)
			{
				MahonyAHRSupdate(-datasensor2.gyro_Y[0]/(DegToRadIMU), -datasensor2.gyro_X[0]/(DegToRadIMU), -datasensor2.gyro_Z[0]/(DegToRadIMU), datasensor2.acc_Y[0], datasensor2.acc_X[0], datasensor2.acc_Z[0], -datasensor2.mag_Z[0],-datasensor2.mag_X[0],- datasensor2.mag_Y[0]);
				q0_t=q0_t+q0_m;
				q1_t=q1_t+q1_m;
				q2_t=q2_t+q2_m;
				q3_t=q3_t+q3_m;
			}
			//media
			q0_t=q0_t/3;
			q1_t=q1_t/3;
			q2_t=q2_t/3;
			q3_t=q3_t/3;
			/*input kalman*/
			sqw = q0_t*q0_t;
			sqx = q1_t*q1_t;
			sqy = q2_t*q2_t;
			sqz = q3_t*q3_t;
			unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
			test = q1_t*q2_t + q3_t*q0_t;
			if (test > 0.499*unit) { // singularity at north pole
				send.yaw= 57.324*(2 * atan2(q1,q0));
				send.pitch= 57.324*(3,14159265358979323846/2);
				send.roll= 0;
		
			}
			else if (test < -0.499*unit) { // singularity at south pole
				send.yaw= 57.324*(-2 * atan2(q1,q0));
				send.pitch= 57.324*(-3,14159265358979323846/2);
				send.roll= 0;
		
			}
			else
			{
				send.roll =-57.324*atan2(2*q2_t*q0_t-2*q1_t*q3_t , sqx - sqy - sqz + sqw);
				send.yaw = 57.324*asin(2*test/unit);
				send.pitch = 57.324*atan2(2*q1_t*q0_t-2*q2_t*q3_t , -sqx + sqy - sqz + sqw);
			}


			if((int)load_data.debug_mode==1)
			{
			  	printf("kalman roll: %f kalman pitch: %f kalman yaw: %f \n",send.roll,send.pitch,send.yaw );
			}
			send.gyro_X=(double)((datasensor.gyro_X[0]/(DegToRadIMU)+datasensor1.gyro_X[0]/(DegToRadIMU)+datasensor2.gyro_X[0]/(DegToRadIMU))/3);
			send.gyro_Y=(double)((datasensor.gyro_Y[0]/(DegToRadIMU)+datasensor1.gyro_Y[0]/(DegToRadIMU)+datasensor2.gyro_Y[0]/(DegToRadIMU))/3);
			send.gyro_Z=(double)((datasensor.gyro_Z[0]/(DegToRadIMU)+datasensor1.gyro_Z[0]/(DegToRadIMU)+datasensor2.gyro_Z[0]/(DegToRadIMU))/3);
			send.q1=(double)q0_t;
			send.q2=(double)q1_t;
			send.q3=(double)q2_t;
			send.q4=(double)q3_t;
			send.q1_dot=(double)0;
			send.q2_dot=(double)0;
			send.q3_dot=(double)0;
			send.q4_dot=(double)0;
		}

		send.Acc_X=(double)((datasensor.acc_X[0]+datasensor1.acc_X[0]+datasensor2.acc_X[0])/3);
		send.Acc_Y=(double)((datasensor.acc_Y[0]+datasensor1.acc_Y[0]+datasensor2.acc_Y[0])/3);
		send.Acc_Z=(double)((datasensor.acc_Z[0]+datasensor1.acc_Z[0]+datasensor2.acc_Z[0])/3);
		send.mag_X=(double)((datasensor.mag_X[0]+datasensor2.mag_X[0])/2);
		send.mag_Y=(double)((datasensor.mag_Y[0]+datasensor2.mag_Y[0])/2);
		send.mag_Z=(double)((datasensor.mag_Z[0]+datasensor2.mag_Z[0])/2);

		update_tmp_cnt ++;
		if(update_tmp_cnt>TMP_PRESS_TIMEOUTCOUNT)
		{	
			update_tmp_cnt=0;
			send.temperature=readtemp( fd_iduec);
			send.static_pressure=1;
		}
		send.tic=elapTicks;
		send.id_dispositivo=load_data.ID;
		send.number_of_packet ++;
		send.end='*';
		send.init='#';
		send.init1='s';
		send.init2='n';
		send.init3='y';
		send.vel_X=0;
		send.vel_Y=0;
		send.vel_Z=0;
		send.crc=crccal(send);
		usleep(100);

	}
	if(sk>=0)
	{
		udp_send (sk, send, ip_dest_addrs, (int)load_data.destination_port);
		udp_send (sk, send,"127.0.0.1", INTERNALUSEPORT);
	}
	if(fd_serial_send>=0){serial_send (fd_serial_send,send);}
	if(fd_serial_gadget>=0){serial_send (fd_serial_gadget,send);}
  }//end main loop

  /*device close*/
 
  close_udp_socket (sk);

 

  if((int)load_data.imu_type==1)
  {
  	closei2c(fd_iduec);
  }
  if((int)load_data.enable_GPS !=0)
  {
	close_gps(fd_gps);
  }
  if(fd_serial_send>=0)
  {
	close (fd_serial_send);
  }
  if(fd_serial_gadget>=0)
  {
 	close (fd_serial_gadget);
  }

  return 0;
}

/*signal*/
void sighandlersigterm(int sig)
{

	close_udp_socket (sk);
	

	if((int)load_data.imu_type==1)
	{
		closei2c(fd_iduec);
	}
	if((int)load_data.enable_GPS !=0)
	{
		close_gps(fd_gps);
	}
	if(fd_serial_send>=0)
	{
		close (fd_serial_send);
	}
	if(fd_serial_gadget>=0)
	{
		close (fd_serial_gadget);
	}
	exit(1);
}

/*timer*/
clock_t BeginTimer()
{
    //timer declaration
    clock_t Begin; //initialize Begin
    Begin = clock() ; //start the timer
    return Begin;
}

int crccal(data_AHRS_Struct a)
{
 /*int ret=(int)(a.latitude +  a.longitude +a.vel_gps + a.altitudine +a.roll+a.pitch +	a.yaw + a.q1 + a.q2 + a.q3 +
	a.q4 + 	a.Acc_X + a.Acc_Y + a.Acc_Z + a.gyro_X + a.gyro_Y + a.gyro_Z +	a.mag_X + a.mag_Y + a.mag_Z +
	a.static_pressure + a.tic + a.temperature + a.gps + a.satellite_number + a.number_of_packet +  a.id_dispositivo);*/
 int ret=1;

return ret;
}



datasensincomp correct_whith_calib_uno(datasensincomp data_row,load_data_struct calibr_data)
{
	datasensincomp calibrated_data;

	calibrated_data.gyro_Y[0] = data_row.gyro_Y[0]-calibr_data.GYRO20;
	calibrated_data.gyro_X[0] =data_row.gyro_X[0]-calibr_data.GYRO10;
	calibrated_data.gyro_Z[0]= data_row.gyro_Z[0]-calibr_data.GYRO30;
	calibrated_data.acc_X[0]=calibr_data.ACC11*data_row.acc_X[0]+ calibr_data.ACC12*data_row.acc_Y[0] +calibr_data.ACC13*data_row.acc_Z[0]+calibr_data.ACC10;
	calibrated_data.acc_Y[0]=calibr_data.ACC21*data_row.acc_X[0]+ calibr_data.ACC22*data_row.acc_Y[0] +calibr_data.ACC23*data_row.acc_Z[0]+calibr_data.ACC20;
	calibrated_data.acc_Z[0]=calibr_data.ACC31*data_row.acc_X[0]+ calibr_data.ACC32*data_row.acc_Y[0] +calibr_data.ACC33*data_row.acc_Z[0]+calibr_data.ACC30;

	calibrated_data.mag_X[0]=calibr_data.MAG11*data_row.mag_X[0]+ calibr_data.MAG12*data_row.mag_Y[0] +calibr_data.MAG13*data_row.mag_Z[0]+calibr_data.MAG10;
	calibrated_data.mag_Y[0]=calibr_data.MAG21*data_row.mag_X[0]+ calibr_data.MAG22*data_row.mag_Y[0] +calibr_data.MAG23*data_row.mag_Z[0]+calibr_data.MAG20;
	calibrated_data.mag_Z[0]=calibr_data.MAG31*data_row.mag_X[0]+ calibr_data.MAG32*data_row.mag_Y[0] +calibr_data.MAG33*data_row.mag_Z[0]+calibr_data.MAG30;

	return calibrated_data; 
}

