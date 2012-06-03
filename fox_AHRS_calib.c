/*
 * File: fox_AHRS_calib.c
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
 * C/C++ source on      	        : 06 01 2012
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

#include "IMU/iduec.h"
#include "IMU/daisysette.h"

#include "IMU/global_sensor.h"
#include "cal_acc_mag/calibrazione_acc_mag0.h"
#include "cal_acc_mag/calibrazione_acc_mag0_private.h"              	/* Model's header file */
#include "cal_acc_mag/rtwtypes.h"                  			/* MathWorks types */
#include "globaldata.h"
#include "config/loadconf.h"
#include "comunication_descriptor_data.h"
#include "udpsocketlib.h"

/*gps*/
#include "gps/gps.h"


load_data_struct load_data;

     
int main(int argc, char *argv[])
{
  /*************load global data*****************/
  load_data=load_configuration();
  /*end load data*/
  int div;
  int dimensione=0;
  int num_cal_sample=60;
  int num_cal_sample_mag=168;
  NavInputs_t gpsData1;
  strcpy(ip_dest_addrs, load_data.ip_dest_addrs);
  printf("inizialize control ip dest %s \n",ip_dest_addrs);
  strcpy(ip_local_addrs, load_data.ip_local_addrs);
  printf("inizialize control ip local %s \n",ip_local_addrs);
  strcpy(imu_port, load_data.imu_port);
  printf("inizialize control imu port %s \n",imu_port);
  strcpy(gps_port, load_data.gps_port);
  printf("inizialize control gps port %s \n",gps_port);

  /*inizialize model*/
  calibrazione_acc_mag0_initialize();

  //create soket udp stream
   if ((sk = create_udp_server ("127.0.0.1", INTERNALUSEPORT)) == 0)
   {
      fprintf (stderr, "cannot open server socket\n");
      //exit (EXIT_FAILURE);
   }
  data_AHRS_Struct recive ;//data to send
  memset(&recive,0,sizeof(recive));
  //data_AHRS_Struct udp_receive (int sk, int *dimr)

  float w1[4], w2[4], w3[4], w4[4] , w5[4], w6[4];
  memset(&w1,0,sizeof(w1));
  memset(&w2,0,sizeof(w2));
  memset(&w3,0,sizeof(w3));
  memset(&w4,0,sizeof(w5));
  memset(&w6,0,sizeof(w6));
  float gyrobias[3];
  memset(&gyrobias,0,sizeof(gyrobias));
  int i;
/*****************************GPS enable*********************************/
	while(dimensione!=sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	dimensione=0;
	gpsData1.lat=recive.latitude;
	gpsData1.lon=recive.longitude;
	if((gpsData1.lat==0)||(gpsData1.lon==0))
	{
		//fake gps		
		gpsData1.lat=LATITUDINE_FAKE ;
		gpsData1.lon=LONGITUDINE_FAKE;
		gpsData1.heigth=ALTITUDINE_FAKE;
		gpsData1.speed=0.0;
		gpsData1.gpsFixQuality=0;
		
	}
	//acquisico gyro	
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		gyrobias[0]=gyrobias[0]+ recive.gyro_X/DegToRadIMU;
		gyrobias[1]=gyrobias[1]+ recive.gyro_Y/DegToRadIMU;
		gyrobias[2]=gyrobias[2]+ recive.gyro_Z/DegToRadIMU;
		div=i;
		usleep(10000);
	}
	div=div*1;
	gyrobias[0]=gyrobias[0]/div;
	gyrobias[1]=gyrobias[1]/div;
	gyrobias[2]=gyrobias[2]/div;

	//acquisico accelerometri nella posizione top
	printf("Place the sensor up\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w1[0]=w1[0] + recive.Acc_X;
		w1[1]=w1[1] + recive.Acc_Y;
		w1[2]=w1[2] + recive.Acc_Z;
		w1[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w1[0]=w1[0]/div;
	calibrazione_acc_mag0_U.w1[1]=w1[1]/div;
	calibrazione_acc_mag0_U.w1[2]=w1[2]/div;
	calibrazione_acc_mag0_U.w1[3]=1;
	printf("w1 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w1[0],calibrazione_acc_mag0_U.w1[1],calibrazione_acc_mag0_U.w1[2],calibrazione_acc_mag0_U.w1[3]);
	//acquisico accelerometri nella posizione basso
	printf("Place the sensor down\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w2[0]=w2[0] + recive.Acc_X;
		w2[1]=w2[1] + recive.Acc_Y;
		w2[2]=w2[2] + recive.Acc_Z;
		w2[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w2[0]=w2[0]/div;
	calibrazione_acc_mag0_U.w2[1]=w2[1]/div;
	calibrazione_acc_mag0_U.w2[2]=w2[2]/div;
	calibrazione_acc_mag0_U.w2[3]=1;
	printf("w2 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w2[0],calibrazione_acc_mag0_U.w2[1],calibrazione_acc_mag0_U.w2[2],calibrazione_acc_mag0_U.w2[3]);	
	//acquisico accelerometri nella posizione lato-destro
	printf("Place the sensor right\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w3[0]=w3[0] + recive.Acc_X;
		w3[1]=w3[1] + recive.Acc_Y;
		w3[2]=w3[2] + recive.Acc_Z;
		w3[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w3[0]=w3[0]/div;
	calibrazione_acc_mag0_U.w3[1]=w3[1]/div;
	calibrazione_acc_mag0_U.w3[2]=w3[2]/div;
	calibrazione_acc_mag0_U.w3[3]=1;
	printf("w3 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w3[0],calibrazione_acc_mag0_U.w3[1],calibrazione_acc_mag0_U.w3[2],calibrazione_acc_mag0_U.w3[3]);	
	//acquisico accelerometri nella posizione lato-sinitro
	printf("Place the sensor left\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w4[0]=w4[0] + recive.Acc_X;
		w4[1]=w4[1] + recive.Acc_Y;
		w4[2]=w4[2] + recive.Acc_Z;
		w4[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w4[0]=w4[0]/div;
	calibrazione_acc_mag0_U.w4[1]=w4[1]/div;
	calibrazione_acc_mag0_U.w4[2]=w4[2]/div;
	calibrazione_acc_mag0_U.w4[3]=1;
	printf("w4 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w4[0],calibrazione_acc_mag0_U.w4[1],calibrazione_acc_mag0_U.w4[2],calibrazione_acc_mag0_U.w4[3]);
	//acquisico accelerometri nella posizione lato-avanti
	printf("Place the sensor forward\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w5[0]=w5[0] + recive.Acc_X;
		w5[1]=w5[1] + recive.Acc_Y;
		w5[2]=w5[2] + recive.Acc_Z;
		w5[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w5[0]=w5[0]/div;
	calibrazione_acc_mag0_U.w5[1]=w5[1]/div;
	calibrazione_acc_mag0_U.w5[2]=w5[2]/div;
	calibrazione_acc_mag0_U.w5[3]=1;
	printf("w5 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w5[0],calibrazione_acc_mag0_U.w5[1],calibrazione_acc_mag0_U.w5[2],calibrazione_acc_mag0_U.w5[3]);
	//acquisico accelerometri nella posizione lato-indietro
	printf("Place the sensor back\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		w6[0]=w6[0] + recive.Acc_X;
		w6[1]=w6[1] + recive.Acc_Y;
		w6[2]=w6[2] + recive.Acc_Z;
		w6[3]=1;
		div=i;
		usleep(10000);
	}
	div=div*1;
	calibrazione_acc_mag0_U.w6[0]=w6[0]/div;
	calibrazione_acc_mag0_U.w6[1]=w6[1]/div;
	calibrazione_acc_mag0_U.w6[2]=w6[2]/div;
	calibrazione_acc_mag0_U.w6[3]=1;
	printf("w6 %f  , %f , %f , %f \n",calibrazione_acc_mag0_U.w6[0],calibrazione_acc_mag0_U.w6[1],calibrazione_acc_mag0_U.w6[2],calibrazione_acc_mag0_U.w6[3]);
	//acquisico posizione magnetometri mentre faccio ruotare su un piano
	printf("Place the sensor up and turn on itself\n");
	sleep(10);
	dimensione=sizeof(recive);
	while(dimensione==sizeof(recive))
	{
  		recive= udp_receive (sk, &dimensione);
	}
	for( i=0; i<num_cal_sample_mag;i++)
	{
		while(dimensione!=sizeof(recive))
		{
			recive= udp_receive (sk, &dimensione);
		}
		dimensione=0;
		calibrazione_acc_mag0_U.mag_x[i]=recive.mag_X;
		calibrazione_acc_mag0_U.mag_y[i]= recive.mag_Y;
		calibrazione_acc_mag0_U.mag_z[i]= recive.mag_Z;
		usleep(10000);
	}

	 //passo i dati

	//calcolo le matrici di correzione
  calibrazione_acc_mag0_step();
  printf("calibragione accelerometro:\n");
  printf(" %f  %f  %f \n %f  %f  %f \n %f  %f  %f \n",
	calibrazione_acc_mag0_Y.G_acc[0],calibrazione_acc_mag0_Y.G_acc[1],calibrazione_acc_mag0_Y.G_acc[2],
	calibrazione_acc_mag0_Y.G_acc[3],calibrazione_acc_mag0_Y.G_acc[4],calibrazione_acc_mag0_Y.G_acc[5],
	calibrazione_acc_mag0_Y.G_acc[6],calibrazione_acc_mag0_Y.G_acc[7],calibrazione_acc_mag0_Y.G_acc[8]);
	printf("bias acc %f  %f  %f \n",calibrazione_acc_mag0_Y.Bias_acc[0],calibrazione_acc_mag0_Y.Bias_acc[1],calibrazione_acc_mag0_Y.Bias_acc[2]);

  printf("calibragione magnetometr:\n");

  printf(" %f  %f  %f \n %f  %f  %f \n %f  %f  %f \n",
	(calibrazione_acc_mag0_Y.G_mag[0]+1),calibrazione_acc_mag0_Y.G_mag[1],calibrazione_acc_mag0_Y.G_mag[2],
	calibrazione_acc_mag0_Y.G_mag[3],(calibrazione_acc_mag0_Y.G_mag[4]+1),calibrazione_acc_mag0_Y.G_mag[5],
	calibrazione_acc_mag0_Y.G_mag[6],calibrazione_acc_mag0_Y.G_mag[7],(calibrazione_acc_mag0_Y.G_mag[8]+1));
  printf("bias mag %f  %f  %f \n",(calibrazione_acc_mag0_Y.Bias_mag[0]/100),(calibrazione_acc_mag0_Y.Bias_mag[1]/100),(calibrazione_acc_mag0_Y.Bias_mag[2]/100));

  printf("bias gyro %f  %f  %f \n",gyrobias[0],gyrobias[1],gyrobias[2]);

  //salvo matrici
  load_data.ACC11=calibrazione_acc_mag0_Y.G_acc[0];
  load_data.ACC12=calibrazione_acc_mag0_Y.G_acc[1];
  load_data.ACC13=calibrazione_acc_mag0_Y.G_acc[2];
  load_data.ACC21=calibrazione_acc_mag0_Y.G_acc[3];
  load_data.ACC22=calibrazione_acc_mag0_Y.G_acc[4];
  load_data.ACC23=calibrazione_acc_mag0_Y.G_acc[5];
  load_data.ACC31=calibrazione_acc_mag0_Y.G_acc[6];
  load_data.ACC32=calibrazione_acc_mag0_Y.G_acc[7];
  load_data.ACC33=calibrazione_acc_mag0_Y.G_acc[8];
  load_data.ACC10=calibrazione_acc_mag0_Y.Bias_acc[0];
  load_data.ACC20=calibrazione_acc_mag0_Y.Bias_acc[1];
  load_data.ACC30=calibrazione_acc_mag0_Y.Bias_acc[2];

  load_data.MAG11=calibrazione_acc_mag0_Y.G_mag[0]+1;
  load_data.MAG12=calibrazione_acc_mag0_Y.G_mag[1];
  load_data.MAG13=calibrazione_acc_mag0_Y.G_mag[2];
  load_data.MAG21=calibrazione_acc_mag0_Y.G_mag[3];
  load_data.MAG22=calibrazione_acc_mag0_Y.G_mag[4]+1;
  load_data.MAG23=calibrazione_acc_mag0_Y.G_mag[5];
  load_data.MAG31=calibrazione_acc_mag0_Y.G_mag[6];
  load_data.MAG32=calibrazione_acc_mag0_Y.G_mag[7];
  load_data.MAG33=calibrazione_acc_mag0_Y.G_mag[8]+1;
  load_data.MAG10=calibrazione_acc_mag0_Y.Bias_mag[0]/100;
  load_data.MAG20=calibrazione_acc_mag0_Y.Bias_mag[1]/100;
  load_data.MAG30=calibrazione_acc_mag0_Y.Bias_mag[2]/100;

  load_data.GYRO10=gyrobias[0];
  load_data.GYRO20=gyrobias[1];
  load_data.GYRO30=gyrobias[2];
  printf("save\n");
  save_calibration( load_data);

  //termino modello
  calibrazione_acc_mag0_terminate();
  close_udp_socket (sk);
  printf("end calibration\n");
  return 0;
 
}
