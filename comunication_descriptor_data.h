/**
 * 
 * 
 *  This program is free software: you can redistribute it and/or modify
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
 * @date 06/01/2012
 *
 * Ground Station
 * http://www.federicololli.com
 * 
 *  
 */

#ifndef COMUNICATION_DESCRIPTOR_DATA_H_
#define COMUNICATION_DESCRIPTOR_DATA_H_

#define DESTADDRESS 	"192.168.0.2"
#define LOCALADDRESS 	"192.168.0.3"
#define DESTPORT 		5051
#define INPORT			5050
#define DEF_SERIAL_PORT 	"/dev/ttyUSB0"
#define DEF_SERIAL_BADU	115200




/**********data structure UDP send recive **************/
struct _data_AHRS_Struct
{
	char init;
	char init1;
	char init2;
	char init3;
	//deg
	float roll;
	float pitch;
	float yaw;

	//g
	float Acc_X;
	float Acc_Y;
	float Acc_Z;
	//deg/sec
	float gyro_X;
	float gyro_Y;
	float gyro_Z;
	//mag
	float mag_X;
	float mag_Y;
	float mag_Z;
	//vel
	float vel_X;
	float vel_Y;
	float vel_Z;
	//quaternioni
	float q1;
	float q2;
	float q3;
	float q4;
	//quat derivati
	float q1_dot;
	float q2_dot;
	float q3_dot;
	float q4_dot;
	/******data recive IMU & status********/
	//status imu  & gps
	float latitude;
	float longitude;
	float vel_gps;//m/s
	float altitudine;//m

	/*dati aria*/
	float static_pressure;
	float tic;
	int temperature;
	//general status
	int gps;		//gps mode correct acquiring: 0 fail, 1 ok
 	int satellite_number;
	int number_of_packet;		//number to packet optional
	int id_dispositivo;
	int crc;
	char end;

};

typedef _data_AHRS_Struct  data_AHRS_Struct;



#endif
