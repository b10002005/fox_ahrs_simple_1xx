/*
 * File: loasconf.h
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

#ifndef  _LOADCONF_H_
#define _LOADCONF_H_

#include <stdio.h>
#include <stdlib.h>
#include <libconfig.h>

 struct _load_data_struct{
	const char *ip_dest_addrs;
	const char *ip_local_addrs;
	long int destination_port;
	long int internal_port;
	long int imu_type;
	const char *imu_port;
	long int kalman_enable;
	long int update_rel_time;

	double GYRO10;
	double GYRO20;
	double GYRO30;

	double ACC10;
	double ACC20;
	double ACC30;

	double ACC11;
	double ACC12;
	double ACC13;
	double ACC21;
	double ACC22;
	double ACC23;
	double ACC31;
	double ACC32;
	double ACC33;

	double MAG10;
	double MAG20;
	double MAG30;

	double MAG11;
	double MAG12;
	double MAG13;
	double MAG21;
	double MAG22;
	double MAG23;
	double MAG31;
	double MAG32;
	double MAG33;

	double Q_quat0;
	double Q_quat1;
	double Q_quat2;
	double Q_quat3;
	double tau_comp_filter;
	double KQ_bias0;
	double KQ_bias1;
	double KQ_bias2;

	long int enable_GPS;
	long int badu_gps;
	const char *gps_port;
	const char *serial_port;
	long int serial_port_badu;
	double	q_dot_max_error;
	double dev_std_accell;
	double dev_std_mag;
	double kalman_delta_time;
	long int debug_mode;
	long int ID;

	};
typedef _load_data_struct load_data_struct;
load_data_struct load_configuration();
int save_calibration( load_data_struct data_in);


#endif

