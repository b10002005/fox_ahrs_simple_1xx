/*
 * File: loasconf.c
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
#include"loadconf.h"
#include <libconfig.h>
 load_data_struct load_configuration()
{
  config_t cfg;

  load_data_struct loaddata;

  config_init(&cfg);
  /* Read the file. If there is an error, report it and exit. */
  if(!config_read_file(&cfg, "config.cfg"))
  {
    printf("error configuration file \n");
    config_destroy(&cfg);
    exit(1);
  }
  /*read string*/
  if(config_lookup_string(&cfg, "serial_port", &loaddata.serial_port))
    printf("serial send port: %s\n", loaddata.serial_port);

  if(config_lookup_int(&cfg, "serial_port_badu", &loaddata.serial_port_badu))
    printf("serial port badu: %d\n",(int) loaddata.serial_port_badu);
  if((loaddata.serial_port_badu!=115200)||(loaddata.serial_port_badu!=38400)||(loaddata.serial_port_badu!=19200)||
		(loaddata.serial_port_badu!=9600)||(loaddata.serial_port_badu!=4800))
  {
	loaddata.serial_port_badu=115200;
  }

  if(config_lookup_int(&cfg, "badu_gps", &loaddata.badu_gps))
    printf("serial port badu: %d\n",(int) loaddata.badu_gps);
  if((loaddata.badu_gps!=115200)||(loaddata.badu_gps!=38400)||(loaddata.badu_gps!=19200)||
		(loaddata.badu_gps!=9600)||(loaddata.badu_gps!=4800))
  {
	loaddata.badu_gps=115200;
  }

  if(config_lookup_string(&cfg, "ip_dest_addrs", &loaddata.ip_dest_addrs))
    printf("ip dest name: %s\n", loaddata.ip_dest_addrs);

  if(config_lookup_string(&cfg, "ip_local_addrs", &loaddata.ip_local_addrs))
    printf("ip local name: %s\n", loaddata.ip_local_addrs);

  if(config_lookup_int(&cfg, "destination_port", &loaddata.destination_port))
    printf("destination_port: %d\n",(int) loaddata.destination_port);

  if(config_lookup_int(&cfg, "internal_port", &loaddata.internal_port))
    printf("internal_port: %d\n",(int)  loaddata.internal_port);

  if(config_lookup_int(&cfg, "imu_type", &loaddata.imu_type))
    printf("imu_type: %d\n", (int) loaddata.imu_type);

  if(config_lookup_int(&cfg, "kalman_enable", &loaddata.kalman_enable))
    printf("kalman_enable: %d\n",(int)  loaddata.kalman_enable);

  if(config_lookup_string(&cfg, "imu_port", &loaddata.imu_port))
    printf("imu port: %s\n", loaddata.imu_port);

  if(config_lookup_int(&cfg, "update_rel_time", &loaddata.update_rel_time))
    printf("update_rel_time: %d\n", (int) loaddata.update_rel_time);


  if(config_lookup_float(&cfg, "GYRO10", &loaddata.GYRO10))
    printf("GYRO10: %f\n", loaddata.GYRO10);
  if(config_lookup_float(&cfg, "GYRO20", &loaddata.GYRO20))
    printf("GYRO20: %f\n", loaddata.GYRO20);
  if(config_lookup_float(&cfg, "GYRO30", &loaddata.GYRO30))
    printf("GYRO30: %f\n", loaddata.GYRO30);

  if(config_lookup_float(&cfg, "MAG10", &loaddata.MAG10))
    printf("MAG10: %f\n", loaddata.MAG10);
  if(config_lookup_float(&cfg, "MAG20", &loaddata.MAG20))
    printf("MAG20: %f\n", loaddata.MAG20);
  if(config_lookup_float(&cfg, "MAG30", &loaddata.MAG30))
    printf("MAG30: %f\n", loaddata.MAG30);

  if(config_lookup_float(&cfg, "MAG11", &loaddata.MAG11))
    printf("MAG11: %f\n", loaddata.MAG11);
  if(config_lookup_float(&cfg, "MAG12", &loaddata.MAG12))
    printf("MAG12: %f\n", loaddata.MAG12);
  if(config_lookup_float(&cfg, "MAG13", &loaddata.MAG13))
    printf("MAG13: %f\n", loaddata.MAG13);

  if(config_lookup_float(&cfg, "MAG21", &loaddata.MAG21))
    printf("MAG21: %f\n", loaddata.MAG21);
  if(config_lookup_float(&cfg, "MAG22", &loaddata.MAG22))
    printf("MAG22: %f\n", loaddata.MAG22);
  if(config_lookup_float(&cfg, "MAG23", &loaddata.MAG23))
    printf("MAG23: %f\n", loaddata.MAG23);

  if(config_lookup_float(&cfg, "MAG31", &loaddata.MAG31))
    printf("MAG31: %f\n", loaddata.MAG31);
  if(config_lookup_float(&cfg, "MAG32", &loaddata.MAG32))
    printf("MAG32: %f\n", loaddata.MAG32);
  if(config_lookup_float(&cfg, "MAG33", &loaddata.MAG33))
    printf("MAG33: %f\n", loaddata.MAG33);

  if(config_lookup_float(&cfg, "ACC10", &loaddata.ACC10))
    printf("ACC10: %f\n", loaddata.ACC10);
  if(config_lookup_float(&cfg, "ACC20", &loaddata.ACC20))
    printf("ACC20: %f\n", loaddata.ACC20);
  if(config_lookup_float(&cfg, "ACC30", &loaddata.ACC30))
    printf("ACC30: %f\n", loaddata.ACC30);

  if(config_lookup_float(&cfg, "ACC11", &loaddata.ACC11))
    printf("ACC11: %f\n", loaddata.ACC11);
  if(config_lookup_float(&cfg, "ACC12", &loaddata.ACC12))
    printf("ACC12: %f\n", loaddata.ACC12);
  if(config_lookup_float(&cfg, "ACC13", &loaddata.ACC13))
    printf("ACC13: %f\n", loaddata.ACC13);

  if(config_lookup_float(&cfg, "ACC21", &loaddata.ACC21))
    printf("ACC21: %f\n", loaddata.ACC21);
  if(config_lookup_float(&cfg, "ACC22", &loaddata.ACC22))
    printf("ACC22: %f\n", loaddata.ACC22);
  if(config_lookup_float(&cfg, "ACC23", &loaddata.ACC23))
    printf("ACC23: %f\n", loaddata.ACC23);

  if(config_lookup_float(&cfg, "ACC31", &loaddata.ACC31))
    printf("ACC31: %f\n", loaddata.ACC31);
  if(config_lookup_float(&cfg, "ACC32", &loaddata.ACC32))
    printf("ACC32: %f\n", loaddata.ACC32);
  if(config_lookup_float(&cfg, "ACC33", &loaddata.ACC33))
    printf("ACC33: %f\n", loaddata.ACC33);

  if(config_lookup_float(&cfg, "Q_quat0", &loaddata.Q_quat0))
    printf("Q_quat0: %f\n", loaddata.Q_quat0);
  if(config_lookup_float(&cfg, "Q_quat1", &loaddata.Q_quat1))
    printf("Q_quat1: %f\n", loaddata.Q_quat1);
  if(config_lookup_float(&cfg, "Q_quat2", &loaddata.Q_quat2))
    printf("Q_quat2: %f\n", loaddata.Q_quat2);
  if(config_lookup_float(&cfg, "Q_quat3", &loaddata.Q_quat3))
    printf("Q_quat3: %f\n", loaddata.Q_quat3);
  if(config_lookup_float(&cfg, "tau_comp_filter", &loaddata.tau_comp_filter))
    printf("tau_comp_filter: %f\n", loaddata.tau_comp_filter);

  if(config_lookup_float(&cfg, "KQ_bias0", &loaddata.KQ_bias0))
    printf("KQ_bias0: %f\n", loaddata.KQ_bias0);
  if(config_lookup_float(&cfg, "KQ_bias1", &loaddata.KQ_bias1))
    printf("KQ_bias1: %f\n", loaddata.KQ_bias1);
  if(config_lookup_float(&cfg, "KQ_bias2", &loaddata.KQ_bias2))
    printf("KQ_bias2: %f\n", loaddata.KQ_bias2);

  if(config_lookup_int(&cfg, "enable_GPS", &loaddata.enable_GPS))
    printf("enable_GPS: %d\n", (int) loaddata.enable_GPS);

  if(config_lookup_string(&cfg, "gps_port", &loaddata.gps_port))
    printf("gps port: %s\n", loaddata.gps_port);

  if(config_lookup_float(&cfg, "kalman_delta_time", &loaddata.kalman_delta_time))
    printf("kalman_delta_time: %f\n", loaddata.kalman_delta_time);

  if(config_lookup_float(&cfg, "q_dot_max_error", &loaddata.q_dot_max_error))
    printf("q_dot_max_error: %f\n", loaddata.q_dot_max_error);

  if(config_lookup_float(&cfg, "dev_std_accell", &loaddata.dev_std_accell))
    printf("dev_std_accell: %f\n", loaddata.dev_std_accell);

  if(config_lookup_float(&cfg, "dev_std_mag", &loaddata.dev_std_mag))
    printf("dev_std_mag: %f\n", loaddata.dev_std_mag);

  if(config_lookup_int(&cfg, "debug_mode", &loaddata.debug_mode))
    printf("debug_mode: %d\n", (int) loaddata.debug_mode);

  if(config_lookup_int(&cfg, "ID", &loaddata.ID))
    printf("ID: %d\n", (int) loaddata.ID);


  return loaddata;
}


 int save_calibration( load_data_struct data_in)
 {
   static const char *output_file = "config.cfg";
   config_t cfg;
   config_setting_t *root, *setting;

   config_init(&cfg);

   root = config_root_setting(&cfg);

   setting = config_setting_add(root, "serial_port", CONFIG_TYPE_STRING);
   config_setting_set_string(setting,  data_in.serial_port);
   setting = config_setting_add(root, "serial_port_badu", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.serial_port_badu);
   setting = config_setting_add(root, "ip_dest_addrs", CONFIG_TYPE_STRING);
   config_setting_set_string(setting,  data_in.ip_dest_addrs);
   setting = config_setting_add(root, "ip_local_addrs", CONFIG_TYPE_STRING);
   config_setting_set_string(setting,  data_in.ip_local_addrs);
   setting = config_setting_add(root, "destination_port", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.destination_port);
   setting = config_setting_add(root, "internal_port", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.internal_port);
   setting = config_setting_add(root, "imu_type", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.imu_type);
   setting = config_setting_add(root, "kalman_enable", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.kalman_enable);
   setting = config_setting_add(root, "imu_port", CONFIG_TYPE_STRING);
   config_setting_set_string(setting,  data_in.imu_port);
   setting = config_setting_add(root, "update_rel_time", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.update_rel_time);
   setting = config_setting_add(root, "enable_GPS", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.enable_GPS);
   setting = config_setting_add(root, "gps_port", CONFIG_TYPE_STRING);
   config_setting_set_string(setting,  data_in.gps_port);
   setting = config_setting_add(root, "badu_gps", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.badu_gps);


   setting = config_setting_add(root, "kalman_delta_time", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.kalman_delta_time);
   setting = config_setting_add(root, "q_dot_max_error", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.q_dot_max_error);
   setting = config_setting_add(root, "dev_std_accell", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.dev_std_accell);
   setting = config_setting_add(root, "dev_std_mag", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.dev_std_mag);
   setting = config_setting_add(root, "debug_mode", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.debug_mode);
   setting = config_setting_add(root, "ID", CONFIG_TYPE_INT);
   config_setting_set_int(setting,  data_in.ID);


   setting = config_setting_add(root, "Q_quat0", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.Q_quat0);
   setting = config_setting_add(root, "Q_quat1", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.Q_quat1);
   setting = config_setting_add(root, "Q_quat2", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.Q_quat2);
   setting = config_setting_add(root, "Q_quat3", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.Q_quat3);
   setting = config_setting_add(root, "tau_comp_filter", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.tau_comp_filter);
   setting = config_setting_add(root, "KQ_bias0", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.KQ_bias0);
   setting = config_setting_add(root, "KQ_bias1", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.KQ_bias1);
   setting = config_setting_add(root, "KQ_bias2", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.KQ_bias2);


   setting = config_setting_add(root, "GYRO10", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.GYRO10);
   setting = config_setting_add(root, "GYRO20", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.GYRO20);
   setting = config_setting_add(root, "GYRO30", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.GYRO30);

   setting = config_setting_add(root, "ACC10", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC10);
   setting = config_setting_add(root, "ACC20", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC20);
   setting = config_setting_add(root, "ACC30", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC30);
   setting = config_setting_add(root, "ACC11", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC11);
   setting = config_setting_add(root, "ACC12", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC12);
   setting = config_setting_add(root, "ACC13", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC13);
   setting = config_setting_add(root, "ACC21", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC21);
   setting = config_setting_add(root, "ACC22", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC22);
   setting = config_setting_add(root, "ACC23", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC23);
   setting = config_setting_add(root, "ACC31", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC31);
   setting = config_setting_add(root, "ACC32", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC32);
   setting = config_setting_add(root, "ACC33", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.ACC33);

   setting = config_setting_add(root, "MAG10", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG10);
   setting = config_setting_add(root, "MAG20", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG20);
   setting = config_setting_add(root, "MAG30", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG30);
   setting = config_setting_add(root, "MAG11", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG11);
   setting = config_setting_add(root, "MAG12", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG12);
   setting = config_setting_add(root, "MAG13", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG13);
   setting = config_setting_add(root, "MAG21", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG21);
   setting = config_setting_add(root, "MAG22", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG22);
   setting = config_setting_add(root, "MAG23", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG23);
   setting = config_setting_add(root, "MAG31", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG31);
   setting = config_setting_add(root, "MAG32", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG32);
   setting = config_setting_add(root, "MAG33", CONFIG_TYPE_FLOAT);
   config_setting_set_float(setting,  data_in.MAG33);

   /* Write out the updated configuration. */
   if(! config_write_file(&cfg, output_file))
   {
     fprintf(stderr, "Error while writing file.\n");
     config_destroy(&cfg);
     return(EXIT_FAILURE);
   }

   fprintf(stderr, "Updated configuration successfully written to: %s\n",
           output_file);
   config_destroy(&cfg);
     return(EXIT_SUCCESS);

 }

