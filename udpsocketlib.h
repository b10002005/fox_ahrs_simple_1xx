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
 * Fox_ahrs
 * http://www.federicololli.com
 * 
 *  
 */

#ifndef __UDPSOKETLIB1_H__
#define __UDPSOKETLIB1_H__
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <strings.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "comunication_descriptor_data.h"


int create_udp_client (void);
int create_udp_server (char *ip_address, int port);
int close_udp_socket (int sk);
int udp_send (int sk, data_AHRS_Struct buffer, char *ip_address, int port);
data_AHRS_Struct udp_receive (int sk, int *dimr);

#endif

