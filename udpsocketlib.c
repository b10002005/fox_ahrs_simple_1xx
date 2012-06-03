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
 * fox_ahrs
 * http://www.federicololli.com
 * 
 *  
 */


#include "udpsocketlib.h"

static struct sockaddr_in reply_to_socket_address;
static int len_of_reply_to_socket_address;

void error_handler (char *message);


int create_udp_client (void)
  {

  int sk;
  struct hostent *host;
  host= (struct hostent *) gethostbyname((char *)"127.0.0.1");
   /* create a socket descriptor */
   if ((sk = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
   {
      perror("socket() [create_udp_client()]");
      return 0;
   }
   return sk;
}


int create_udp_server (char *ip_address, int port)
  {
   int sk;
   struct sockaddr_in srv;
   /* create a socket descriptor */
   if ((sk = socket (AF_INET, SOCK_DGRAM, 0)) < 0)
     {
       perror("socket() [create_udp_server()]");
        return 0;
     }
  int flags=O_NONBLOCK;
  fcntl(sk,F_SETFL,flags);

   /* fill the (used) fields of the socket address with
      the server information (local socket address) */
   bzero ((char *) &srv, sizeof (srv));
   srv.sin_family = AF_INET;
   srv.sin_addr.s_addr = inet_addr (ip_address);
   srv.sin_port = htons (port);

   /* add the local socket address to the socket descriptor */
   if (bind (sk, (struct sockaddr *) &srv, sizeof(srv)) < 0)
   {
      perror("bind() [create_udp_server()] ip addres error ");
      return 0;
   }

   return sk;
  }


int close_udp_socket (int sk)
  {
   if (close (sk) != 0)
   {
      perror("close() [close_udp_socket()]");
      return 0;
   }
   return 1;
}


int udp_send (int sk, data_AHRS_Struct buffer, char *ip_address, int port)
{
   struct sockaddr_in ska;
   int msg_len = sizeof(buffer);

   /* fill the (used) fields of the socket address with
      the server information (remote socket address) */
   bzero ((char *) &ska, sizeof (ska));
   ska.sin_family = AF_INET;
   ska.sin_addr.s_addr = inet_addr (ip_address);
   ska.sin_port = htons ((unsigned short) port);
   if (sendto (sk, (void*)&buffer, msg_len, 0, (struct sockaddr *) &ska, sizeof(ska)) != msg_len)
   {
     // error_handler ("sendto() [udp_send()]");
      return 0;
   }

   return 1;
}


data_AHRS_Struct udp_receive (int sk, int *dimr)
{
  int dim;
  data_AHRS_Struct buffer;
   len_of_reply_to_socket_address = sizeof (reply_to_socket_address);
  if ((dim = recvfrom (sk, (void*)&buffer, sizeof(buffer),O_NONBLOCK,
                        (struct sockaddr *) &reply_to_socket_address,
                        (socklen_t *) &len_of_reply_to_socket_address)) < 0)
   {
	*dimr=dim;
        //error_handler ("recvfrom() [udp_receive()]");
         return buffer;
   }
   *dimr=dim;
   return buffer;
}



void error_handler (char *message)
{
   printf ("fatal error: %s\n", message);
   exit (EXIT_FAILURE);

}



