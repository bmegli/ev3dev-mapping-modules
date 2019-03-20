/*
 * ccmcu 
 *
 * Copyright 2019 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */
 
 /*
  * This example:
  * - initilies communication with cave-crawler microcontroller
  * - reads & prints data from cave-crawler mcu 1000 times
  * - cleans after itself
  * 
  * Program expects terminal device, e.g.
  * 
  * ./cc-read-all /dev/ttyACM0
  * 
  *
  */

#include "cave-crawler-lib/cave_crawler.h"

#include "../lib/shared/misc.h"
#include "../lib/shared/net_udp.h"

#include <string.h> //memcpy
#include <signal.h> //sigaction, sig_atomic_t
#include <stdio.h> //printf
#include <stdlib.h> //exit

// constants
const int DATA_SIZE=10; 

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;

void main_loop(int odometry_socket, const sockaddr_in &odometry_dst, struct cc *c);

int EncodeDeadReconningPacket(const cc_odometry_data &packet, char *buffer);
void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const cc_odometry_data &frame);

void process_arguments(int argc, char **argv, int *odometry_port);
void usage(char **argv);
void finish(int signal);

/* we will use cc_odometry_packet internally but
 * reencode it to dead_reconning_packet for network
struct dead_reconning_packet
{
	uint64_t timestamp_us;
	int32_t position_left;
	int32_t position_right;
	float w;
	float x;
	float y;
	float z;
};
*/

const int DEAD_RECONNING_PACKET_BYTES=32; //8 + 2*4 + 4*4

int main(int argc, char **argv)
{
	struct cc *c = NULL;
	int odometry_socket, odometry_port;
	sockaddr_in odometry_destination;

	//input
	process_arguments(argc, argv, &odometry_port);
	const char *tty_device=argv[1];
	const char *host=argv[2];

	SetStandardInputNonBlocking();
	RegisterSignals(finish);

	if( (c = cc_init(tty_device)) == NULL)
	{
		perror("ccmcu: unable to initialize cave-crawler mcu communication\n");
		g_finish_program=1;
	}	

	InitNetworkUDP(&odometry_socket, &odometry_destination, host, odometry_port, 0);
			
	main_loop(odometry_socket, odometry_destination, c);

	CloseNetworkUDP(odometry_socket);
			
	cc_close(c);
	
	printf("ccmcu: bye\n");
	
	return 0;
}

void main_loop(int odometry_socket, const sockaddr_in &odometry_dst, struct cc *c)
{
	struct cc_odometry_data odometry[DATA_SIZE];
	struct cc_rplidar_data rplidar[DATA_SIZE];
	struct cc_xv11lidar_data xv11lidar[DATA_SIZE];

	struct cc_data data = {0};
	struct cc_size size = {0}; 

	size.odometry = size.rplidar = size.xv11lidar = DATA_SIZE;

	data.odometry = odometry;
	data.rplidar = rplidar;
	data.xv11lidar = xv11lidar;
	data.size = size;
	
	int ret;
	
	while( !g_finish_program && ( ret = cc_read_all(c, &data) ) != CC_ERROR)
	{		
		for(int i=0;i<data.size.odometry;++i)
			SendDeadReconningPacketUDP(odometry_socket, odometry_dst, odometry[i]);

		for(int i=0;i<data.size.rplidar;++i)
			printf("[rp ] t=%u seq=%d\n",
			data.rplidar[i].timestamp_us, data.rplidar[i].sequence);

		for(int i=0;i<data.size.xv11lidar;++i)
			printf("[xv11] t=%u aq=%d s=%d d=?\n", data.xv11lidar[i].timestamp_us,
			data.xv11lidar[i].angle_quad, data.xv11lidar[i].speed64/64);
					
		//refresh the sizes of the arrays for data streams
		data.size=size;
		
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;		
	}
	
}

int encode_float(float f, char *buffer)
{
	uint32_t t32;
	memcpy(&t32, &f, sizeof(float));
	t32=htobe32(t32);
	memcpy(buffer, &t32, sizeof(float));
	return sizeof(float);
}

int encode_int16(int16_t i, char *buffer)
{
	uint16_t t16=htobe16(i);
	memcpy(buffer, &t16, sizeof(int16_t));
	return sizeof(int16_t);
}

int encode_int32(int32_t i, char *buffer)
{
	uint32_t t32=htobe32(i);
	memcpy(buffer, &t32, sizeof(int32_t));
	return sizeof(int32_t);
}

int EncodeDeadReconningPacket(const cc_odometry_data &p, char *buffer)
{
	uint64_t temp64;
	size_t offset=0;
	
	temp64=htobe64(p.timestamp_us);
	memcpy(buffer, &temp64, sizeof(uint64_t));
	offset+=sizeof(uint64_t);

	offset+=encode_int32(p.left_encoder_counts, buffer+offset);
	offset+=encode_int32(p.right_encoder_counts, buffer+offset);

	offset += encode_float(p.qw, buffer+offset);
	offset += encode_float(p.qy, buffer+offset);
	offset += encode_float(p.qz, buffer+offset);	
	offset += encode_float(p.qx, buffer+offset);

	return DEAD_RECONNING_PACKET_BYTES;	
}

void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const cc_odometry_data &frame)
{
	static char buffer[DEAD_RECONNING_PACKET_BYTES];
	EncodeDeadReconningPacket(frame, buffer);
	SendToUDP(socket, dest, buffer, DEAD_RECONNING_PACKET_BYTES);
}


void process_arguments(int argc, char **argv, int *odometry_port)
{
	//ccmcu tty_device host odometry_port //rplidar_port xv11lidar port
	if(argc!=4)
	{
		usage(argv);
		exit(EXIT_SUCCESS);		
	}
	
	long temp;
	
	temp=strtol(argv[3], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccmcu: the argument server_port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*odometry_port=temp;
}
void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s tty_device host odometry_port\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0 192.168.0.125 8013\n", argv[0]);
}

void finish(int signal)
{
	g_finish_program=1;	
}
