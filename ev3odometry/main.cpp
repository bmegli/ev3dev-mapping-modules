/*
 * ev3odometry program
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
 /*   
  * This program was created for EV3 with ev3dev OS 
  * 
  * ev3odometry:
  * -reads 2 motors positions and speeds
  * -timestamps the data
  * -sends the above data in UDP messages
  * -waits defined time before next read
  *
  * Preconditions (for EV3/ev3dev):
  * -two tacho motors connected to ports A, D
  * 
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "shared/misc.h"
#include "shared/net_udp.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <stdio.h>
#include <endian.h> //htobe16, htobe32, htobe64
#include <limits.h> //INT_MAX

struct odometry_packet
{
	uint64_t timestamp_us;
	int32_t position_left;
	int32_t position_right;
	int32_t speed_left;
	int32_t speed_right;
	int16_t reserved1;
	int16_t reserved2;
};

const int ODOMETRY_PACKET_BYTES=28; //2*2 + 4*4 + 8

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const ev3dev::large_motor &left,const ev3dev::large_motor &right, int poll_ms);

void InitDriveMotor(ev3dev::large_motor *m);

int EncodeOdometryPacket(const odometry_packet &packet, char *buffer);
void SendOdometryFrameUDP(int socket, const sockaddr_in &dest, const odometry_packet &frame);

void Usage();
int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms);

int main(int argc, char **argv)
{
	int socket_udp;
	sockaddr_in destination_udp;
	
	int port, poll_ms;
		
	if( ProcessInput(argc, argv, &port, &poll_ms) )
	{
		Usage();
		return 0;
	}
	const char *host=argv[1];
	
	ev3dev::large_motor motor_left(ev3dev::OUTPUT_A);
	ev3dev::large_motor motor_right(ev3dev::OUTPUT_D);

	SetStandardInputNonBlocking();	

	InitNetworkUDP(&socket_udp, &destination_udp, host, port, 0);
	
	InitDriveMotor(&motor_left);
	InitDriveMotor(&motor_right);
		
	MainLoop(socket_udp, destination_udp, motor_left, motor_right, poll_ms);
	
	CloseNetworkUDP(socket_udp);

	return 0;
}

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const ev3dev::large_motor &motor_left,const ev3dev::large_motor &motor_right, int poll_ms)
{
	const int BENCHS=INT_MAX;
		
	struct odometry_packet frame;
	uint64_t start=TimestampUs();
	int i, elapsed_us, poll_us=1000*poll_ms;
	
	for(i=0;i<BENCHS;++i)
	{
		frame.timestamp_us=TimestampUs();	
		frame.position_left=  motor_left.position();
		frame.position_right=  motor_right.position();
		SendOdometryFrameUDP(socket_udp, destination_udp, frame);

		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;
		
		elapsed_us=(int)(TimestampUs()-frame.timestamp_us);
		
		if( elapsed_us < poll_us )
			SleepUs(poll_us - elapsed_us);
	}
		
	uint64_t end=TimestampUs();
	double seconds_elapsed=(end-start)/ 1000000.0L;
	printf("%f\n", seconds_elapsed/i);
}

void InitDriveMotor(ev3dev::large_motor *m)
{
	if(!m->connected())
		Die("Motor not connected");
}

int EncodeOdometryPacket(const odometry_packet &p, char *data)
{
	*((uint64_t*)data) = htobe64(p.timestamp_us);
	data += sizeof(p.timestamp_us);

	*((uint32_t*)data)= htobe32(p.position_left);
	data += sizeof(p.position_left);

	*((uint32_t*)data)= htobe32(p.position_right);
	data += sizeof(p.position_right);

	*((uint32_t*)data)= htobe32(p.speed_left);
	data += sizeof(p.speed_left);

	*((uint32_t*)data)= htobe32(p.speed_right);
	data += sizeof(p.speed_right);
	
	*((uint16_t*)data)= htobe16(0);
	data += sizeof(p.reserved1);

	*((uint16_t*)data)= htobe16(0);
	data += sizeof(p.reserved2);
		
	return ODOMETRY_PACKET_BYTES;	
}
void SendOdometryFrameUDP(int socket, const sockaddr_in &destination, const odometry_packet &frame)
{
	static char buffer[ODOMETRY_PACKET_BYTES];
	EncodeOdometryPacket(frame, buffer);
	SendToUDP(socket, destination, buffer, ODOMETRY_PACKET_BYTES);
}

void Usage()
{
	printf("ev3odemtry host port poll_ms\n\n");
	printf("examples:\n");
	printf("./ev3odometry 192.168.0.103 8005 10\n");
}

int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms)
{
	long int port, poll_ms;
			
	if(argc!=4)
		return -1;
		
	port=strtol(argv[2], NULL, 0);
	if(port <= 0 || port > 65535)
	{
		fprintf(stderr, "The argument port has to be in range <1, 65535>\n");
		return -1;
	}
	*out_port=port;

	poll_ms=strtol(argv[3], NULL, 0);
	if(poll_ms <= 0 || poll_ms > 1000)
	{
		fprintf(stderr, "The argument poll_ms has to be in range <1, 1000>\n");
		return -1;
	}
	*out_poll_ms=poll_ms;
	
	return 0;
}