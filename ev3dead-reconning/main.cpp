/*
 * ev3dead-reconning program
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
  * ev3dead-reconning:
  * -reads 2 motors positions
  * -reads gyroscope angle
  * -timestamps the data
  * -sends the above data in UDP messages
  *
  * Preconditions (for EV3/ev3dev):
  * -two tacho motors connected to ports A, D
  * -MicroInfinity CruizCore XG1300L gyroscope connected to port 3 with manually loaded I2C driver
  * . 
  * See Usage() function for syntax details (or run the program without arguments)
  */

// GYRO CONSTANTS
const char *GYRO_PORT="i2c-legoev35:i2c1";
const int GYRO_PATH_MAX=100;
char GYRO_PATH[GYRO_PATH_MAX]="/sys/class/lego-sensor/sensor";

#include "shared/misc.h"
#include "shared/net_udp.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <limits.h> //INT_MAX
#include <stdio.h>
#include <string.h> //memcpy
#include <unistd.h> //open, close, read, write
#include <fcntl.h> //O_RDONLY flag
#include <endian.h> //htobe16, htobe32, htobe64

struct dead_reconning_packet
{
	uint64_t timestamp_us;
	int32_t position_left;
	int32_t position_right;
	int32_t speed_left;
	int32_t speed_right;
	int16_t heading;
	int16_t angular_speed;
};

const int DEAD_RECONNING_PACKET_BYTES=28; //2*2 + 4*4 + 8

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const ev3dev::large_motor &left,const ev3dev::large_motor &right, int gyro_direct_fd, int poll_ms);

void InitDriveMotor(ev3dev::large_motor *m);
int InitGyro(ev3dev::i2c_sensor *gyro);
int ReadGyroAngle(int gyro_direct_fd, int16_t *out_angle);

int EncodeDeadReconningPacket(const dead_reconning_packet &packet, char *buffer);
void SendDeadReconningFrameUDP(int socket, const sockaddr_in &dest, const dead_reconning_packet &frame);

void Usage();
int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms);

int main(int argc, char **argv)
{
	int socket_udp, gyro_direct_fd;
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
	ev3dev::i2c_sensor gyro(GYRO_PORT, {"mi-xg1300l"});

	SetStandardInputNonBlocking();	

	gyro_direct_fd=InitGyro(&gyro);

	InitNetworkUDP(&socket_udp, &destination_udp, host, port, 0);
	
	InitDriveMotor(&motor_left);
	InitDriveMotor(&motor_right);
		
	MainLoop(socket_udp, destination_udp, motor_left, motor_right, gyro_direct_fd, poll_ms);
	
	close(gyro_direct_fd);
	CloseNetworkUDP(socket_udp);

	return 0;
}

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const ev3dev::large_motor &motor_left,const ev3dev::large_motor &motor_right, int gyro_direct_fd, int poll_ms)
{
	const int BENCHS=INT_MAX;
		
	struct dead_reconning_packet frame;
	int16_t heading;
	uint64_t start=TimestampUs();
	int i, enxios=0, elapsed_us, poll_us=1000*poll_ms;
		
	for(i=0;i<BENCHS;++i)
	{	
		frame.timestamp_us=TimestampUs();
		frame.position_left=  motor_left.position();
		frame.position_right=  motor_right.position();
		
		if(ReadGyroAngle(gyro_direct_fd, &heading) == -ENXIO)
		{ //this is workaround for occasional ENXIO problem
			fprintf(stderr, "Got ENXIO, retrying %d\n", ++enxios);
			continue; //we need to collect data again, this failure could be time consuming
		}
		frame.heading=heading;
		SendDeadReconningFrameUDP(socket_udp, destination_udp, frame);
		enxios=0; //part of workaround for occasoinal ENXIO

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
int InitGyro(ev3dev::i2c_sensor *gyro)
{
	int fd;
	
	if(!gyro->connected())	
		Die("Unable to find gyroscope");
		
	gyro->set_poll_ms(0);
	gyro->set_command("RESET");
	
	printf("Callculating gyroscope bias drift\n");
	Sleep(1000);
	fflush(stdout);
	
	snprintf(GYRO_PATH+strlen(GYRO_PATH), GYRO_PATH_MAX, "%d/direct", gyro->device_index());
	
	if((fd=open(GYRO_PATH, O_RDONLY))==-1)	
		DieErrno("open(GYRO_PATH, O_RDONLY)");

	printf("Gyroscope ready\n");

	return fd;
}

int ReadGyroAngle(int gyro_direct_fd, int16_t *out_angle)
{
	char temp[2];
	int result;
	
	if(lseek(gyro_direct_fd, 0x42, SEEK_SET)==-1)
		DieErrno("lseek(gyro_direct_fd, 0x42, SEEK_SET)==-1");
		
	if( (result=read(gyro_direct_fd, temp, 2 )) == 2)
	{
		memcpy(out_angle, temp, 2);
		return 0;
	}	
		
	if( (result <= 0 && errno != ENXIO) )
		DieErrno("Read Gyro failed");
		
	if( result == 1)
		Die("Incomplete I2C read");
	
	return -ENXIO;			
}

int EncodeDeadReconningPacket(const dead_reconning_packet &p, char *data)
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
	
	*((uint16_t*)data)= htobe16(p.heading);
	data += sizeof(p.heading);

	*((uint16_t*)data)= htobe16(p.angular_speed);
	data += sizeof(p.angular_speed);
	
	return DEAD_RECONNING_PACKET_BYTES;	
}
void SendDeadReconningFrameUDP(int socket, const sockaddr_in &destination, const dead_reconning_packet &frame)
{
	static char buffer[DEAD_RECONNING_PACKET_BYTES];
	EncodeDeadReconningPacket(frame, buffer);
	SendToUDP(socket, destination, buffer, DEAD_RECONNING_PACKET_BYTES);
}

void Usage()
{
	printf("ev3dead-reconning host port poll_ms\n\n");
	printf("examples:\n");
	printf("./ev3dead-reconning 192.168.0.103 8005 10\n");
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