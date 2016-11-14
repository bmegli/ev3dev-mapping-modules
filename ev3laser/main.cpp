/*
 * ev3laser program
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
  * This program was created for EV3 & XV11 lidar with ev3dev OS
  * 
  * ev3laser:
  * -starts lidar motor
  * -reads lidar data from tty
  * -timestamps the data
  * -sends the above data in UDP messages
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "shared/misc.h"
#include "shared/net_udp.h"

#include "xv11lidar/xv11lidar.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <limits.h> //INT_MAX
#include <stdio.h>
#include <signal.h> //sigaction
#include <string.h> //memset
#include <endian.h> //htobe16, htobe32, htobe64

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;

const int TTY_PATH_MAX=100;
const int LASER_FRAMES_PER_READ=10;

struct laser_packet
{
	uint64_t timestamp_us;
	uint16_t laser_speed; //fixed point, 6 bits precision, divide by 64.0 to get floating point 
	uint16_t laser_angle; //angle of laser_readings[0]
	laser_reading laser_readings[4*LASER_FRAMES_PER_READ];
};

const int LASER_PACKET_BYTES = 12 + 16 * LASER_FRAMES_PER_READ;

void MainLoop(int socket_udp, const struct sockaddr_in &address, struct xv11lidar_data *laser, ev3dev::dc_motor *laser_motor);

int ProcessInput(int argc, char **argv, int *port, int *duty_cycle, int *crc_tolerance_pct);
void Usage();
void RegisterSignals();
void Finish(int signal);

void InitLaserMotor(ev3dev::dc_motor *m, int duty_cycle);

int EncodeLaserReading(const laser_reading *reading, char *data);
int EncodeLaserFrame(const laser_frame *frame, char *data);
int EncodeLaserPacket(const laser_packet &p, char *data);

void SendLaserPacket(int socket_udp, const sockaddr_in &dst, const laser_packet &packet);

int main(int argc, char **argv)
{
	int socket_udp;
	struct sockaddr_in address_udp;
	struct xv11lidar_data laser;    
	int port;
	int duty_cycle, crc_tolerance_pct;
	
	if( ProcessInput(argc, argv, &port, &duty_cycle, &crc_tolerance_pct) )
	{
		Usage();
		return 0;
	}
	SetStandardInputNonBlocking();
	
	const char *laser_tty=argv[1];
	const char *motor_port=argv[2];
	const char *host=argv[3];
			
	ev3dev::dc_motor motor(motor_port);

	RegisterSignals(Finish);
	InitNetworkUDP(&socket_udp, &address_udp, host, port, 0);
	InitLaserMotor(&motor, duty_cycle);
	//let the motor spin for a while
	Sleep(2000); 
	 
 	if( InitLaser(&laser, laser_tty, LASER_FRAMES_PER_READ, crc_tolerance_pct) !=SUCCESS )
	{
		fprintf(stderr, "ev3laser: init laser failed\n");
		g_finish_program=true;
	}

	MainLoop(socket_udp, address_udp, &laser, &motor);

	CloseLaser(&laser);
	motor.stop();
	CloseNetworkUDP(socket_udp);

	printf("ev3laser: bye\n");

	return 0;	
}

void MainLoop(int socket_udp, const struct sockaddr_in &address, struct xv11lidar_data *laser, ev3dev::dc_motor *laser_motor)
{
	struct laser_packet packet;
	struct laser_frame frames[LASER_FRAMES_PER_READ];
	uint64_t last_timestamp;
	uint32_t rpm;
	int status, counter, benchs=INT_MAX;

	uint64_t start=TimestampUs();	
	last_timestamp=start;
		
	for(counter=0;!g_finish_program && counter<benchs;++counter)
	{
		packet.timestamp_us=last_timestamp;
		
		if( (status=ReadLaser(laser, frames)) != SUCCESS )
		{
			fprintf(stderr, "ev3laser: ReadLaser failed with status %d\n", status);
			break;
		}
		// when read is finished, next read proceeds
		last_timestamp=TimestampUs(); 

		packet.laser_angle=(frames[0].index-0xA0)*4;
		
		rpm=0;
		for(int i=0;i<LASER_FRAMES_PER_READ;++i)
		{
			rpm+=frames[i].speed;
			memcpy(packet.laser_readings+4*i, frames[i].readings, 4*sizeof(laser_reading));
		}
		
		packet.laser_speed=rpm/LASER_FRAMES_PER_READ;
		
		SendLaserPacket(socket_udp, address, packet);
		
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;
	}
	
	uint64_t end=TimestampUs();
	double seconds_elapsed=(end-start)/ 1000000.0L;
	printf("ev3laser: average loop %f seconds\n", seconds_elapsed/counter);
	printf("ev3laser: last laser speed %f\n", packet.laser_speed/64.0);
}


int ProcessInput(int argc, char **argv, int *out_port, int *duty_cycle, int *crc_tolerance_pct)
{
	long int port, duty, crc;
				
	if(argc!=7)
		return -1;
		
	port=strtol(argv[4], NULL, 0);
	if(port <= 0 || port > 65535)
	{
		fprintf(stderr, "ev3laser: the argument port has to be in range <1, 65535>\n");
		return -1;
	}
	*out_port=port;

	duty=strtol(argv[5], NULL, 0);
	if(duty <= 0 || duty > 100)
	{
		fprintf(stderr, "ev3laser: the argument duty_cycle has to be in range <0, 100>\n");
		return -1;
	}
	*duty_cycle=duty;

	crc=strtol(argv[6], NULL, 0);
	if(crc < 0 || crc > 100)
	{
		fprintf(stderr, "ev3laser: the argument crc_tolerance_pct has to be in range <0, 100>\n");
		return -1;
	}
	*crc_tolerance_pct=crc;
		
	return 0;
}
void Usage()
{
	printf("ev3laser tty motor_port host port duty_cycle crc_tolerance_pct\n\n");
	printf("examples:\n");
	printf("./ev3laser /dev/tty_in2 outB 192.168.0.103 8002 40 10\n");
	printf("./ev3laser /dev/tty_in1 outC 192.168.0.103 8001 -40 10\n");
}

void Finish(int signal)
{
	g_finish_program=1;
}

void InitLaserMotor(ev3dev::dc_motor *m, int duty_cycle)
{
	if(!m->connected())
		Die("ev3laser: laser motor not connected");
	m->set_stop_action(ev3dev::motor::stop_action_coast);

	m->set_duty_cycle_sp(duty_cycle);
	m->run_direct();
}

 
int EncodeLaserReading(const laser_reading *reading, char *data)
{
	const uint16_t *reading_as_u16=(uint16_t*)reading;
	
	*((uint16_t*)data)=htobe16(reading_as_u16[0]);
	data += sizeof(uint16_t);
	*((uint16_t*)data)=htobe16(reading_as_u16[1]);
	data += sizeof(uint16_t);
	
	return 4;
}

int EncodeLaserFrame(const laser_frame *frame, char *data)
{
	*data=frame->start;
	++data;

	*data=frame->index;
	++data;
	
	*((uint16_t*)data)= htobe16(frame->speed);
	data += sizeof(frame->speed);
	
	for(int i=0;i<4;++i)
		data += EncodeLaserReading(frame->readings+i, data);
		
	*((uint16_t*)data)= htobe16(frame->checksum);
	data += sizeof(frame->speed);
	
	return 22;// 1 + 1 + 2 + 4*4 + 2;
}

int EncodeLaserPacket(const laser_packet &p, char *data)
{	
	*((uint64_t*)data) = htobe64(p.timestamp_us);
	data += sizeof(p.timestamp_us);

	*((uint16_t*)data)= htobe16(p.laser_speed);
	data += sizeof(p.laser_speed);
	
	*((uint16_t*)data)= htobe16(p.laser_angle);
	data += sizeof(p.laser_angle);
	
	for(int i=0;i<4*LASER_FRAMES_PER_READ; ++i)
		data += EncodeLaserReading(p.laser_readings+i, data);
		
	return 12 + 16 * LASER_FRAMES_PER_READ; //8 + 2 + 2 +  4*4 * LASER_FRAMES_PER_READ  	
}

void SendLaserPacket(int socket_udp, const sockaddr_in &dst, const laser_packet &packet)
{
	static char buffer[LASER_PACKET_BYTES];
	//memcpy(buffer, &packet, sizeof(laser_packet));
	EncodeLaserPacket(packet, buffer);
	SendToUDP(socket_udp, dst, buffer, LASER_PACKET_BYTES);
}