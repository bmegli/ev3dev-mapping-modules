/*
 * ev3drive program
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
  * ev3drive:
  * -initializes 2 motors
  * -reads UDP messages
  * -sets motor speeds accordingly
  * -or sets motor positions and speeds accordingly
  * -stops motors on timeout
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */


#include "shared/misc.h"
#include "shared/net_udp.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <signal.h> //sigaction, sig_atomic_t
#include <endian.h> //htobe16, htobe32, htobe64
#include <stdio.h> //printf, etc

using namespace ev3dev;

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;

// temporary control packet, subject to change
struct drive_packet
{
	uint64_t timestamp_us;
	int16_t command;
	int16_t param1;
	int16_t param2;	
	int16_t param3;
	int16_t param4;	
};

const int CONTROL_PACKET_BYTES = 18; //8 + 5*2 = 18 bytes
enum Commands {KEEPALIVE=0, SET_SPEED=1, TO_POSITION_WITH_SPEED=2};

void MainLoop(int socket_udp, large_motor *left, large_motor *right);
void ProcessMessage(const drive_packet &packet,large_motor *left, large_motor *right);

void InitMotor(large_motor *m);
void StopMotors(large_motor *left, large_motor *right);

int RecvDrivePacket(int socket_udp, drive_packet *packet);
void DecodeDrivePacket(drive_packet *packet, const char *data);

void Usage();
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms);
void Finish(int signal);


int main(int argc, char **argv)
{			
	int socket_udp, port, timeout_ms;
	sockaddr_in destination_udp;
	large_motor motor_left(OUTPUT_A);
	large_motor motor_right(OUTPUT_D);
	
	ProcessArguments(argc, argv, &port, &timeout_ms);
	SetStandardInputNonBlocking();
	
	//init
	RegisterSignals(Finish);

	InitMotor(&motor_left);
	InitMotor(&motor_right);
		
	InitNetworkUDP(&socket_udp, &destination_udp, NULL, port, timeout_ms);

	//work
	MainLoop(socket_udp, &motor_left, &motor_right);
	
	//cleanup
	StopMotors(&motor_left, &motor_right);
	CloseNetworkUDP(socket_udp);
		
	return 0;
}

void MainLoop(int socket_udp, large_motor *left, large_motor *right)
{
	int status;
	drive_packet packet;
	
	while(!g_finish_program)
	{
		status=RecvDrivePacket(socket_udp, &packet);
		if(status < 0)
			break;
		if(status == 0) //timeout
		{
			StopMotors(left, right);
			fprintf(stderr, "waiting for drive controller...\n");
		}
		else
			ProcessMessage(packet, left, right);

		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;
	}
}

void ProcessMessage(const drive_packet &packet,large_motor *left, large_motor *right)
{	
	if(packet.command == KEEPALIVE)
		return;
		
	if(packet.command == SET_SPEED)
	{		
		int16_t l=packet.param1, r=packet.param2;
		left->set_speed_sp(l); 
		right->set_speed_sp(r); 
		
		(l!=0) ? left->run_forever() : left->stop(); 
		(r!=0) ? right->run_forever() : right->stop(); 
	}
	else if(packet.command == TO_POSITION_WITH_SPEED)
	{
		left->stop();
		right->stop();
		
		int16_t lspeed=packet.param1, rspeed=packet.param2;
		int16_t lpos=packet.param3, rpos=packet.param4;
		left->set_speed_sp(lspeed); 
		right->set_speed_sp(rspeed); 
		left->set_position_sp(lpos);
		right->set_position_sp(rpos);
	
		left->run_to_rel_pos();
		right->run_to_rel_pos();
	}
}

void InitMotor(large_motor *m)
{
	if(!m->connected())
		Die("Motor not connected");
	m->set_stop_action(m->stop_action_coast);
}

void StopMotors(large_motor *left, large_motor *right)
{
	left->stop();
	right->stop();	
}


// returns CONTROL_PACKET_BYTES on success  0 on timeout, -1 on error
int RecvDrivePacket(int socket_udp, drive_packet *packet)
{
	static char buffer[CONTROL_PACKET_BYTES];	
	int recv_len;
	
	if((recv_len = recvfrom(socket_udp, buffer, CONTROL_PACKET_BYTES, 0, NULL, NULL)) == -1)	
	{
		if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINPROGRESS)
			return 0; //timeout!
		perror("Error while receiving control packet");
		return -1;
	}
	if(recv_len < CONTROL_PACKET_BYTES)
	{
		fprintf(stderr, "Received incomplete datagram\n");
		return -1; 
	}
	DecodeDrivePacket(packet, buffer);	

	return recv_len;	       
}
 
void DecodeDrivePacket(drive_packet *packet, const char *data)
{
	packet->timestamp_us=be64toh(*((uint64_t*)data));
	packet->command=be16toh(*((int16_t*)(data+8)));
	packet->param1=be16toh(*((int16_t*)(data+10)));
	packet->param2=be16toh(*((int16_t*)(data+12)));	
	packet->param3=be16toh(*((int16_t*)(data+14)));
	packet->param4=be16toh(*((int16_t*)(data+16)));
}

void Usage()
{
	printf("ev3drive udp_port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ev3drive 8003 500\n");
}
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms)
{
	if(argc!=3)
	{
		Usage();
		exit(EXIT_SUCCESS);		
	}
	
	long temp;
	
	temp=strtol(argv[1], NULL, 0);
	*port=temp;
	temp=strtol(argv[2], NULL, 0);
	*timeout_ms=temp;
}
void Finish(int signal)
{
	g_finish_program=1;
}
