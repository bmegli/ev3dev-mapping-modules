/*
 * ccdrive program
 *
 * Copyright (C) 2018 Bartosz Meglicki <meglickib@gmail.com>
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
  * 
  * ccdrive: TODO
  * -initializes 
  * -reads UDP messages
  * -sets motor speeds accordingly
  * -or sets motor positions and speeds accordingly
  * -stops motors on timeout
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */


#include "shared/misc.h"
#include "shared/net_udp.h"

#include "roboclaw/roboclaw.h"

#include <signal.h> //sigaction, sig_atomic_t
#include <endian.h> //htobe16, htobe32, htobe64
#include <stdio.h> //printf, etc
#include <stdlib.h> //exit
#include <errno.h> //errno


// constants
const uint8_t FRONT_MOTOR_ADDRESS=0x80;
const uint8_t MIDDLE_MOTOR_ADDRESS=0x81;
const uint8_t REAR_MOTOR_ADDRESS=0x82;
const int32_t MOTOR_ACCELERATION=6000;

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

void MainLoop(int socket_udp, roboclaw *rc);
void ProcessMessage(const drive_packet &packet, roboclaw *rc);

roboclaw *InitMotors(const char *tty, int baudrate);
void CloseMotors(roboclaw *rc);
void StopMotors(roboclaw *rc);

int RecvDrivePacket(int socket_udp, drive_packet *packet);
void DecodeDrivePacket(drive_packet *packet, const char *data);

void Usage();
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms);
void Finish(int signal);


int main(int argc, char **argv)
{			
	int socket_udp, port, timeout_ms;
	sockaddr_in destination_udp;

	roboclaw *rc;
	
	ProcessArguments(argc, argv, &port, &timeout_ms);
	SetStandardInputNonBlocking();
	
	//init
	RegisterSignals(Finish);

	rc=InitMotors("/dev/ttyXRUSB0", 460800); //TO DO hardcoded params
		
	InitNetworkUDP(&socket_udp, &destination_udp, NULL, port, timeout_ms);

	//work
	MainLoop(socket_udp, rc);
	
	//cleanup
	StopMotors(rc);
	CloseMotors(rc);
	CloseNetworkUDP(socket_udp);
		
	printf("ccdrive: bye\n");
		
	return 0;
}

void MainLoop(int socket_udp, roboclaw *rc)
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
			StopMotors(rc);
			fprintf(stderr, "ccdrive: waiting for drive controller...\n");
		}
		else
			ProcessMessage(packet, rc);

		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;
	}
}

void ProcessMessage(const drive_packet &packet, roboclaw *rc)
{	
	if(packet.command == KEEPALIVE)
		return;
		
	if(packet.command == SET_SPEED)
	{		
		int16_t l=packet.param1, r=packet.param2;
		bool ok=true;

		ok &= roboclaw_speed_accel_m1m2(rc, FRONT_MOTOR_ADDRESS, r, l, MOTOR_ACCELERATION) == ROBOCLAW_OK;
		ok &= roboclaw_speed_accel_m1m2(rc, MIDDLE_MOTOR_ADDRESS, r, l,  MOTOR_ACCELERATION) == ROBOCLAW_OK;
		ok &= roboclaw_speed_accel_m1m2(rc, REAR_MOTOR_ADDRESS, r, l, MOTOR_ACCELERATION) == ROBOCLAW_OK;	

		if(!ok)
			fprintf(stderr, "ccdrive: failed to set motor speed, no reaction implemented\n");
	}
	else if(packet.command == TO_POSITION_WITH_SPEED)
	{
		//TO DO
		//left->stop();
		//right->stop();
		
		//int16_t lspeed=packet.param1, rspeed=packet.param2;
		//int16_t lpos=packet.param3, rpos=packet.param4;

//TO DO
//		left->set_speed_sp(lspeed); 
//		right->set_speed_sp(rspeed); 
//		left->set_position_sp(lpos);
//		right->set_position_sp(rpos);
	
//		left->run_to_rel_pos();
//		right->run_to_rel_pos();
	}
}

roboclaw *InitMotors(const char *tty, int baudrate)
{
	roboclaw *rc;
	int16_t voltage;
	bool ok=true;
	
	rc=roboclaw_init(tty, baudrate);
	
	if(!rc)
		DieErrno("ccdrive: unable to initialize motors");
	
	ok &= roboclaw_main_battery_voltage(rc, FRONT_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(rc, MIDDLE_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(rc, REAR_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	
	if(!ok)
	{
		CloseMotors(rc); //TO DO add more informative message
		Die("ccdrive: unable to communicate with motors");
	}
	
	printf("ccdrive: battery voltage is %d.%d\n", voltage/10, voltage % 10);
		
	return rc;
}

void CloseMotors(roboclaw *rc)
{
	roboclaw_close(rc);
}
void StopMotors(roboclaw *rc)
{
	bool ok=true;
	ok &= roboclaw_duty_m1m2(rc, FRONT_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;
	ok &= roboclaw_duty_m1m2(rc, MIDDLE_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;
	ok &= roboclaw_duty_m1m2(rc, REAR_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;

	if(!ok)
		fprintf(stderr, "ccdrive: unable to stop motors\n");
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
		perror("ccdrive: error while receiving control packet");
		return -1;
	}
	if(recv_len < CONTROL_PACKET_BYTES)
	{
		fprintf(stderr, "ccdrive: received incomplete datagram\n");
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
	printf("ccdrive udp_port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ccdrive 8003 500\n");
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
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccdrive: the argument port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*port=temp;
	temp=strtol(argv[2], NULL, 0);
	if(temp <= 0 || temp > 10000)
	{
		fprintf(stderr, "ccdrive: the argument timeout_ms has to be in range <1, 10000>\n");
		exit(EXIT_SUCCESS);
	}
	
	*timeout_ms=temp;
}
void Finish(int signal)
{
	g_finish_program=1;
}
