/*
 * ev3control program
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
  * ev3control:
  * -reads UDP messages
  * -starts modules on request
  * -monitors modules state
  * -stops modules on request (by closing their stdin so that they get EOF on read)
  *
  * See Usage() function for syntax details (or run the program without arguments)
  * 
  * 
  *  This is PRELIMINARY version:
  * -it's insecure (can run arbirary program on EV3!)
  * -it will transition to TCP/IP (UDP is not really applicable here in the long run!)
  * -the protocol will shift to variable length messages
  * -once module is loaded ev3control has to be restarted to change it's parameters
  * -code needs cleanup
  * 
  */


#include "modules.h"

#include "shared/misc.h"
#include "shared/net_udp.h"

#include <signal.h> //sigaction, sig_atomic_t
#include <endian.h> //htobe16, htobe32, htobe64
#include <stdio.h> //printf, etc
#include <errno.h> // errno
#include <stdlib.h> //EXIT_FAILURE
#include <string.h> //memcpy
#include <map> //map
#include <string> //string

using namespace std;

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;

enum Commands {ENABLE=0, DISABLE=1, DISABLE_ALL=3, ENABLED=4, DISABLED=5, FAILED=6 };

struct control_packet
{
	uint64_t timestamp_us;
	int16_t command;
	int16_t creation_delay_ms;
	char unique_name[MODULE_NAME_WITH0_MAX];
	char call[MODULE_CALL_WITH0_MAX];
};

const int CONTROL_PACKET_BYTES = sizeof(uint64_t) +  2 * sizeof(int16_t) +  MODULE_NAME_WITH0_MAX + MODULE_CALL_WITH0_MAX;   //8 + 4 + 20+128+=160

void MainLoop(int socket_udp, map<string, robot_module> &modules);
void ProcessMessage(int socket_udp, const struct sockaddr_in &address, const control_packet &packet, map<string, robot_module> &modules);
void FailedModulesNotify(int socket_udp, const struct sockaddr_in &address, map<string, robot_module> &modules);

int RecvControlPacket(int socket_udp,  struct sockaddr_in *host_address, control_packet *packet);
void SendControlPacket(int socket_udp, const sockaddr_in &dst, const control_packet &packet);
void DecodeControlPacket(control_packet *packet, const char *data);
int EncodeControlPacket(const control_packet &p, char *data);

robot_module ModuleFromControlPacket(const control_packet &packet);
void ControlPacketFromModule(const robot_module &module, control_packet *packet, Commands command);

void Usage();
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms);
void Finish(int signal);

int main(int argc, char **argv)
{			
	int socket_udp, port, timeout_ms;
	
	map<string, robot_module> modules;

	ProcessArguments(argc, argv, &port, &timeout_ms);
	
	//init
	RegisterSignals(Finish);

	InitNetworkUDP(&socket_udp, NULL, NULL, port, timeout_ms);

	//work
	MainLoop(socket_udp, modules);
	
	//cleanup
	DisableModules(modules);
	CloseNetworkUDP(socket_udp);
		
   	return 0;
}

void MainLoop(int socket_udp, map<string, robot_module> &modules)
{
	int status;
	bool received_first=false; //and initialized host address
	control_packet packet; 
	struct sockaddr_in host_address;
		
	while(!g_finish_program)
	{
		status=RecvControlPacket(socket_udp,&host_address, &packet);
		if(status < 0)
			break;
		if(status == 0)
		{//timeout, check modules status
			if( received_first &&  !EnabledModulesRunning(modules) )
				FailedModulesNotify(socket_udp, host_address, modules);
		}
		else
		{
			ProcessMessage(socket_udp, host_address, packet, modules);
			received_first=true;
		}
	}
}

void ProcessMessage(int socket_udp, const struct sockaddr_in &address, const control_packet &packet, map<string, robot_module> &modules)
{
	static control_packet response;
	
	if(packet.command == ENABLE)
	{
		printf("Request: ENABLE \"%s\" with \"%s\" \n", packet.unique_name, packet.call);

		if( modules.find(packet.unique_name) == modules.end() )
			modules.insert(pair<string, robot_module>(packet.unique_name, ModuleFromControlPacket(packet)) );
			
		EnableModule(modules, packet.unique_name);
		ControlPacketFromModule(modules[packet.unique_name], &response, ENABLED); 
		SendControlPacket(socket_udp, address, response);
	}
	else if(packet.command == DISABLE)
	{
		printf("Request: DISABLE \"%s\" with \"%s\" \n", packet.unique_name, packet.call);
		if( modules.find(packet.unique_name) == modules.end() )
		{
			fprintf(stderr, "Requested to diable module but module doesn't exist\n");
			return;
		}
		DisableModule(modules, packet.unique_name);
		ControlPacketFromModule(modules[packet.unique_name], &response, DISABLED);
		SendControlPacket(socket_udp, address, response);
	}
	else if(packet.command == DISABLE_ALL)
	{
		printf("Request: DISABLE_ALL modules\n");
		DisableModules(modules);
		//to do - some response
	}
	else
		fprintf(stderr, "Ignoring unknown command: %d\n", packet.command);	
}
void FailedModulesNotify(int socket_udp, const struct sockaddr_in &address, map<string, robot_module> &modules)
{
	static control_packet packet;
	
	map<string, robot_module>::iterator it;
	for(it=modules.begin();it!=modules.end();++it)
		if(it->second.state==MODULE_FAILED)
		{
			ControlPacketFromModule(it->second, &packet, FAILED);
			SendControlPacket(socket_udp, address, packet);
		}
}
 
int RecvControlPacket(int socket_udp, struct sockaddr_in *host_address, control_packet *packet)
{
	static char buffer[CONTROL_PACKET_BYTES];	
	int recv_len;
	socklen_t host_address_length=sizeof(*host_address);

	if((recv_len = recvfrom(socket_udp, buffer, CONTROL_PACKET_BYTES, 0,(struct sockaddr*) host_address, &host_address_length)) == -1)	
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
	
	if(host_address_length>sizeof(*host_address))
	{ //only warn for now
		fprintf(stderr, "Warning - address longer than supplied structure! Not filled!");	
	}
	DecodeControlPacket(packet, buffer);	

	return recv_len;	       
}

void SendControlPacket(int socket_udp, const sockaddr_in &dst, const control_packet &packet)
{
	static char buffer[CONTROL_PACKET_BYTES];
	EncodeControlPacket(packet, buffer);
	SendToUDP(socket_udp, dst, buffer, CONTROL_PACKET_BYTES);
}
 
void DecodeControlPacket(control_packet *packet, const char *data)
{
	packet->timestamp_us=be64toh(*((uint64_t*)data));
	packet->command=be16toh(*((int16_t*)(data+8)));
	packet->creation_delay_ms=be16toh(*((int16_t*)(data+10)));
	memcpy(packet->unique_name, data + 12, MODULE_NAME_WITH0_MAX);
	packet->unique_name[MODULE_NAME_WITH0_MAX-1]='\0';
	memcpy(packet->call, data + 12 + MODULE_NAME_WITH0_MAX, MODULE_CALL_WITH0_MAX);
	packet->call[MODULE_CALL_WITH0_MAX-1]='\0';
}
int EncodeControlPacket(const control_packet &p, char *data)
{
	*((uint64_t*)data) = htobe64(p.timestamp_us);
	data += sizeof(p.timestamp_us);

	*((int16_t*)data)= htobe16(p.command);
	data += sizeof(p.command);

	*((int16_t*)data)= htobe16(p.creation_delay_ms);
	data += sizeof(p.creation_delay_ms);

	memcpy(data, p.unique_name, MODULE_NAME_WITH0_MAX);
	*(data+MODULE_NAME_WITH0_MAX-1)='\0';
	data += MODULE_NAME_WITH0_MAX;

	memcpy(data, p.call, MODULE_CALL_WITH0_MAX);
	*(data+MODULE_CALL_WITH0_MAX-1)='\0';
	data += MODULE_CALL_WITH0_MAX;
	
	return CONTROL_PACKET_BYTES;	
}

robot_module ModuleFromControlPacket(const control_packet &packet)
{
	robot_module module;
	module.unique_name=string(packet.unique_name);
	module.call=string(packet.call);
	module.creation_sleep_ms=packet.creation_delay_ms;
	module.state=MODULE_DISABLED;
	module.pid=0;
	module.write_fd=0;	
	return module;
}
void ControlPacketFromModule(const robot_module &module, control_packet *packet, Commands command)
{
	packet->timestamp_us=TimestampUs();
	packet->command=command;
	packet->creation_delay_ms=module.creation_sleep_ms;
	memcpy(packet->unique_name, module.unique_name.c_str(), module.unique_name.size()+1);
	memcpy(packet->call, module.call.c_str(), module.call.size()+1);
}

void Usage()
{
	printf("ev3control udp_port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ev3control 8004 500\n");
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

