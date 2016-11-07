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
  * -starts TCP/IP server and waits for client
  * -reads messages
  * -starts modules on request
  * -disables modules on request (by closing their stdin so that they get EOF on read)
  * -monitors modules state
  * -informs peer on module state changes
  *
  * See Usage() function for syntax details (or run the program without arguments)
  * 
  *  This is PRELIMINARY version:
  * -it's insecure (can run arbirary program on EV3!)
  *
  * For funtional requirements see:
  * https://github.com/bmegli/ev3dev-mapping-modules/issues/20 
  */

#include "control.h"
#include "control_protocol.h"
#include "net_tcp.h"

#include "shared/misc.h"

#include <sys/socket.h> //send

#include <unistd.h> //read
#include <signal.h> //sigaction, sig_atomic_t
#include <stdio.h> //printf, etc
#include <errno.h> // errno
#include <stdlib.h> //EXIT_FAILURE


#include <list> //list
#include <string> //string

using namespace std;

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;

void ServerLoop(int serv_socket, int timeout_ms, Control *control);
void MainClientLoop(int client_socket, Control *control);

bool ReceiveMessage(int client_socket, Control *control);
bool ProcessMessage(int client_socket, const char *msg, char *response, Control *control);
void CheckProtocolVersion(const control_header &header);
bool CheckCommandSupport(const control_header &header);

bool ProcessMessageKEEPALIVE(int client_socket, const char *payload, const control_header &header, char *response, Control *control);
bool ProcessMessageENABLE(int client_socket, const char *payload, const control_header &header, char *response, Control *control);
bool ProcessMessageDISABLE(int client_socket, const char *payload, const control_header &header, char *response, Control *control);
bool ProcessMessageDISABLE_ALL(int client_socket, const char *payload, const control_header &header, char *response, Control *control);

bool CheckModulesStates(int client_socket, char *response, Control *control);

bool SendMessage(int sock, char *msg, int msg_len);

int EncodeModuleMessage(char *buffer, int buffer_length, ControlCommands command, const std::string &module_name);
int EncodeFailedMessage(char *buffer, int buffer_length, const std::string &module_name, int32_t status);
int EncodeKeepaliveMessage(char *buffer, int buffer_length);

void Usage();
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms);
void Finish(int signal);
void IgnoreSIGPIPE();

int main(int argc, char **argv)
{			
	int serv_socket, port, timeout_ms;
	Control control;
	
	ProcessArguments(argc, argv, &port, &timeout_ms);
	
	//init
	RegisterSignals(Finish);
	IgnoreSIGPIPE();
	InitNetworkTCP(&serv_socket, port);

	
	//work
	ServerLoop(serv_socket, timeout_ms, &control);
	
	//cleanup
	control.DisableModules();
	CloseNetworkTCP(serv_socket);
		
	printf("ev3control: bye\n");	
   	return 0;
}

void ServerLoop(int serv_socket, int timeout_ms, Control *control)
{
	int client_socket;
	
	while(!g_finish_program)
	{
		if( WaitForClientTCP(serv_socket, timeout_ms, &client_socket) == 0) //timeout
			continue; //check g_finish_program
		
		printf("ev3control: client connected\n");
		MainClientLoop(client_socket, control);
		printf("ev3control: client disconnected\n");
		
		CloseNetworkTCP(client_socket);
	}
	
}

void MainClientLoop(int client_socket, Control *control)
{
	while(!g_finish_program)
	{
		if(!ReceiveMessage(client_socket, control))
			break;
	}
}

// false on disconnect, true otherwise 
bool ReceiveMessage(int client_socket, Control *control)
{
	static char buffer[CONTROL_BUFFER_BYTES];	
	static char response_buffer[CONTROL_BUFFER_BYTES];		
	
	char *data=buffer;
	
	int received=0, bytes=CONTROL_HEADER_BYTES, result;
	
	while(received<bytes)
	{
		if( (result=read(client_socket, data+received, bytes-received)) == -1)
		{
			if(errno==EAGAIN || errno==EWOULDBLOCK) //timeout
			{
				if( !CheckModulesStates(client_socket, response_buffer, control))
				{
					printf("Check modules failed\n");
					return false;
				}
				continue;
			}
			else //maybe handle EINTR separately
			{
				perror("ev3control: read\n");
				return false; //connection closed
			}
		}
		else if(result == 0)
			return false;
			
		received += result;
		
		if(data == buffer && received==CONTROL_HEADER_BYTES)
		{ //we have received complete header, now get payload
			data=buffer+CONTROL_HEADER_BYTES;
			bytes = GetControlHeaderPayloadLength(buffer);
			received=0;
			if(bytes > CONTROL_BUFFER_BYTES-CONTROL_HEADER_BYTES)
			{
				fprintf(stderr, "ev3control: ignoring message, doesn't fit buffer: payload %d, buffer is %d\n", bytes, CONTROL_BUFFER_BYTES-CONTROL_HEADER_BYTES);
				return true;
			}
		}
	}		
	return ProcessMessage(client_socket, buffer, response_buffer, control);
}

bool ProcessMessage(int client_socket, const char *msg, char *response, Control *control)
{
	static bool (*handlers[])(int, const char *, const control_header &,char *,Control *)={ProcessMessageKEEPALIVE, ProcessMessageENABLE, ProcessMessageDISABLE, ProcessMessageDISABLE_ALL};
	static control_header header;
	
	GetControlHeader(msg, &header);
	
	CheckProtocolVersion(header);
	if( !CheckCommandSupport(header) )
		return true;
		
	return handlers[header.command](client_socket, msg+CONTROL_HEADER_BYTES, header,response, control);
}


void CheckProtocolVersion(const control_header &header)
{
	static bool protocol_version_not_yet_warned=true;
	if(protocol_version_not_yet_warned && header.protocol_version != CONTROL_PROTOCOL_VERSION)
	{
		fprintf(stderr, "ev3control: received message with protocol version %d, implementation version is %d\n", header.protocol_version, CONTROL_PROTOCOL_VERSION);
		fprintf(stderr, "ev3control: some functionality may not be supported\n");
		protocol_version_not_yet_warned=false;
	}
}

bool CheckCommandSupport(const control_header &header)
{
	if(header.command >= KEEPALIVE && header.command <= DISABLE_ALL)
		return true;
	fprintf(stderr, "ev3control: ignoring message with unsupported command %d\n", header.command);
	return false;
}


bool ProcessMessageKEEPALIVE(int client_socket, const char *payload, const control_header &header, char *response, Control *control)
{
	return CheckModulesStates(client_socket, response, control);
}

bool ProcessMessageENABLE(int client_socket, const char *payload, const control_header &header, char *response, Control *control)
{
	static control_attribute attributes[3];
	const static ControlAttributes expected[]={UNIQUE_NAME, CALL, CREATION_DELAY_MS};
	
	if(!ParseControlMessage(header, payload, attributes, 3, expected))
	{
		fprintf(stderr, "ev3control: ignoring invalid command %d\n", header.command);
		return true;
	}

	string unique_name(GetControlAttributeString(attributes[0]));
	string call(GetControlAttributeString(attributes[1]));
	uint16_t creation_delay_ms = GetControlAttributeU16(attributes[2]);
	
	printf("ev3control: request to enable %s\n", unique_name.c_str());
	
	Module module;
	bool contains_module=control->ContainsModule(unique_name, &module);
	
	if( contains_module && module.state == MODULE_ENABLED && control->CheckModuleState(unique_name) == MODULE_ENABLED )
	{
		fprintf(stderr, "ev3control: request to enable %s but it is enabled\n", unique_name.c_str());
		
		int response_length=EncodeModuleMessage(response, CONTROL_BUFFER_BYTES, ENABLED, unique_name);
		return SendMessage(client_socket, response, response_length);
	}
	
	if(!contains_module)
		control->InsertModule(unique_name, creation_delay_ms);
	
	control->EnableModule(unique_name, call);

	printf("ev3control: enabled module: %s\n", unique_name.c_str());
	
	int response_length=EncodeModuleMessage(response, CONTROL_BUFFER_BYTES, ENABLED, unique_name);
	
	return SendMessage(client_socket, response, response_length);
}
bool ProcessMessageDISABLE(int client_socket, const char *payload, const control_header &header, char *response, Control *control)
{
	static control_attribute attributes[1];
	const static ControlAttributes expected[]={UNIQUE_NAME};

	if(!ParseControlMessage(header, payload, attributes, 1, expected))
	{
		fprintf(stderr, "ev3control: ignoring invalid command %d\n", header.command);
		return true;
	}
	
	string unique_name(GetControlAttributeString(attributes[0]));	
		
	printf("ev3control: request to disable %s\n", unique_name.c_str());
				
	Module module;
	bool contains_module=control->ContainsModule(unique_name, &module);
	
	if( !contains_module)
	{
		fprintf(stderr, "ev3control: request to disable %s but no such module\n", unique_name.c_str());
		return true;
	}
	
	if(module.state != MODULE_ENABLED)
	{
		fprintf(stderr, "ev3control: request to disable %s module but is not enabled\n", unique_name.c_str());
		
		int response_length;
		if(module.state==MODULE_DISABLED)
			response_length=EncodeModuleMessage(response, CONTROL_BUFFER_BYTES, DISABLED, unique_name);
		else //if(module.state==MODULE_FAILED)
			response_length=EncodeFailedMessage(response, CONTROL_BUFFER_BYTES, unique_name, module.return_value);
		
		return SendMessage(client_socket, response, response_length);
	}
	
	control->DisableModule(unique_name);
	printf("ev3control: disabled module: %s\n", unique_name.c_str());
	
	int response_length=EncodeModuleMessage(response, CONTROL_BUFFER_BYTES, DISABLED, unique_name);
	return SendMessage(client_socket, response, response_length);
}
bool ProcessMessageDISABLE_ALL(int client_socket, const char *payload, const control_header &header, char *response, Control *control)
{
	printf("ev3control: request to disable all modules\n");

	list<string> disabled=control->DisableModules();
	
	for(list<string>::iterator it=disabled.begin();it!=disabled.end();++it)
	{
		printf("ev3control: disabled module: %s\n", it->c_str());
		int response_length=EncodeModuleMessage(response, CONTROL_BUFFER_BYTES, DISABLED, *it);
		if(!SendMessage(client_socket, response, response_length))
			return false;
	}
	return true;
}

bool CheckModulesStates(int client_socket, char *response, Control *control)
{
	list<FailedModule> failed=control->CheckModulesStates();
	
	for(list<FailedModule>::iterator it=failed.begin();it!=failed.end();++it)
	{
		printf("ev3control: %s failed with status %d\n", it->name.c_str(), it->status);
		int response_length=EncodeFailedMessage(response, CONTROL_BUFFER_BYTES, it->name, it->status);
		if(!SendMessage(client_socket, response, response_length))
			return false;
	}	
	if(failed.size() == 0)
	{
		int response_length=EncodeKeepaliveMessage(response, CONTROL_BUFFER_BYTES);
		return SendMessage(client_socket, response, response_length);
	}
	return true;
}

bool SendMessage(int sock, char *msg, int msg_len)
{
	int data_sent=0;
	int data_written=0;
	while(data_sent<msg_len)
	{
		if( ( data_written=send(sock , msg+data_sent , msg_len-data_sent, 0) ) == -1 )
		{
			perror("ev3control: socket send failed");
			return false;
		}
		data_sent+=data_written;
	}
	return true;
}

int EncodeModuleMessage(char *buffer, int buffer_length, ControlCommands command, const std::string &module_name)
{
	if( !PutControlHeader(buffer, buffer_length, TimestampUs(), command) 
	|| !PutControlAttributeString(buffer, buffer_length, UNIQUE_NAME, module_name.c_str()) )
		Die("ev3control: unable to encode module message\n");
	
	return GetControlMessageLength(buffer);
}
int EncodeFailedMessage(char *buffer, int buffer_length, const std::string &module_name, int32_t status)
{
	if(!EncodeModuleMessage(buffer, buffer_length, FAILED, module_name) 
	|| !PutControlAttributeI32(buffer, buffer_length, RETURN_VALUE, status))
		Die("ev3control: unable to encode module failed message\n");

	return GetControlMessageLength(buffer);
}

int EncodeKeepaliveMessage(char *buffer, int buffer_length)
{
	if( !PutControlHeader(buffer, buffer_length, TimestampUs(), KEEPALIVE) )
		Die("ev3control: unable to encode keepalive message\n");
	return GetControlMessageLength(buffer);
}

void Usage()
{
	printf("ev3control tcp_port timeout_ms\n\n");
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

void IgnoreSIGPIPE()
{
	if( signal(SIGPIPE, SIG_IGN) == SIG_ERR )
		DieErrno("ev3control: unable to ignore SIGPIPE");
}