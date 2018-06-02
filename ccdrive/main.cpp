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
  * -initializes 3 x roboclaw MC and vmu931 IMU
  * -reads UDP messages with motor settings
  * -sets motor speeds/positions accordingly
  * -stops motors on timeout
  * -reads motors' encoder positions
  * -reads IMU data
  * -sends motor encoders and IMU data in UDP messages
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "roboclaw/roboclaw.h"
#include "vmu931/vmu931.h"

#include "shared/misc.h"
#include "shared/net_udp.h"

#include <sys/select.h> //select

#include <string.h> //memcpy
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

struct dead_reconning_packet
{
	uint64_t timestamp_us;
	int32_t position_left;
	int32_t position_right;
	//quaternion as in Unity coordinate system
	float w;
	float x;
	float y;
	float z;
};

const int DEAD_RECONNING_PACKET_BYTES=32; //8 + 2*4 + 4*4

const int CONTROL_PACKET_BYTES = 18; //8 + 5*2 = 18 bytes
enum Commands {KEEPALIVE=0, SET_SPEED=1, TO_POSITION_WITH_SPEED=2};

void MainLoop(int server_udp, int client_udp, const sockaddr_in &destination_udp, roboclaw *rc, vmu *vmu);
void ProcessMessage(const drive_packet &packet, roboclaw *rc);
int GetEncoders(roboclaw *rc, dead_reconning_packet *packet);
int GetEulerAngles(vmu *vmu, dead_reconning_packet *packet);
int GetQuaternion(vmu *vmu, dead_reconning_packet *packet);


struct roboclaw *InitMotors(const char *tty, int baudrate);
void CloseMotors(roboclaw *rc);
void StopMotors(roboclaw *rc);

struct vmu *InitIMU(const char *tty);
void CloseIMU(struct vmu *vmu931);

int RecvDrivePacket(int socket_udp, drive_packet *packet);
void DecodeDrivePacket(drive_packet *packet, const char *data);

int EncodeDeadReconningPacket(const dead_reconning_packet &packet, char *buffer);
void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const dead_reconning_packet &frame);

void Usage();
void ProcessArguments(int argc, char **argv,int *server_port, int *port, int *timeout_ms);
void Finish(int signal);


int main(int argc, char **argv)
{			
	int server_udp, client_udp, server_port, port, timeout_ms;
	sockaddr_in destination_udp;

	struct roboclaw *rc;
	struct vmu *vmu;
	
	ProcessArguments(argc, argv,&server_port, &port, &timeout_ms);
	const char *roboclaw_tty=argv[1];
	const char *vmu931_tty=argv[3];
	const char *host=argv[4];

	SetStandardInputNonBlocking();

	//init
	RegisterSignals(Finish);

	rc=InitMotors(roboclaw_tty, 460800); //TO DO hardcoded baudrate
	vmu=InitIMU(vmu931_tty);
	
	if(rc==NULL || vmu== NULL)
	{
		CloseMotors(rc), CloseIMU(vmu);
		return 1;
	}
			
	InitNetworkUDP(&server_udp, NULL, NULL, server_port, timeout_ms); //to do - no timeout needed here, we timeout on select
	InitNetworkUDP(&client_udp, &destination_udp, host, port, 0);

	//work
	MainLoop(server_udp, client_udp, destination_udp, rc, vmu);
	
	//cleanup
	StopMotors(rc);
	CloseMotors(rc);
	CloseIMU(vmu);
	CloseNetworkUDP(client_udp);
	CloseNetworkUDP(server_udp);
		
	printf("ccdrive: bye\n");
		
	return 0;
}

void MainLoop(int server_udp, int client_udp, const sockaddr_in &destination_udp, roboclaw *rc, vmu *vmu)
{
	int status;	
	fd_set rfds;
	struct timeval tv;

	struct dead_reconning_packet odometry_packet;
	struct drive_packet drive_packet;
	uint64_t last_drive_packet_timestamp_us=TimestampUs();

	uint64_t TIMEOUT_US=500*1000;
		
	while(!g_finish_program)
	{
		FD_ZERO(&rfds);
		FD_SET(server_udp, &rfds);
		tv.tv_sec=0;
		tv.tv_usec=1000*10; //10 ms hardcoded 

		status=select(server_udp+1, &rfds, NULL, NULL, &tv);
		if(status == -1)
		{
			perror("ccdrive: select failed");
			break;
		}
		else if(status) //got something on udp socket
		{
			status=RecvDrivePacket(server_udp, &drive_packet);
			if(status < 0)
				break;
			if(status == 0) //timeout but this should not happen, we are after select
			{
				StopMotors(rc);
				fprintf(stderr, "ccdrive: waiting for drive controller...\n");
			}
			else
			{
				ProcessMessage(drive_packet, rc);
				last_drive_packet_timestamp_us=TimestampUs();
			}
		}
		
		//check timeout
		if(TimestampUs() - last_drive_packet_timestamp_us > TIMEOUT_US)
		{
			StopMotors(rc);
			last_drive_packet_timestamp_us=TimestampUs(); //mark timestamp not to flood with messages
			fprintf(stderr, "ccdrive: timeout...\n");		
		}
		
		//fill the dead reconning data (encoders and euler angles)
		
		if(GetEncoders(rc, &odometry_packet) == -1)
			break;
		
		odometry_packet.timestamp_us = TimestampUs();

		if(GetQuaternion(vmu, &odometry_packet) == -1)
			break; //consider if it is possible to not get data

		SendDeadReconningPacketUDP(client_udp, destination_udp, odometry_packet);
		//printf("l=%d r=%d x=%f y=%f z=%f\n", odometry_packet.position_left, odometry_packet.position_right, odometry_packet.x, odometry_packet.y, odometry_packet.z);
			
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

int GetEncoders(roboclaw *rc, dead_reconning_packet *packet)
{
	int status;
	status=roboclaw_encoders(rc, MIDDLE_MOTOR_ADDRESS, &packet->position_left, &packet->position_right);

	if(status == ROBOCLAW_OK)
		return 0;

	if(status == ROBOCLAW_ERROR)
		perror("ccdrive: unable to read encoders\n");
	if(status == ROBOCLAW_RETRIES_EXCEEDED)
		fprintf(stderr, "ccdrive: retries exceeded while reading encoders\n");
	return -1;
}

int GetEulerAngles(vmu *vmu, dead_reconning_packet *packet)
{
	static vmu_txyz euler_data[10];
	int status;
	
	while( (status=vmu_euler(vmu, euler_data, 10)) > 10 )
		; //burn through old readings to get the lastest
	
	if(status == VMU_ERROR)
	{
		perror("ccdrive: failed to read imu data\n");
		return -1;
	}
	if(status == 0) //this should not happen
	{
		fprintf(stderr, "ccdrive: status 0 WTF?");
		return -1;
	}	

	//the last reading is the latest
	--status; 

	//euler angles as in Unity coordinate system
	packet->x = euler_data[status].x;
	packet->y = -euler_data[status].z;
	packet->z = euler_data[status].y;
	
	return 0;
}

int GetQuaternion(vmu *vmu, dead_reconning_packet *packet)
{
	static vmu_twxyz quat_data[10];
	int status;
	
	while( (status=vmu_quat(vmu, quat_data, 10)) > 10 )
		; //burn through old readings to get the lastest
	
	if(status == VMU_ERROR)
	{
		perror("ccdrive: failed to read imu data\n");
		return -1;
	}
	if(status == 0) //this should not happen
	{
		fprintf(stderr, "ccdrive: status 0 WTF?");
		return -1;
	}	

	//the last reading is the latest
	--status; 

	//quaterion as in Unity coordinate system
	packet->w = quat_data[status].w;
	packet->x = quat_data[status].x;
	packet->y = -quat_data[status].z;
	packet->z = quat_data[status].y;
	
	return 0;
}


struct roboclaw *InitMotors(const char *tty, int baudrate)
{
	roboclaw *rc;
	int16_t voltage;
	bool ok=true;
	
	rc=roboclaw_init(tty, baudrate);
	
	if(!rc)
	{
		perror("ccdrive: unable to initialize motors\n");
		return NULL;
	}
	
	ok &= roboclaw_main_battery_voltage(rc, FRONT_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(rc, MIDDLE_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(rc, REAR_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	
	if(!ok)
	{
		CloseMotors(rc); //TO DO add more informative message
		fprintf(stderr, "ccdrive: unable to communicate with motors\n");
		return NULL;
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

struct vmu *InitIMU(const char *tty)
{
	struct vmu *vmu;
	
	if( (vmu=vmu_init(tty)) == NULL)
	{
		perror(	"ccdrive: unable to initialize VMU931\n\n"
				"hints:\n"
				"- it takes a few seconds after plugging in to initialize device\n"
				"- make sure you are using correct tty device (dmesg after plugging vmu)\n"
				"- if all else fails unplug/plug VMU931\n\n"
				"error details");
		return NULL;
	} 
	
	if( vmu_stream(vmu, VMU_STREAM_QUAT) == VMU_ERROR )
	{
		perror("ccdrive: vmu failed to stream quaternion data\n");
		vmu_close(vmu);
		return NULL;
	}
	
	return vmu;
}

void CloseIMU(struct vmu *vmu931)
{
	vmu_close(vmu931);
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

int encode_float(float f, char *buffer)
{
	uint32_t t32;
	memcpy(&t32, &f, sizeof(float));
	t32=htobe32(t32);
	memcpy(buffer, &t32, sizeof(float));
	return sizeof(float);
}
int encode_int32(int32_t i, char *buffer)
{
	uint32_t t32=htobe32(i);
	memcpy(buffer, &t32, sizeof(int32_t));
	return sizeof(int32_t);
}

int EncodeDeadReconningPacket(const dead_reconning_packet &p, char *buffer)
{
	uint64_t temp64;
	size_t offset=0;
	
	temp64=htobe64(p.timestamp_us);
	memcpy(buffer, &temp64, sizeof(uint64_t));
	offset+=sizeof(uint64_t);

	offset+=encode_int32(p.position_left, buffer+offset);
	offset+=encode_int32(p.position_right, buffer+offset);

	offset += encode_float(p.w, buffer+offset);
	offset += encode_float(p.x, buffer+offset);
	offset += encode_float(p.y, buffer+offset);
	offset += encode_float(p.z, buffer+offset);	
	
	return DEAD_RECONNING_PACKET_BYTES;	
	
}
void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const dead_reconning_packet &frame)
{
	static char buffer[DEAD_RECONNING_PACKET_BYTES];
	EncodeDeadReconningPacket(frame, buffer);
	SendToUDP(socket, dest, buffer, DEAD_RECONNING_PACKET_BYTES);
}

void Usage()
{
	printf("ccdrive roboclaw_tty server_port vmu_tty host port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ccdrive /dev/ttyO1 8003 /dev/ttyACM0 192.168.1.80 8013 500\n");
}
void ProcessArguments(int argc, char **argv,int *server_port, int *port, int *timeout_ms)
{
	if(argc!=7)
	{
		Usage();
		exit(EXIT_SUCCESS);		
	}
	
	long temp;
	
	temp=strtol(argv[2], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccdrive: the argument server_port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*server_port=temp;

	temp=strtol(argv[5], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccdrive: the argument server_port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*port=temp;	
	
	temp=strtol(argv[6], NULL, 0);
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
