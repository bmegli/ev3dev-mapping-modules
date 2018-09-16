/*
 * rplidar program
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
  * This program was created for... TO DO
  * 
  * rplidar:
  * -initializes RPLidar A3
  * -reads lidar data
  * -timestamps the data
  * -sends the above data in UDP messages
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "shared/misc.h"
#include "shared/net_udp.h"

#include <stdlib.h> //strtol, Slamtec rplidar.h depends on some not included defs...
#include "rplidar.h"

#include <limits.h> //INT_MAX
#include <stdio.h> //printf
#include <signal.h> //sigaction
#include <string.h> //memmcpy
#include <endian.h> //htobe16, htobe32, htobe64

using namespace rp::standalone::rplidar;

const int LIDAR_READINGS_PER_PACKET=192;

struct lidar_reading
{
	uint16_t angle_z_q14;
	uint16_t distance_mm;
};

struct lidar_packet
{
	uint64_t timestamp_us; 
	uint16_t sample_us; //microseconds per sample
	uint16_t readings_count;
	lidar_reading readings[LIDAR_READINGS_PER_PACKET];
};

const int LIDAR_PACKET_BYTES = 8 + 2 + 2 + 4 * LIDAR_READINGS_PER_PACKET;

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program=0;


void MainLoop(int socket_udp, const struct sockaddr_in &address, RPlidarDriver *lidar, const RplidarScanMode &lidar_mode);
int ProcessScan(lidar_packet *packet, rplidar_response_measurement_node_hq_t *nodes, int count);

void RegisterSignals();
void Finish(int signal);

RPlidarDriver *InitLidar(const char *tty_device, int baudrate, RplidarScanMode *mode);
void CloseLidar(RPlidarDriver *drv);

int EncodeLidarReading(const lidar_reading *reading, char *data);
int EncodeLidarPacket(const lidar_packet &p, char *data);
void SendLidarPacket(int socket_udp, const sockaddr_in &dst, const lidar_packet &packet);

int ProcessInput(int argc, char **argv, int *port);
void Usage();

int main(int argc, char **argv)
{
	int socket_udp, port;
	struct sockaddr_in address_udp;
	RPlidarDriver *lidar;
	RplidarScanMode lidar_mode;
	
	if( ProcessInput(argc, argv, &port) )
	{
		Usage();
		return 0;
	}
	
	SetStandardInputNonBlocking();
	
	const char *lidar_tty=argv[1]; //e.g "/dev/ttyUSB0"
	const char *host=argv[2];
	
	RegisterSignals(Finish);
	InitNetworkUDP(&socket_udp, &address_udp, host, port, 0);
	lidar=InitLidar(lidar_tty, 256000, &lidar_mode);
	
	if(!lidar)
	{
		fprintf(stderr, "rplidar: InitLidar failed\n");
		g_finish_program=1;
	}
 

	MainLoop(socket_udp, address_udp, lidar, lidar_mode);

	CloseLidar(lidar);
	CloseNetworkUDP(socket_udp);

	printf("rplidar: bye\n");

	return 0;	
}

void MainLoop(int socket_udp, const struct sockaddr_in &address, RPlidarDriver *lidar, const RplidarScanMode &lidar_mode)
{
	rplidar_response_measurement_node_hq_t nodes[RPlidarDriver::MAX_SCAN_NODES];
	lidar_packet packet;
	uint64_t timestamp_us;
	int counter, benchs=INT_MAX;
	u_result res;
	
	packet.sample_us=lidar_mode.us_per_sample;
	
	uint64_t start=TimestampUs();	
		
	for(counter=0;!g_finish_program && counter<benchs;++counter)
	{
		size_t count = RPlidarDriver::MAX_SCAN_NODES;
		size_t processed=0;
		
		//possbly adjust sleep
		SleepUs(lidar_mode.us_per_sample*LIDAR_READINGS_PER_PACKET);
		
		res = lidar->getScanDataWithIntervalHq(nodes, count);
		timestamp_us=TimestampUs()-count*lidar_mode.us_per_sample;

		if(IS_OK(res))
		{
			lidar->ascendScanData(nodes, count);
			while(processed<count)
			{
				packet.timestamp_us=timestamp_us+processed*lidar_mode.us_per_sample;
				processed += ProcessScan(&packet, nodes+processed, count-processed);
				SendLidarPacket(socket_udp, address, packet);
			}
		}
		else
			fprintf(stderr, "rplidar: failed to get data...\n");
		
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;
	}
	
	uint64_t end=TimestampUs();
	double seconds_elapsed=(end-start)/ 1000000.0L;
	
	printf("rplidar: avg loop %f seconds\n", seconds_elapsed/counter);
}

int ProcessScan(lidar_packet *packet, rplidar_response_measurement_node_hq_t *nodes, int count)
{	
	if(count>LIDAR_READINGS_PER_PACKET)
		count=LIDAR_READINGS_PER_PACKET;
	
	packet->readings_count=count;
	lidar_reading *readings=packet->readings;
	
	for(int i=0;i<count;++i)
	{
		readings[i].angle_z_q14=nodes[i].angle_z_q14;
		readings[i].distance_mm=nodes[i].dist_mm_q2 / (1 << 2);
	}
	
	return count;
}

void Finish(int signal)
{
	g_finish_program=1;
}

RPlidarDriver *InitLidar(const char *tty_device, int baudrate, RplidarScanMode *mode)
{
	RPlidarDriver *drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	rplidar_response_device_info_t devinfo;
	rplidar_response_device_health_t healthinfo;

	std::vector<RplidarScanMode> scanModes;
	u_result res;

	if (!drv)
	{
        fprintf(stderr, "rplidar: insufficent memory\n");
        return NULL;
    }

	if (IS_OK(drv->connect(tty_device, baudrate)))
	{
		res = drv->getDeviceInfo(devinfo);

		if (!IS_OK(res)) 
		{
			fprintf(stderr, "rplidar: cannot bind to the specified serial port %s\n", tty_device);
			RPlidarDriver::DisposeDriver(drv);
			return NULL;
		}
	}
	
    res = drv->getHealth(healthinfo);
    if (IS_OK(res))
	{ 
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
		{
            fprintf(stderr, "rplidar: internal error detected, reboot the device to retry.\n");
			RPlidarDriver::DisposeDriver(drv);
			return NULL;
		}
    }
	else
	{
        fprintf(stderr, "rplidar: cannot retrieve the lidar health code: %x\n", res);
		RPlidarDriver::DisposeDriver(drv);
		return NULL;
    }

	
	//temp?
	drv->getAllSupportedScanModes(scanModes);
	
	for(std::vector<RplidarScanMode>::const_iterator it=scanModes.begin();it!=scanModes.end();++it)
		printf("Scan mode id=%d %s, dist=%f us_per_sample=%f ans %d\n",it->id, it->scan_mode, it->max_distance, it->us_per_sample, it->ans_type);
	//end temp
	
	drv->startMotor();
	
    res = drv->startScanExpress(0, 3, 0, mode);

	if(!IS_OK(res))
	{
		fprintf(stderr, "rplidar: failed to start scan\n");
		CloseLidar(drv);
		return NULL;
	}
	else
		printf("rplidar: scan mode id=%d %s, dist=%f us_per_sample=%f\n",mode->id, mode->scan_mode, mode->max_distance, mode->us_per_sample);

	return drv;
}

void CloseLidar(RPlidarDriver *drv)
{
    if(!drv)
        return;
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
}
 
int EncodeLidarReading(const lidar_reading *reading, char *data)
{
	uint16_t u16=htobe16(reading->angle_z_q14);
	memcpy(data, &u16, sizeof(u16));
	data+=sizeof(u16);
	
	u16=htobe16(reading->distance_mm);
	memcpy(data, &u16, sizeof(u16));
	data+=sizeof(u16);
		
	return 2*sizeof(u16);
}

int EncodeLidarPacket(const lidar_packet &p, char *data)
{	
	uint64_t u64=htobe64(p.timestamp_us);
	memcpy(data, &u64, sizeof(u64));
	data += sizeof(u64);

	uint16_t u16=htobe16(p.sample_us);
	memcpy(data, &u16, sizeof(u16));
	data += sizeof(u16);
	
	u16=htobe16(p.readings_count);
	memcpy(data, &u16, sizeof(u16));
	data += sizeof(u16);
	
	for(int i=0;i<p.readings_count; ++i)
		data += EncodeLidarReading(p.readings+i, data);
		
	return LIDAR_PACKET_BYTES;  	
}

void SendLidarPacket(int socket_udp, const sockaddr_in &dst, const lidar_packet &packet)
{
	static char buffer[LIDAR_PACKET_BYTES];
	EncodeLidarPacket(packet, buffer);
	SendToUDP(socket_udp, dst, buffer, LIDAR_PACKET_BYTES);
}

int ProcessInput(int argc, char **argv, int *out_port)
{
	long int port;
				
	if(argc!=4)
		return -1;
		
	port=strtol(argv[3], NULL, 0);
	if(port <= 0 || port > 65535)
	{
		fprintf(stderr, "rplidar: the argument port has to be in range <1, 65535>\n");
		return -1;
	}
	*out_port=port;
		
	return 0;
}

void Usage()
{
	printf("rplidar lidar_tty host port \n\n");
	printf("examples:\n");
	printf(" ./rplidar /dev/ttyUSB0 192.168.1.80 8022\n");
}
