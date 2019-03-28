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
//#include "rplidar.h"

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

void main_loop(int odometry_socket, const sockaddr_in &odometry_dst,int rplidar_socket, const sockaddr_in &rplidar_dst, struct cc *c);

int EncodeDeadReconningPacket(const cc_odometry_data &packet, char *buffer);
void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const cc_odometry_data &frame);

void process_arguments(int argc, char **argv, int *odometry_port, int *lidar_port);
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

enum {RPLIDAR_READINGS_PER_PACKET = 32*3};

struct rplidar_reading
{
	uint16_t	angle_q14;
	uint16_t distance_mm;
	//uint32_t	dist_mm_q2; 
};	


struct rplidar_packet
{
	uint64_t timestamp_us; //!< microseconds elapsed since MCU was plugged in
	uint16_t sample_us; //microseconds per sample
	struct rplidar_reading readings[RPLIDAR_READINGS_PER_PACKET];
};

const int RPLIDAR_PACKET_BYTES = 8 + 2 + 4 * RPLIDAR_READINGS_PER_PACKET;


int EncodeLidarReading(const rplidar_reading *reading, char *data);
int EncodeLidarPacket(const rplidar_packet &p, char *data);
void SendLidarPacket(int socket_udp, const sockaddr_in &dst, const rplidar_packet &packet);

void rplidar_decode(const rplidar_response_ultra_capsule_measurement_nodes_t *capsule,
	const rplidar_response_ultra_capsule_measurement_nodes_t *prev_capsule,
	struct rplidar_reading *nodebuffer);



const int DEAD_RECONNING_PACKET_BYTES=32; //8 + 2*4 + 4*4

int main(int argc, char **argv)
{
	struct cc *c = NULL;
	int odometry_socket, odometry_port;
	sockaddr_in odometry_destination;
	int lidar_socket, lidar_port;
	sockaddr_in lidar_destination;


	//input
	process_arguments(argc, argv, &odometry_port, &lidar_port);
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
	InitNetworkUDP(&lidar_socket, &lidar_destination, host, lidar_port, 0);

	main_loop(odometry_socket, odometry_destination, lidar_socket, lidar_destination, c);
	
	CloseNetworkUDP(lidar_socket);
	CloseNetworkUDP(odometry_socket);
			
	cc_close(c);
	
	printf("ccmcu: bye\n");
	
	return 0;
}

void main_loop(int odometry_socket, const sockaddr_in &odometry_dst,int rplidar_socket, const sockaddr_in &rplidar_dst, struct cc *c)
{
	struct cc_odometry_data odometry[DATA_SIZE];
	struct cc_rplidar_data rplidar[DATA_SIZE];
	struct cc_xv11lidar_data xv11lidar[DATA_SIZE];

	struct cc_rplidar_data rplidar_prev;
	struct rplidar_packet rp_data;
	rp_data.sample_us = 63; // 1/16000(Hz) * 1000000 (us), temp hardcoded


	struct cc_data data = {0};
	struct cc_size size = {0}; 

	size.odometry = size.rplidar = size.xv11lidar = DATA_SIZE;

	data.odometry = odometry;
	data.rplidar = rplidar;
	data.xv11lidar = xv11lidar;
	data.size = size;
	
	int ret=0;
	
	while( !g_finish_program && ( ret = cc_read_all(c, &data) ) != CC_ERROR)
	{		
		for(int i=0;i<data.size.odometry;++i)
			SendDeadReconningPacketUDP(odometry_socket, odometry_dst, odometry[i]);

		for(int i=0;i<data.size.rplidar;++i)
		{
			if(rplidar_prev.sequence + 1 == data.rplidar[i].sequence)
			{
				rplidar_decode(&data.rplidar[i].capsule, &rplidar_prev.capsule, rp_data.readings);
				rp_data.timestamp_us = rplidar_prev.timestamp_us;
				SendLidarPacket(rplidar_socket, rplidar_dst, rp_data);
				
	//			float angle_deg=rp_data.readings[0].angle_q14 * 90.0f / (1 << 14);
	//			int distance=rp_data.readings[0].distance_mm;				
	//			printf("[rp first] t=%u ang=%f dist=%d\n",
	//			rp_data.timestamp_us, angle_deg, distance);
			}
			//temp ineeficient, use pointers
			memcpy(&rplidar_prev, &data.rplidar[i], sizeof(cc_rplidar_data));

			//printf("[rp ] t=%u seq=%d ang=%d\n",
			//data.rplidar[i].timestamp_us, data.rplidar[i].sequence, data.rplidar[i].capsule.start_angle_sync_q6/64);
			
		}

		for(int i=0;i<data.size.xv11lidar;++i)
			printf("[xv11] t=%u aq=%d s=%d d=?\n", data.xv11lidar[i].timestamp_us,
			data.xv11lidar[i].angle_quad, data.xv11lidar[i].speed64/64);
					
		//refresh the sizes of the arrays for data streams
		data.size=size;
		
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;		
	}
	
	if(!g_finish_program && ret != CC_OK)
		perror("ccmcu: error while reading from MCU\n");
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
	offset += encode_float(-p.qy, buffer+offset);
	offset += encode_float(p.qz, buffer+offset);	
	offset += encode_float(-p.qx, buffer+offset);

	return DEAD_RECONNING_PACKET_BYTES;	
}

void SendDeadReconningPacketUDP(int socket, const sockaddr_in &dest, const cc_odometry_data &frame)
{
	static char buffer[DEAD_RECONNING_PACKET_BYTES];
	EncodeDeadReconningPacket(frame, buffer);
	SendToUDP(socket, dest, buffer, DEAD_RECONNING_PACKET_BYTES);
}

void process_arguments(int argc, char **argv, int *odometry_port, int *lidar_port)
{
	//ccmcu tty_device host odometry_port, rplidar_port //xv11lidar port
	if(argc!=5)
	{
		usage(argv);
		exit(EXIT_SUCCESS);		
	}
	
	long temp;
	
	temp=strtol(argv[3], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccmcu: the argument odometry_port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*odometry_port=temp;

	temp=strtol(argv[4], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ccmcu: the argument lidar_port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*lidar_port=temp;

}
void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s tty_device host odometry_port lidar_port\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0 192.168.0.125 8013 8022\n", argv[0]);
}

void finish(int signal)
{
	g_finish_program=1;	
}

//RPLIDAR encoding (UDP)

int EncodeLidarReading(const rplidar_reading *reading, char *data)
{
	uint16_t u16=htobe16(reading->angle_q14);
	memcpy(data, &u16, sizeof(u16));
	data+=sizeof(u16); 
	
	u16=htobe16(reading->distance_mm);
	memcpy(data, &u16, sizeof(u16));
	data+=sizeof(u16);
		
	return 2*sizeof(u16);
}

int EncodeLidarPacket(const rplidar_packet &p, char *data)
{	
	uint64_t u64=htobe64(p.timestamp_us);
	memcpy(data, &u64, sizeof(u64));
	data += sizeof(u64);

	uint16_t u16=htobe16(p.sample_us);
	memcpy(data, &u16, sizeof(u16));
	data += sizeof(u16);
		
	for(int i=0;i<RPLIDAR_READINGS_PER_PACKET; ++i)
		data += EncodeLidarReading(p.readings+i, data);
		
	return RPLIDAR_PACKET_BYTES;  	
}

void SendLidarPacket(int socket_udp, const sockaddr_in &dst, const rplidar_packet &packet)
{
	static char buffer[RPLIDAR_PACKET_BYTES];
	EncodeLidarPacket(packet, buffer);
	SendToUDP(socket_udp, dst, buffer, RPLIDAR_PACKET_BYTES);
}


//RPLIDAR decoding (USB)

/*
typedef struct _rplidar_response_ultra_cabin_nodes_t {
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    uint32_t combined_x3;
} __attribute__((packed)) rplidar_response_ultra_cabin_nodes_t;  

typedef struct _rplidar_response_ultra_capsule_measurement_nodes_t {
    uint8_t                             s_checksum_1; // see [s_checksum_1]
    uint8_t                             s_checksum_2; // see [s_checksum_1]
    uint16_t                            start_angle_sync_q6;
    rplidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} __attribute__((packed)) rplidar_response_ultra_capsule_measurement_nodes_t;
*/

#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))

// Definition of the variable bit scale encoding mechanism
#define RPLIDAR_VARBITSCALE_X2_SRC_BIT  9
#define RPLIDAR_VARBITSCALE_X4_SRC_BIT  11
#define RPLIDAR_VARBITSCALE_X8_SRC_BIT  12
#define RPLIDAR_VARBITSCALE_X16_SRC_BIT 14

#define RPLIDAR_VARBITSCALE_X2_DEST_VAL 512
#define RPLIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define RPLIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define RPLIDAR_VARBITSCALE_X16_DEST_VAL 3328

uint32_t _varbitscale_decode(uint32_t scaled, uint32_t *scaleLevel)
{
    static const uint32_t VBS_SCALED_BASE[] = {
        RPLIDAR_VARBITSCALE_X16_DEST_VAL,
        RPLIDAR_VARBITSCALE_X8_DEST_VAL,
        RPLIDAR_VARBITSCALE_X4_DEST_VAL,
        RPLIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const uint32_t VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const uint32_t VBS_TARGET_BASE[] = {
        (0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i)
    {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            *scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << *scaleLevel);
        }
    }
    return 0;
}


#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2

//void RPlidarDriverImplCommon::_ultraCapsuleToNormal(const rplidar_response_ultra_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)

void rplidar_decode(const rplidar_response_ultra_capsule_measurement_nodes_t *capsule,
                    const rplidar_response_ultra_capsule_measurement_nodes_t *prev_capsule,
                    struct rplidar_reading *nodebuffer)
{
     int nodeCount = 0;
	  int diffAngle_q8;
	  int currentStartAngle_q8 = ((capsule->start_angle_sync_q6 & 0x7FFF) << 2);
	  int prevStartAngle_q8 = ((prev_capsule->start_angle_sync_q6 & 0x7FFF) << 2);

	  diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
	  if (prevStartAngle_q8 >  currentStartAngle_q8) {
			diffAngle_q8 += (360 << 8);
	  }

	  int angleInc_q16 = (diffAngle_q8 << 3) / 3;
	  int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
	  for (size_t pos = 0; pos < _countof(prev_capsule->ultra_cabins); ++pos)
	  {
			int dist_q2[3];
			int angle_q6[3];
			int syncBit[3];

			uint32_t combined_x3 = prev_capsule->ultra_cabins[pos].combined_x3;

			// unpack ...
			int dist_major = (combined_x3 & 0xFFF);

			// signed partical integer, using the magic shift here
			// DO NOT TOUCH

			int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
			int dist_predict2 = (((int)combined_x3) >> 22);

			int dist_major2;

			uint32_t scalelvl1, scalelvl2;

			// prefetch next ...
			if (pos == _countof(prev_capsule->ultra_cabins) - 1)
			{
				 dist_major2 = (capsule->ultra_cabins[0].combined_x3 & 0xFFF);
			}
			else {
				 dist_major2 = (prev_capsule->ultra_cabins[pos + 1].combined_x3 & 0xFFF);
			}

			// decode with the var bit scale ...
			dist_major = _varbitscale_decode(dist_major, &scalelvl1);
			dist_major2 = _varbitscale_decode(dist_major2, &scalelvl2);


			int dist_base1 = dist_major;
			int dist_base2 = dist_major2;

			if ((!dist_major) && dist_major2) {
				 dist_base1 = dist_major2;
				 scalelvl1 = scalelvl2;
			}

		  
			dist_q2[0] = (dist_major << 2);
			if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
				 dist_q2[1] = 0;
			} else {
				 dist_predict1 = (dist_predict1 << scalelvl1);
				 dist_q2[1] = (dist_predict1 + dist_base1) << 2;

			}

			if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
				 dist_q2[2] = 0;
			} else {
				 dist_predict2 = (dist_predict2 << scalelvl2);
				 dist_q2[2] = (dist_predict2 + dist_base2) << 2;
			}
		  
			for (int cpos = 0; cpos < 3; ++cpos)
			{

				 syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

				 int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

				 if (dist_q2[cpos] >= (50 * 4))
				 {
					  const int k1 = 98361;
					  const int k2 = (int)(k1 / dist_q2[cpos]);

					  offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
				 }

				 angle_q6[cpos] = ((currentAngle_raw_q16 - (int)(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
				 currentAngle_raw_q16 += angleInc_q16;

				 if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
				 if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

				 struct rplidar_reading node;

				 node.angle_q14 = (uint16_t)((angle_q6[cpos] << 8) / 90);
				 //loss of information - from 1/4 mm precission to 1 mm precision
				 node.distance_mm= dist_q2[cpos] / (1 << 2); //loss of information!
				 //node.dist_mm_q2 = dist_q2[cpos];
				 nodebuffer[nodeCount++] = node;
			}
	  }
}
