/*
 * cc-read-all example for cave-crawler-lib library
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

#include <stdio.h> //printf
#include <stdlib.h> //exit

void usage(char **argv);

const int DATA_SIZE=10; 
const int MAX_READS=1000;

int main(int argc, char **argv)
{
	struct cc *c = NULL;
	struct cc_odometry_data odometry[DATA_SIZE];
	struct cc_rplidar_data rplidar[DATA_SIZE];
	struct cc_xv11lidar_data xv11lidar[DATA_SIZE];

	struct cc_data data = {0}; //the library will return data here and note number of read values in data.size
	struct cc_size size = {0}; //this notes sizes of our arrays, data.size is refreshed with this value before read

	size.odometry = size.rplidar = size.xv11lidar = DATA_SIZE;

	data.odometry = odometry;
	data.rplidar = rplidar;
	data.xv11lidar = xv11lidar;
	data.size = size;
		
	const char *tty_device=argv[1];
	int ret, reads=0;
	
	if(argc != 2)
	{
		usage(argv);
		return EXIT_SUCCESS;
	}
	
	if( (c = cc_init(tty_device)) == NULL)
	{
		perror(	"unable to initialize cave-crawler communication\n\n"
				"hints:\n"
				"- it takes a few seconds after plugging in to initialize device\n"
				"- make sure you are using correct tty device (dmesg after plugging cave-crawler-mcu)\n"
				"- if all else fails unplug/plug cave-crawler-mcu\n\n"
				"error details");
		return 1;
	}	
			
	while( (ret = cc_read_all(c, &data)) != CC_ERROR )
	{
		for(int i=0;i<data.size.odometry;++i)
			printf("[odo] t=%u i=%d left=%d right=%d qw=%f qx=%f qy=%f qz=%f\n",
			data.odometry[i].timestamp_us, i, data.odometry[i].left_encoder_counts,
			data.odometry[i].right_encoder_counts, data.odometry[i].qw,
			data.odometry[i].qx, data.odometry[i].qy, data.odometry[i].qz);

		for(int i=0;i<data.size.rplidar;++i)
			printf("[rp ] t=%u seq=%d\n",
			data.rplidar[i].timestamp_us, data.rplidar[i].sequence);

		for(int i=0;i<data.size.xv11lidar;++i)
			printf("[xv11] t=%u aq=%d s=%d d=?\n", data.xv11lidar[i].timestamp_us,
			data.xv11lidar[i].angle_quad, data.xv11lidar[i].speed64/64);
		
		//terminate after reading MAX_READS times
		//remove those lines if you want to read infinitely
		if(++reads >= MAX_READS) 
			break;
			
		//refresh the sizes of the arrays for data streams
		data.size=size;
	}
		
	if(ret == CC_ERROR)
		perror("failed to read from cave-crawler mcu");
	else
		printf("success reading from cave-crawler mcu, bye...\n");
	
	cc_close(c);
	
	return 0;
}

void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s tty_device\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0\n", argv[0]);
}
