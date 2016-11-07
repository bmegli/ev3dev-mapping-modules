/*
 * ev3wifi program
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
  * ev3wifi:
  * -reads associated WiFi station info (bssid, ssid, signal strength, rx/tx packets)
  * -timestamps the data
  * -sends the above data in UDP messages
  *
  * Preconditions (for EV3/ev3dev):
  * - libmnl has to be installed
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "wifi-scan/wifi_scan.h"

#include "shared/misc.h"
#include "shared/net_udp.h"

#include <limits.h> //INT_MAX
#include <stdlib.h>
#include <stdio.h>
#include <string.h> //memcpy
#include <unistd.h> //open, close, read, write
#include <endian.h> //htobe16, htobe32, htobe64

struct wifi_packet
{
	uint64_t timestamp_us;
	uint8_t bssid[BSSID_LENGTH]; //this is hardware mac address of your AP
	char ssid[SSID_MAX_LENGTH_WITH_NULL]; //this is the name of your AP as you see it when connecting
	int8_t signal_dbm;  //signal strength in dBm from last received PPDU, you may need to average that
	uint32_t rx_packets; //the number of received packets
	uint32_t tx_packets; //the number of transmitted packets
};

const int WIFI_PACKET_BYTES=56; // 8 + 6 + 33 + 1 + 4 + 4

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, wifi_scan *wifi, int poll_ms);

int EncodeWifiPacket(const wifi_packet &packet, char *buffer);
void SendWifiPacketUDP(int socket, const sockaddr_in &dest, const wifi_packet &packet);

void Usage();
int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms);

int main(int argc, char **argv)
{
	int socket_udp, port, poll_ms;
	sockaddr_in destination_udp;
	wifi_scan *wifi=NULL;

	if( ProcessInput(argc, argv, &port, &poll_ms) )
	{
		Usage();
		return 0;
	}
	
	const char *host=argv[1];
	const char *wireless_interface=argv[3]; 

	wifi=wifi_scan_init(wireless_interface);
	if(!wifi)
		Die("Unable to initialize wifi-scan library");

	SetStandardInputNonBlocking();	

	InitNetworkUDP(&socket_udp, &destination_udp, host, port, 0);
			
	MainLoop(socket_udp, destination_udp, wifi, poll_ms);
	
	CloseNetworkUDP(socket_udp);
	
	wifi_scan_close(wifi);

	printf("ev3wifi: bye\n");
	
	return 0;
}

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, wifi_scan *wifi, int poll_ms)
{
	const int BENCHS=INT_MAX;
		
	struct station_info station;	
	struct wifi_packet packet;
	uint64_t start=TimestampUs();
	int i, elapsed_us, poll_us=1000*poll_ms;
	
	for(i=0;i<BENCHS;++i)
	{	
		packet.timestamp_us=TimestampUs();
		if(wifi_scan_station(wifi, &station)>0)
		{
			memcpy(packet.bssid, station.bssid, BSSID_LENGTH);
			memcpy(packet.ssid, station.ssid, SSID_MAX_LENGTH_WITH_NULL);
			packet.signal_dbm=station.signal_dbm;
			packet.rx_packets=station.rx_packets;
			packet.tx_packets=station.tx_packets;
		
			SendWifiPacketUDP(socket_udp, destination_udp, packet);
		}
		else
			fprintf(stderr, "ev3wifi: no associated station\n");
			
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;

		elapsed_us=(int)(TimestampUs()-packet.timestamp_us);
		
		if( elapsed_us < poll_us )
			SleepUs(poll_us - elapsed_us);
	}
		
	uint64_t end=TimestampUs();
	
	double seconds_elapsed=(end-start)/ 1000000.0L;
	printf("ev3wifi: average loop %f seconds\n", seconds_elapsed/i);
}

int EncodeWifiPacket(const wifi_packet &p, char *data)
{
	*((uint64_t*)data) = htobe64(p.timestamp_us);
	data += sizeof(p.timestamp_us);

	memcpy(data, p.bssid, BSSID_LENGTH);
	data += BSSID_LENGTH;

	memcpy(data, p.ssid, SSID_MAX_LENGTH_WITH_NULL);
	data += SSID_MAX_LENGTH_WITH_NULL;

	*data=p.signal_dbm;
	data += 1;

	*((uint32_t*)data)= htobe32(p.rx_packets);
	data += sizeof(p.rx_packets);

	*((uint32_t*)data)= htobe32(p.tx_packets);
	data += sizeof(p.tx_packets);
	
	return WIFI_PACKET_BYTES;	
}
void SendWifiPacketUDP(int socket, const sockaddr_in &destination, const wifi_packet &packet)
{
	static char buffer[WIFI_PACKET_BYTES];
	EncodeWifiPacket(packet, buffer);
	SendToUDP(socket, destination, buffer, WIFI_PACKET_BYTES);
}

void Usage()
{
	printf("ev3wifi host port wireless_interface poll_ms\n\n");
	printf("examples:\n");
	printf("./ev3wifi 192.168.0.103 8006 wlan0 100\n");
}

int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms)
{
	long int port, poll;
		
	if(argc!=5)
		return -1;
		
	port=strtol(argv[2], NULL, 0);
	if(port <= 0 || port > 65535)
	{
		fprintf(stderr, "ev3wifi: the argument port has to be in range <1, 65535>\n");
		return -1;
	}
	*out_port=port;
	
	poll=strtol(argv[4], NULL, 0);
	if(poll <= 0)
	{
		fprintf(stderr, "ev3wifi: the argument poll_ms has to be greater than 0\n");
		return -1;
	}
	*out_poll_ms=poll;

	
	return 0;
}