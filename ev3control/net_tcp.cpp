/*
 * TCP/IP networking implementation file
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

#include "net_tcp.h"

#include "shared/misc.h"

#include <netinet/in.h> //socaddr_in
#include <sys/types.h>  //for historical portabilty
#include <sys/socket.h> //socket
#include <sys/select.h> //select

#include <errno.h> //errno
#include <string.h> //memset
#include <unistd.h> //fcntl
#include <fcntl.h> //fcntl

// Note - all the sockets are created with O_CLOEXEC (or SOCK_CLOEXEC) flags
// so that they are not inherited by child processes

void InitNetworkTCP(int *sock, short port)
{
	struct sockaddr_in servaddr;
	int flags;
	
    if( (*sock = socket(AF_INET , SOCK_STREAM , 0) ) == -1)
		DieErrno("InitNetworkTCP socket");
		
	if( (flags = fcntl(*sock, F_GETFL, 0)) == -1)
		DieErrno("InitNetworkTCP fcntl F_GETFL");
	
	//we don't want to the spawned children to inherit this descriptor
	if( fcntl(*sock, F_SETFL, flags | O_CLOEXEC | O_NONBLOCK) == -1)
		DieErrno("InitNetworkTCP fntnl F_SETFL");
			
	memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_addr.s_addr =  INADDR_ANY;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons( port );
 
	if ( bind(*sock, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) 
              DieErrno("InitNetworkTCP bind");  
			  
	if ( listen(*sock,1) == -1 )
		DieErrno("InitNetworkTCP listen");
}

bool WaitForClientTCP(int serv_sock, int timeout_ms, int *client_sock)
{
	struct timeval tv;
	struct sockaddr_in clientaddr;

	socklen_t clientaddr_length=sizeof(clientaddr);
	fd_set rfds;

    int ret;

    FD_ZERO(&rfds);
    FD_SET(serv_sock, &rfds);

	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms-(timeout_ms/1000)*1000) * 1000;		
				
    ret = select(serv_sock+1, &rfds, NULL, NULL, &tv);

	if(ret == -1)
	{
		if(errno == EINTR)
			return false;
		DieErrno("WaitForClientTCP select");
	}
	else if(ret==0)
		return false;
		
	//otherwise select returned postive value
   
	*client_sock = accept4(serv_sock, (struct sockaddr *) &clientaddr, &clientaddr_length, SOCK_CLOEXEC);
	
	if(*client_sock == -1)
	{   //the peer gave up connection between select and accept, just return timetout
		if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
			return false;
		DieErrno("WaitForClientTCP accept4");
	}
	
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms-(timeout_ms/1000)*1000) * 1000;		
				
	if (setsockopt(*client_sock, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) == -1)
		DieErrno("WaitForClientTCP setsockopt");				
	
	return true;
}

void CloseNetworkTCP(int serv_sock)
{
	if( close(serv_sock) == -1 )
		DieErrno("CloseNetworkTCP close");
}