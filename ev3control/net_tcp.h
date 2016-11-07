/*
 * TCP/IP networking header file
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

#pragma once

// all created sockets have O_CLOEXEC flag set

void InitNetworkTCP(int *serv_sock, short port);

// prerequisities:
// - InitNetworkTCP called with serv_sock as argument
//
// timeout:
// - waiting for client timeouts after timetout_ms
// - the returned client socket has timeout_ms set for receiving
//
// returns:
// - false on timeout
// - true if client was accepted, then client_sock is its socket
bool WaitForClientTCP(int serv_sock, int timeout_ms, int *client_sock);

void CloseNetworkTCP(int serv_sock);