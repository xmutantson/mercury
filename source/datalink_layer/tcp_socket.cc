/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "datalink_layer/tcp_socket.h"

cl_tcp_socket::cl_tcp_socket()
{
	type=TYPE_SERVER;
	socket_fd=0;
	connection_fd=0;
	status=TCP_STATUS_CLOSED;
	address="";
	port=0;
	message_counter=0;
	server_sent_packets=0;
	server_received_packets=0;
	client_sent_packets=0;
	client_received_packets=0;
	memset(&server, 0, sizeof(server));
	memset(&client, 0, sizeof(client));
	allow_out_of_order_release=0;
	message= new st_tcp_message;
	timeout_ms=1000;
	link_buffer=NULL;
	buffer_occupancy=0;


}
cl_tcp_socket::~cl_tcp_socket()
{
	if(socket_fd>0)
	{
#if defined(_WIN32)
        closesocket(socket_fd);
        WSACleanup();
#else
		close(socket_fd);
#endif
	}
}


int cl_tcp_socket::init()
{
#if defined(_WIN32)
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2 ,2), &wsaData);
	if (iResult != 0)
    {
		printf("error at WSASturtup\n");
		return ERROR_;
	}
#endif

    int return_val=SUCCESS;
	if(status==TCP_STATUS_CLOSED)
	{
		if(type==TYPE_SERVER)
		{
			socket_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (socket_fd == ERROR_)
			{
				status=TCP_STATUS_SOCKET_CREATION_ERROR;
				return_val=ERROR_;
			}
			else
			{
				server.sin_family = AF_INET;
				// Bind address is platform-conditional:
				//   Windows  -> 127.0.0.1 (loopback): avoids the Windows Firewall "allow on the
				//     network?" prompt that fires per new mercury.exe path (per-worktree dev/sim
				//     builds spammed the user), and keeps the control socket off the LAN. Every
				//     Windows-local client (sim harness, GUI, VB-Cable ARQ tests) connects via
				//     127.0.0.1, so loopback is correct there.
				//   Linux/Pi -> INADDR_ANY (0.0.0.0): the IONOS bench harness runs on the Windows
				//     HOST and connects to the Pi's LAN IP, so loopback-only REFUSES it (regression
				//     caught 2026-06-04 -- a blanket INADDR_LOOPBACK broke the testbed). The Pi is
				//     Linux (no Windows Firewall), so binding all interfaces is safe + required.
				// If a Windows station ever needs LAN access, add a CLI/INI override (do not flip
				// the Windows default back to INADDR_ANY -- that reintroduces the firewall prompt).
#if defined(_WIN32)
				server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#else
				server.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
				server.sin_port = htons((uint16_t)port);
				status=TCP_STATUS_SOCKET_CREATED;
			}
			int enable = 1;  // Must be int, not char, for Windows setsockopt
			int sso_result = setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int));
			if (sso_result != 0)
			{
				status=TCP_STATUS_REUSEADDR_ERROR;
				return_val=ERROR_;
			}
#if !defined(_WIN32)
			if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(int)) != 0)
			{
				status=TCP_STATUS_REUSEPORT_ERROR;
				return_val=ERROR_;
			}
#endif
			int bind_result = bind(socket_fd, (struct sockaddr*)&server, sizeof(server));
			if (bind_result != 0)
			{
				status=TCP_STATUS_BINDING_ERROR;
				return_val=ERROR_;
			}
			else
			{
				status=TCP_STATUS_BINDED;
			}
			int listen_result = listen(socket_fd, 5);
			if (listen_result != 0)
			{
				status=TCP_STATUS_LISTENING_ERROR;
				return_val=ERROR_;
			}
			else
			{
				status=TCP_STATUS_LISTENING;
			}
#if defined(_WIN32)
            u_long mode = 1;  // 1 to enable non-blocking socket
            ioctlsocket(socket_fd, FIONBIO, &mode);
#else
			fcntl(socket_fd, F_SETFL, O_NONBLOCK);
#endif
		}
		if(type==TYPE_CLIENT)
		{
			socket_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (socket_fd == ERROR_)
			{
				status=TCP_STATUS_SOCKET_CREATION_ERROR;
				std::cout<<"Error-Client socket can't be created"<<std::endl;
				return_val=ERROR_;
			}
			else
			{
				server.sin_family = AF_INET;
				server.sin_addr.s_addr = inet_addr(address);
				server.sin_port = htons(port);
				status=TCP_STATUS_SOCKET_CREATED;
				std::cout<<"Client socket is created"<<std::endl;

			}
			if (connect(socket_fd, (struct sockaddr*)&server, sizeof(server)) != 0) {
				status=TCP_STATUS_CONNECTING_ERROR;
				std::cout<<"Error-Client can't initial the connection"<<std::endl;
				return_val=ERROR_;
			}
			else
			{
				status=TCP_STATUS_CONNECTED;
				std::cout<<"Client is connected to "<<inet_ntoa(server.sin_addr)<<std::endl;
			}
#if defined(_WIN32)
            u_long mode = 1;  // 1 to enable non-blocking socket
            ioctlsocket(socket_fd, FIONBIO, &mode);
#else
			fcntl(socket_fd, F_SETFL, O_NONBLOCK);
#endif
		}
	}


    return return_val;
}

int cl_tcp_socket::check_incomming_connection()
{
    int return_val=SUCCESS;
#if defined(_WIN32)
    int len = sizeof(client);
#else
    unsigned int len = sizeof(client);
#endif
    connection_fd = accept(socket_fd, (struct sockaddr*)&client, &len);

#if defined(_WIN32)
    u_long mode = 1;  // 1 to enable non-blocking socket
    ioctlsocket(connection_fd, FIONBIO, &mode);
#else
    fcntl(connection_fd, F_SETFL, O_NONBLOCK);
#endif


	if(connection_fd < 0 || (client.sin_addr.s_addr==htonl(INADDR_ANY)))
	{
		if(connection_fd < 0 && get_status()!=TCP_STATUS_LISTENING)
		{
			return_val=ERROR_;
			status=TCP_STATUS_LISTENING;
			std::cout<<"Client connection dropped"<<std::endl;
			std::cout<<"Server socket is listening"<<std::endl;
		}
		return_val=ERROR_;
	}
	else
	{
		if (connection_fd < 0) {
			status=TCP_STATUS_ACCEPTING_ERROR;
			std::cout<<"Error-Server can't accept the connection"<<std::endl;
			return_val=ERROR_;
		}
		else
		{
			status=TCP_STATUS_ACCEPTED;
			std::cout<<"Server accepted a connection from "<<inet_ntoa(client.sin_addr)<<std::endl;
		}
	}
	return return_val;
}


// FIX-6 (RX-drain backpressure regression seam): when set, transmit() routes
// through this hook instead of the real send(). It lets the in-process unit test
// (--test-rx-drain-backpressure) model a back-pressured / would-block app socket
// WITHOUT real sockets — the exact condition that the bench3 61,621-byte stall hit
// on RF (a full OS send buffer). The hook receives the bytes the test "accepts"
// and returns the same short/would-block semantics as a non-blocking send():
//   >=0 = bytes accepted (may be < length: a short write),
//   <0  = would-block (EWOULDBLOCK), nothing accepted.
// Production builds leave it NULL → real send() path, zero behavior change.
int (*cl_tcp_socket::g_test_transmit_hook)(const char* buf, int length) = nullptr;

int cl_tcp_socket::transmit()
{
	int n=0;

	if(g_test_transmit_hook != nullptr)
	{
		// Test seam: model the app socket's send() (short / would-block) without RF.
		n = g_test_transmit_hook(message->buffer, message->length);
		if(type==TYPE_SERVER) server_sent_packets++; else client_sent_packets++;
		return n;
	}

	if (type==TYPE_SERVER)
	{
		n= send(connection_fd,message->buffer, message->length,0);
		server_sent_packets++;

	}
	else if(type==TYPE_CLIENT)
	{
		n= send(socket_fd,message->buffer, message->length,0);
		client_sent_packets++;
	}

	return n;
}

int cl_tcp_socket::receive()
{
	int n=0;
	if (type==TYPE_SERVER)
	{
		n= recv (connection_fd,message->buffer, MAX_BUFFER_SIZE,0);
		if(n>0)
		{
			message->length=n;
			message->status=MESSAGE_STATUS_CAPTURED;
			server_received_packets++;
		}
	}
	else if(type==TYPE_CLIENT)
	{
		n= recv (socket_fd,message->buffer, MAX_BUFFER_SIZE,0);
		if(n>0)
		{
			message->length=n;
			message->status=MESSAGE_STATUS_CAPTURED;
			client_received_packets++;
		}
	}

	return n;
}

int cl_tcp_socket::get_status()
{
	return status;
}

void cl_tcp_socket::set_type(int _type)
{
	if (_type==TYPE_SERVER)
	{
		type=TYPE_SERVER;
	}
	else if(_type==TYPE_CLIENT)
	{
		type=TYPE_CLIENT;
	}
	else
	{
		std::cout<< "wrong type"<<std::endl;
		exit(ERROR_);
	}
}

void cl_tcp_socket::print_packet_status()
{
	if (type==TYPE_SERVER)
	{
		std::cout<<"Server: Packets Received= "<<server_received_packets<<" Packets sent= "<<server_sent_packets<<std::endl;

	}
	else if(type==TYPE_CLIENT)
	{
		std::cout<<"Client: Packets Received= "<<client_received_packets<<" Packets sent= "<<client_sent_packets<<std::endl;

	}
}


