#pragma once

#define MAX_UDP_PACKET_SIZE 65467 // UDP protocol max message size

struct UDPPacket {
	unsigned short port;    
	int source_addr;
  int dest_addr;

	unsigned int len;
	unsigned char* data;
};


