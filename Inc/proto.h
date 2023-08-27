#ifndef _PROTO_H_
#define _PROTO_H_

#include <stdint.h>
#include "lwrb.h"

#define MAX_DATA_LEN  110
#define PACKET_RX_TIMEOUT  20
#define PACKETS_NUM 8
#define PORTS_NUM 1

#ifndef NULL
#define NULL ((void *)0)
#endif

enum
{
	SOP_485_CODE0 = 0x21,
	SOP_485_CODE1 = 0x34,
	SOP_485_CODE2 = 0x38,
	SOP_485_CODE3 = 0x35
};

enum
{
	CMD_485_PING = 0,
	CMD_485_REG_READ8,
	CMD_485_REG_READ16,
	CMD_485_REG_READ32,
	CMD_485_DATA_READ,
	CMD_485_REG_WRITE8,
	CMD_485_REG_WRITE16,
	CMD_485_REG_WRITE32,
	CMD_485_DATA_WRITE
};

typedef struct
{
	uint8_t  sop[4];  // Start of packet
	uint8_t  dst; 
	uint8_t  src;
	uint8_t  trid; // Transaction id
	uint8_t  cmd;  // Command/Reply code
	uint8_t  len;  // Data length
	uint8_t  data[MAX_DATA_LEN+2];
} packet_t;


typedef struct
{
	lwrb_t*  rb;
	uint8_t  rx_pos;
	long     rx_time;
  packet_t pkts[PACKETS_NUM];
} port_t;

void proto_init(int n, lwrb_t*  rb);
packet_t* proto_poll(int n);
int proto_tx(lwrb_t* rb, packet_t* packet);

void pkt_dump(const char* s, packet_t* packet);


#endif /* _PROTO_H_ */