#include <stddef.h>
#include <stdio.h>

#include "main.h"
#include "proto.h"

static port_t   port[PORTS_NUM];

/* Calculating XMODEM CRC-16 in 'C'
   ================================
   Reference model for the translated code */

#define poly 0x1021

/* On entry:
 addr=>start of data
 len = length of data
 crc = incoming CRC     */
static uint16_t crc16(uint8_t* addr, int len, uint32_t crc)
{
  int i;
  for(; len > 0; len--)                 /* Step through bytes in memory */
  {
    crc = crc ^ (*addr++ << 8);         /* Fetch byte from memory, XOR into CRC top byte*/
    for (i=0; i<8; i++)                 /* Prepare to rotate 8 bits */
    {
      crc = crc << 1;                   /* rotate */
      if (crc & 0x10000)                /* bit 15 was set (now bit 16)... */
          crc = (crc ^ poly) & 0x0FFFF; /* XOR with XMODEM polynomic */
                                        /* and ensure CRC remains 16-bit value */
    }                                   /* Loop for 8 bits */
  }                                     /* Loop until num=0 */
  return (uint16_t)crc;                 /* Return updated CRC */
}

static inline uint16_t packet_crc(packet_t* pkt)
{
  int len = pkt->len + offsetof(packet_t, data);
	uint8_t* p = (uint8_t*)pkt;
	uint16_t crc = crc16(p, len, 0);
	return crc;
}

void proto_init(int n, lwrb_t*  rb)
{
	port[n].rb = rb;
	port[n].rx_pos = 0;
	port[n].rx_time = 0;
}

int proto_tx(lwrb_t* rb, packet_t* pkt)
{
  int rc, len = pkt->len + offsetof(packet_t, data);
	uint8_t* p = (uint8_t*)pkt;
	uint16_t crc = packet_crc(pkt);
	uint8_t* c = (uint8_t*)&crc;
	pkt->data[pkt->len + 0] = c[0];
	pkt->data[pkt->len + 1] = c[1];
	pkt_dump("TX", pkt);
  rc = lwrb_write(rb, p, len+2);
	return rc;
}

void pkt_dump(const char* s, packet_t* p)
{
 	int i;
	printf("%s: 0x%02X->0x%02X 0x%02X cmd:0x%02X [%d]: ", s, p->src, p->dst, p->trid, p->cmd, p->len);
	for(i = 0; i < p->len; ++i) { printf("0x%02X ", p->data[i]); }
	printf("0x%04X\n", p->data[p->len] + (p->data[p->len+1]<<8));
}

packet_t* proto_poll(int n)
{
	int rc;
  packet_t *pkt = (packet_t*)port[n].pkts;
  uint8_t rx_pos = port[n].rx_pos;
	uint8_t  c, *p = (uint8_t*)pkt;

  // Check for timeout    
  // if(rx_pos && port[n].rx_time && HAL_GetTick() > port[n].rx_time) rx_pos = 0;
	
	do
	{
    rc = lwrb_read(port[n].rb, &c, 1);
		if(rc <= 0) { pkt = NULL; break; }
		
		if(rx_pos == 0 && c != SOP_485_CODE0) continue;
		if(rx_pos == 1 && c != SOP_485_CODE1) { rx_pos = 0; continue; }
		if(rx_pos == 2 && c != SOP_485_CODE2) { rx_pos = 0; continue; }
		if(rx_pos == 3 && c != SOP_485_CODE3) { rx_pos = 0; continue; }
		
		if(rx_pos == offsetof(packet_t, len) && pkt->len > MAX_DATA_LEN) { rx_pos = 0; continue; }
			
		if(rx_pos < offsetof(packet_t, data) || rx_pos < sizeof(packet_t) + pkt->len + 2)
    { 
      p[rx_pos++] = c;
			port[n].rx_time = HAL_GetTick()+PACKET_RX_TIMEOUT;
		}

		if(rx_pos >= offsetof(packet_t, data) + pkt->len + 2)
    {
      rx_pos = 0;
      uint16_t crc = (pkt->data[pkt->len+1] << 8) + pkt->data[pkt->len];
#if 1 // debug
			pkt_dump("PKT", pkt);
			printf("crc: 0x%04X\n", crc /*packet_crc(pkt)*/);
#endif
      if(crc == packet_crc(pkt)) break;
    }
	} while(1);
	
	port[n].rx_pos = rx_pos;

	return pkt;

}