#ifndef _NMEA2000_H_
#define _NMEA2000_H_

#include <stdint.h>
#include "stm32f10x_can.h"

#define MAX_PACKET_LENGTH	516	/* 7 + 506 + 3 */

typedef enum {
    complete = 0, 
    error = 1,  
    incomplete = 2
} nmea2000_state;

typedef struct {
  uint16_t idx;
  uint32_t ptr;
  uint32_t fast_packet_len;
  uint8_t outbuffer[MAX_PACKET_LENGTH*2+1];
  uint32_t outbuflen;
  uint32_t pgn;
  uint32_t state;
  uint32_t frame_count;
  uint32_t packet_count;
  uint32_t packet_transfer_count;
} nmea2000_packet;

extern void nmea2000_parsemsg(CanRxMsg * frame, nmea2000_packet * packet);

#endif // _NMEA2000_H_

