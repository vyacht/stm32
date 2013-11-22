/*
*/

#include <stdint.h>
#include "stm32f10x_can.h"

#include "nmea2000.h"
#include "uart.h"

extern uint32_t packetCounter;
extern uint32_t packetFastCounter;
extern uint32_t packetErrorCounter;
uint32_t frameCounter;

void nmea2000_parsemsg(CanRxMsg * frame, nmea2000_packet * packet) {

    uint8_t l2;

    uint32_t mid = (frame->ExtId >> 8) & 0x01ffff;

    if ((mid>>8) < 240)
       mid &= 0x01ff00;

    frameCounter++;
    if((mid == 129029) || (mid == 126464) || (mid == 126996) || (mid == 129540)) {

      if((frame->Data[0] & 0x1f) == 0) {
        // start of fast transmission
        packet->fast_packet_len = frame->Data[1];
        packet->idx = frame->Data[0];
        packet->ptr = 0;
        packet->idx += 1;
        packet->pgn = 0; // use this sign for whole fast trans being done
        packet->state = incomplete;

        for (l2=2;l2<8;l2++) {
          packet->outbuffer[packet->ptr++]= frame->Data[l2];
        }
      } else if(frame->Data[0] == packet->idx) {
	packet->state = incomplete;
        for (l2=1;l2<8;l2++) {
          if (packet->fast_packet_len > packet->ptr) {
            packet->outbuffer[packet->ptr++] = frame->Data[l2];
          }
        }
        if (packet->ptr == packet->fast_packet_len) {
          packet->outbuflen = packet->fast_packet_len;
          packet->fast_packet_len = 0;
          packet->pgn = mid;
          packet->state = complete;
          packet->packet_count++;
        } else {
          packet->idx += 1;
        }
      } else {
        // error
        // packetError = packet->idx;
        packet->idx = 0;
        packet->fast_packet_len = 0;
        packet->state = error;
        packetErrorCounter++;
	UART_printf(0, "Error packet cnt = %d, idx = %d, error cnt = %d\n", 
		    packet->packet_count, packet->idx, packetErrorCounter);
      }
    } else {
      // single transmission
      packet->ptr = 0;
      for (l2=2;l2<frame->DLC;l2++) {
        packet->outbuffer[packet->ptr++]= frame->Data[l2];
      }
      packet->idx = 0;
      packet->outbuflen = frame->DLC;
      packet->fast_packet_len = 0;
      packet->pgn = mid;
      packet->state = complete; 
      packet->packet_count++;
    }
    packet->frame_count++;
}
