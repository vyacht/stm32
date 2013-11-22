#ifndef _NMEA0183_H_
#define _NMEA0183_H_

#define MAX_PACKET_LEN 4096

#define BAD_PACKET              -1
#define COMMENT_PACKET          0
#define NMEA_PACKET             1
#define AIVDM_PACKET            2
#define GARMINTXT_PACKET        3
#define MAX_TEXTUAL_TYPE        3       /* increment this as necessary */

struct gps_packet_t {
  int type;
  unsigned int state;
  char inbuffer[MAX_PACKET_LEN];
  char * inbufptr;
  int inbuflen;
  uint8_t origin;
};

void packet_init(struct gps_packet_t * packet);
void packet_parse(struct gps_packet_t *lexer);

#endif // _NMEA0183_H_
