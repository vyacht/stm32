#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "nmea0183.h"
#include "uart.h"
#include "spi.h"

#define dbg_printf(x, ...) 

#define NMEA_ENABLE 1
enum
{
#include "packet_states.h"
};

void packet_reset( /*@out@*/ struct gps_packet_t *lexer)
/* return the packet machine to the ground state */
{
    lexer->type = BAD_PACKET;
    lexer->state = GROUND_STATE;
    lexer->inbuflen = 0;
    lexer->inbufptr = lexer->inbuffer;
#ifdef BINARY_ENABLE
    isgps_init(lexer);
#endif /* BINARY_ENABLE */
}

void packet_init(struct gps_packet_t * packet) {
  packet->inbufptr = 0;
  packet->inbuflen = 0;
  packet_reset(packet);
}

static void character_pushback(struct gps_packet_t *lexer)
/* push back the last character grabbed */
{
    --lexer->inbufptr;
}

static void nextstate(struct gps_packet_t *lexer, unsigned char c)
{
    static int n = 0;
    n++;
    switch (lexer->state) {
    case GROUND_STATE:
	n = 0;
	if (c == '#') {
	    lexer->state = COMMENT_BODY;
	    break;
	}
#ifdef NMEA_ENABLE
	if (c == '$') {
	    lexer->state = NMEA_DOLLAR;
	    break;
	}
	if (c == '!') {
	    lexer->state = NMEA_BANG;
	    break;
	}
#endif /* NMEA_ENABLE */
	break;
    case COMMENT_BODY:
	if (c == '\n')
	    lexer->state = COMMENT_RECOGNIZED;
	else if (!isprint(c)) {
	    lexer->state = GROUND_STATE;
	    character_pushback(lexer);
	}
	break;
#ifdef NMEA_ENABLE
    case NMEA_DOLLAR:
	if (c == 'G')
	    lexer->state = NMEA_PUB_LEAD;
	else if (c == 'P')	/* vendor sentence */
	    lexer->state = NMEA_VENDOR_LEAD;
	else if (c == 'I')	/* Seatalk */
	    lexer->state = SEATALK_LEAD_1;
	else if (c == 'W')	/* Weather instrument */
	    lexer->state = WEATHER_LEAD_1;
	else if (c == 'H')	/* Heading/compass */
	    lexer->state = HEADCOMP_LEAD_1;
	else if (c == 'T')	/* Turn indicator */
	    lexer->state = TURN_LEAD_1;
	else if (c == 'A')	/* SiRF Ack */
	    lexer->state = SIRF_ACK_LEAD_1;
	else if (c == 'E')	/* ECDIS */
	    lexer->state = ECDIS_LEAD_1;
	else if (c == 'S')
	    lexer->state = SOUNDER_LEAD_1;
#ifdef OCEANSERVER_ENABLE
	else if (c == 'C')
	    lexer->state = NMEA_LEADER_END;
#endif /* OCEANSERVER_ENABLE */
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_PUB_LEAD:
	/*
	 * $GP == GPS, $GL = GLONASS only, $GN = mixed GPS and GLONASS,
	 * according to NMEA (IEIC 61162-1) DRAFT 02/06/2009.
	 */
	if (c == 'P' || c == 'N' || c == 'L')
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_VENDOR_LEAD:
	if (c == 'A')
	    lexer->state = NMEA_PASHR_A;
	else if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    /*
     * Without the following six states, DLE in a $PASHR can fool the
     * sniffer into thinking it sees a TSIP packet.  Hilarity ensues.
     */
    case NMEA_PASHR_A:
	if (c == 'S')
	    lexer->state = NMEA_PASHR_S;
	else if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_PASHR_S:
	if (c == 'H')
	    lexer->state = NMEA_PASHR_H;
	else if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_PASHR_H:
	if (c == 'R')
	    lexer->state = NMEA_BINARY_BODY;
	else if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_BINARY_BODY:
	if (c == '\r')
	    lexer->state = NMEA_BINARY_CR;
	break;
    case NMEA_BINARY_CR:
	if (c == '\n')
	    lexer->state = NMEA_BINARY_NL;
	else
	    lexer->state = NMEA_BINARY_BODY;
	break;
    case NMEA_BINARY_NL:
	if (c == '$') {
	    character_pushback(lexer);
	    lexer->state = NMEA_RECOGNIZED;
	} else
	    lexer->state = NMEA_BINARY_BODY;
	break;
    case NMEA_BANG:
	if (c == 'A')
	    lexer->state = AIS_LEAD_1;
	else if (c == 'B')
	    lexer->state = AIS_LEAD_ALT1;
	else
	    lexer->state = GROUND_STATE;
	break;
    case AIS_LEAD_1:
	if (c == 'I')
	    lexer->state = AIS_LEAD_2;
	else
	    lexer->state = GROUND_STATE;
	break;
    case AIS_LEAD_2:
	if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case AIS_LEAD_ALT1:
	if (c == 'S')
	    lexer->state = AIS_LEAD_ALT2;
	else
	    lexer->state = GROUND_STATE;
	break;
    case AIS_LEAD_ALT2:
	if (isalpha(c))
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_LEADER_END:
	if (c == '\r')
	    lexer->state = NMEA_CR;
	else if (c == '\n')
	    /* not strictly correct, but helps for interpreting logfiles */
	    lexer->state = NMEA_RECOGNIZED;
	else if (c == '$'){
	    /* faster recovery from missing sentence trailers */
	    lexer->state = NMEA_DOLLAR;
	    lexer->inbufptr += (n-1);
	} else if (!isprint(c))
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_CR:
	if (c == '\n')
	    lexer->state = NMEA_RECOGNIZED;
	/*
	 * There's a GPS called a Jackson Labs Firefly-1a that emits \r\r\n
	 * at the end of each sentence.  Don't be confused by this.
	 */
	else if (c == '\r')
	    lexer->state = NMEA_CR;
	else
	    lexer->state = GROUND_STATE;
	break;
    case NMEA_RECOGNIZED:
	if (c == '#')
	    lexer->state = COMMENT_BODY;
	else if (c == '$')
	    lexer->state = NMEA_DOLLAR;
	else if (c == '!')
	    lexer->state = NMEA_BANG;
	else
	    lexer->state = GROUND_STATE;
	break;
    case SEATALK_LEAD_1:
	if (c == 'I' || c == 'N')	/* II or IN are accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case WEATHER_LEAD_1:
	if (c == 'I')		/* Weather instrument leader accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case HEADCOMP_LEAD_1:
	if (c == 'C')		/* Heading/compass leader accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case TURN_LEAD_1:
	if (c == 'I')		/* Turn indicator leader accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case ECDIS_LEAD_1:
	if (c == 'C')		/* ECDIS leader accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case SOUNDER_LEAD_1:
	if (c == 'D')		/* Depth-sounder leader accepted */
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
    case SIRF_ACK_LEAD_1:
	if (c == 'c')
	    lexer->state = SIRF_ACK_LEAD_2;
	else if (c == 'I')
	    lexer->state = AIS_LEAD_2;
	else
	    lexer->state = GROUND_STATE;
	break;
    case SIRF_ACK_LEAD_2:
	if (c == 'k')
	    lexer->state = NMEA_LEADER_END;
	else
	    lexer->state = GROUND_STATE;
	break;
#endif /* NMEA_ENABLE */
    }
}

static void packet_accept(struct gps_packet_t *lexer, int packet_type)
/* packet grab succeeded, move to output buffer */
{
    size_t packetlen = lexer->inbufptr - lexer->inbuffer;
    dbg_printf(0, "Packet type %d accepted %u = %s\n",
		    packet_type, packetlen,
			lexer->inbuffer);

    lexer->type = packet_type;
    spi_writeNmea0183(lexer);
}

static void packet_discard(struct gps_packet_t *lexer)
/* shift the input buffer to discard all data up to current input pointer */
{
    size_t discard = lexer->inbufptr - lexer->inbuffer;
    size_t remaining = lexer->inbuflen - discard;
    lexer->inbufptr = memmove(lexer->inbuffer, lexer->inbufptr, remaining);
    lexer->inbuflen = remaining;
}

static void character_discard(struct gps_packet_t *lexer)
/* shift the input buffer to discard one character and reread data */
{
    memmove(lexer->inbuffer, lexer->inbuffer + 1, (size_t)-- lexer->inbuflen);
    lexer->inbufptr = lexer->inbuffer;
    dbg_printf(0, "Character discarded, buffer %u chars = %s\n",
		    lexer->inbuflen,
		    lexer->inbuffer);
}

#define packet_buffered_input(lexer) \
	((lexer)->inbuffer + (lexer)->inbuflen - (lexer)->inbufptr)

void packet_parse(struct gps_packet_t *lexer) 
{
  dbg_printf("%d: %s\n", lexer->inbuflen, lexer->inbuffer);
  while (packet_buffered_input(lexer) > 0) {

     unsigned char c = *lexer->inbufptr++;
     char *state_table[] = {
#include "packet_names.h"
     };

     nextstate(lexer, c);
     dbg_printf(0, "character '%c' [%02x], new state: %s\n",
		    (isprint(c) ? c : '.'), c,
		    state_table[lexer->state]);
     if (lexer->state == GROUND_STATE) {
            character_discard(lexer);
     } else if (lexer->state == COMMENT_RECOGNIZED) {
            packet_accept(lexer, COMMENT_PACKET);
            packet_discard(lexer);
            lexer->state = GROUND_STATE;
            break;
     }
#ifdef NMEA_ENABLE
     else if (lexer->state == NMEA_RECOGNIZED) {
            /*
             * $PASHR packets have no checksum. Avoid the possibility
             * that random garbage might make it look like they do.
             */
            if (strncmp((const char *)lexer->inbuffer, "$PASHR,", 7) != 0)
            {
                bool checksum_ok = true;
                char csum[3] = { '0', '0', '0' };
                char *end;
                /*
                 * Back up past any whitespace.  Need to do this because
                 * at least one GPS (the Firefly 1a) emits \r\r\n
                 */
                for (end = (char *)lexer->inbufptr - 1; isspace(*end); end--)
                    continue;
                while (strchr("0123456789ABCDEF", *end))
                    --end;
                if (*end == '*') {
                    unsigned int n, crc = 0;
                    for (n = 1; (char *)lexer->inbuffer + n < end; n++)
                        crc ^= lexer->inbuffer[n];
                    (void)snprintf(csum, sizeof(csum), "%02X", crc);
                    checksum_ok = (csum[0] == toupper(end[1])
                                   && csum[1] == toupper(end[2]));
                }
                if (!checksum_ok) {
		  UART_printf(0, "bad checksum in NMEA packet; expected %s.\n",
                        csum);
                    packet_accept(lexer, BAD_PACKET);
                    lexer->state = GROUND_STATE;
                    packet_discard(lexer);
                    break;    /* exit case */
                }
            }
            /* checksum passed or not present */
#ifdef AIVDM_ENABLE
            if (strncmp((char *)lexer->inbuffer, "!AIVDM", 6) == 0)
                packet_accept(lexer, AIVDM_PACKET);
            else if (strncmp((char *)lexer->inbuffer, "!AIVDO", 6) == 0)
                packet_accept(lexer, AIVDM_PACKET);
            else if (strncmp((char *)lexer->inbuffer, "!BSVDM", 6) == 0)
                packet_accept(lexer, AIVDM_PACKET);
            else if (strncmp((char *)lexer->inbuffer, "!BSVDO", 6) == 0)
                packet_accept(lexer, AIVDM_PACKET);
            else
#endif /* AIVDM_ENABLE */
                packet_accept(lexer, NMEA_PACKET);
            packet_discard(lexer);
            break;
        }
#endif /* NMEA_ENABLE */
    }                           /* while */
}
