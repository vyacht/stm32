char * nmea[] = {
  "$GPRMC,194907.00,A,5241.99815,N,00517.56525,E,0.005,,010809,,,A*7B",
  "!AIVDO,1,1,,,B3aC3LP00063aj7RNpl03wSUwP06,0*43",
  "!AIVDM,1,1,,B,13njCt031t0DA=lN2:jKmad60l1p,0*12",
  "$GPGBS,194907.00,3.0,1.9,4.2,,,,*4E",
  "$GPRMC,194908.00,A,5241.99805,N,00517.56503,E,0.003,,010809,,,A*77",
  "!AIVDO,1,1,,,B3aC3LP00063ai7RNph03wT5wP06,0*23",
  "!AIVDM,1,1,,B,33meMd50000EoJPMvw?:Ubp@0000,0*4F"
};

int main(int argc, char * argv[])
{
  int i = 0;
  int j = 0;
  char *state_table[] = {
#include "packet_names.h"
  };

  struct gps_packet_t lexer;
  packet_init(&lexer);

  for (i = 0; i < 7; i++) {
     printf("Sentence no %d\n", i);
     for(j = 0; j < strlen(nmea[i]); j++) {
       lexer.inbuffer[lexer.inbuflen] = nmea[i][j];
       lexer.inbuflen++;
       if(lexer.inbuflen % 10 == 0)
         packet_parse(&lexer);
     }
     lexer.inbuffer[lexer.inbuflen] = '\r';
       lexer.inbuflen++;
     lexer.inbuffer[lexer.inbuflen] = '\n';
       lexer.inbuflen++;

  }

  packet_parse(&lexer);

  return 0;

}


