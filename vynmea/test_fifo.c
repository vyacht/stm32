#include <stdio.h>
#include "fifo.h"

int main(int argc, char * argv[]) {

  unsigned int i = 0;
  char b[32];
  int ret = 0;

  fifo_t fifo;
  fifo_init(&fifo);

  for(i = 0; i < 2600; i++) {
   if(!fifo_put(&fifo, i + 'a')) {
     printf("buffer full at %d\n", i);
     break;
   }
  }

  unsigned int out = -1;
  unsigned int in  = out + 10;
  printf("%u - %u = %u\n", in, out, in - out);

  

  ret = fifo_out(&fifo, b, 7);
  b[ret] = 0;

  printf("%d - %s\n", ret, b);
}
