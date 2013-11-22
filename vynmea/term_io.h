#ifndef TERM_IO_H_
#define TERM_IO_H_

#include "integer.h"
#include "comm.h"

#define xgetc() (char)comm_get()
#define xavail() comm_test()

int xatoi (char**, long*);
void xputc (char);
void xputs (const char*);
void xitoa (long, int, int);
void xprintf (const char*, ...) __attribute__ ((format (__printf__, 1, 2)));
void put_dump (const BYTE*, DWORD ofs, int cnt);
void get_line (char*, int len);
int get_line_r (char*, int len, int *idx);

#endif /* TERM_IO_H_ */
