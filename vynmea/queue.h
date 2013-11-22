#ifndef _QUEUE_H_
#define _QUEUE_H_

/*
   This queue is for can specifically. Its none-locking 
   if there is only one writer and one reader.
*/

#include "stm32f10x_can.h"

#define RxCanMask 15

typedef struct _Queue {
    CanRxMsg can_msg[15];
    volatile uint16_t can_tail;
    volatile uint16_t can_head;
    volatile uint16_t overflow;
} Queue;

int queue_pop(Queue * q, CanRxMsg * msg);
void queue_push(Queue * q, CanRxMsg * msg);


#endif
