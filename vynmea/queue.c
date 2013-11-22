#include "queue.h"

int queue_pop(Queue * q, CanRxMsg * msg) {

    uint16_t tail;

    // ring buffer -> <= won't work
    if(q->can_tail != q->can_head) {
        tail = q->can_tail + 1;
        if(tail >= RxCanMask) tail = 0;
        q->can_tail = tail;
        *msg = q->can_msg[tail];
        return 1;
    }
    return 0;
}

void queue_push(Queue * q, CanRxMsg * msg) {

    uint16_t head;

    head = (q->can_head + 1);
    if(head >= RxCanMask) head = 0;

    // caught main's tail?
    q->can_msg[head] = *msg;
    q->can_head = head;

    if ( head == q->can_tail) {
        // skip message
        q->overflow = 1;
    }

}
