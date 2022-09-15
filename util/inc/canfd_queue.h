#ifndef CANFD_QUEUE_H
#define CANFD_QUEUE_H

#include <stdint.h>
#include <stdio.h>

#define MAX_QUEUE_SIZE          32U

#define QUEUE_FULL_ERR          1U
#define QUEUE_EMPTY_ERR         2U
#define QUEUE_SUCCESS           0U

#define CANFD_SYNC_QUEUE          0U
#define CANFD_ASYNC_QUEUE         1U
#define CANFD_MAX_QUEUES          1U

typedef struct CANFD_Queue_Desc
{
    uint32_t msg_id;
    uint32_t is_cyclic;
    int32_t cycle_count;
}canfd_queue_desc_t;

typedef struct CANFD_Queue
{
    uint32_t size;
    canfd_queue_desc_t volatile *PutPt;
    canfd_queue_desc_t volatile *GetPt;
}canfd_queue_t;


void canfd_queue_init(void);
uint32_t canfd_enqueue(canfd_queue_desc_t e, uint32_t qtype);
uint32_t canfd_dequeue(canfd_queue_desc_t *qdesc, uint32_t qtype);
uint32_t canfd_queue_peek(canfd_queue_desc_t *e, uint32_t qtype);
void canfd_queue_flush(uint32_t pack);
uint32_t canfd_is_queue_initialized(void);

#endif /* CANFD_QUEUE_H */
