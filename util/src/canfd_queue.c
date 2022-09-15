#include "fw_features.h"
#include "fw_common.h"
#include "canfd_queue.h"
#include "osif.h"

static canfd_queue_desc_t canfd_cmd_queue[CANFD_MAX_QUEUES][MAX_QUEUE_SIZE];
static canfd_queue_t canfd_queue[CANFD_MAX_QUEUES];
static uint32_t canfd_queue_initialized = 0xFC0000AEU;

uint32_t canfd_is_queue_initialized(void)
{
    return (canfd_queue_initialized == 0xFC0000BFU) ? 1U : 0U;
}

void canfd_queue_init(void)
{
    canfd_queue[0].size = MAX_QUEUE_SIZE;
    canfd_queue[0].PutPt = canfd_queue[0].GetPt = &canfd_cmd_queue[0][0];

    canfd_queue_initialized = 0xFC0000BFU;
}

uint32_t canfd_enqueue(canfd_queue_desc_t e, uint32_t qtype)
{
    uint32_t ret = QUEUE_SUCCESS;
    canfd_queue_desc_t volatile *nextPutPt;
    int32_t lc = 0;
    
    DEV_ASSERT(qtype < CANFD_MAX_QUEUES);

    lc = osif_enter_critical();
    
    nextPutPt = canfd_queue[qtype].PutPt + 1;

    if (nextPutPt == &canfd_cmd_queue[qtype][MAX_QUEUE_SIZE]) 
    {
        nextPutPt = &canfd_cmd_queue[qtype][0];   /* Wrap around. */
    }

    if (nextPutPt == canfd_queue[qtype].GetPt) 
    {
        /* Fifo is full. */
        ret = QUEUE_FULL_ERR;
    }
    else
    {
        *(canfd_queue[qtype].PutPt) = e;
        canfd_queue[qtype].PutPt = nextPutPt;
        ret = QUEUE_SUCCESS;
    }
    
    (void)osif_exit_critical(lc);
    
    return ret;
}

uint32_t canfd_dequeue(canfd_queue_desc_t *qdesc, uint32_t qtype)
{
    int32_t lc = 0;
    uint32_t ret = QUEUE_SUCCESS;
    
    DEV_ASSERT(qtype < CANFD_MAX_QUEUES);
    DEV_ASSERT(qdesc != NULL);
    
    lc = osif_enter_critical();
    
    if (canfd_queue[qtype].PutPt == canfd_queue[qtype].GetPt)
    {
        /* Fifo is empty. */
        ret = QUEUE_EMPTY_ERR;
    }

    *qdesc = *(canfd_queue[qtype].GetPt);

    canfd_queue[qtype].GetPt++;
    if (canfd_queue[qtype].GetPt == &canfd_cmd_queue[qtype][MAX_QUEUE_SIZE])
    {
        canfd_queue[qtype].GetPt = &canfd_cmd_queue[qtype][0];   /* Wrap around */
    }

    (void)osif_exit_critical(lc);
    
    return ret;
}

uint32_t canfd_queue_peek(canfd_queue_desc_t *e, uint32_t qtype)
{
    int32_t lc = 0;
    uint32_t ret = QUEUE_SUCCESS;
    
    DEV_ASSERT(qtype < CANFD_MAX_QUEUES);
    DEV_ASSERT(e != NULL);    
    
    lc = osif_enter_critical();
    
    if (canfd_queue[qtype].PutPt == canfd_queue[qtype].GetPt) 
    {
        /* Fifo is empty. */
        ret = QUEUE_EMPTY_ERR;
    }

    *e = *(canfd_queue[qtype].GetPt);

    (void)osif_exit_critical(lc);
    
    return ret;
}

void canfd_queue_flush(uint32_t pack)
{
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    canfd_queue[pack].PutPt = canfd_queue[pack].GetPt = &canfd_cmd_queue[pack][0];
    (void)osif_exit_critical(lc);
}


