/*
 * Copyright 2017-2020 NXP
 * All rights reserved.
 *
 * This software is owned or controlled by NXP and may only be
 * used strictly in accordance with the applicable license terms. By expressly
 * accepting such terms or by downloading, installing, activating and/or otherwise
 * using the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms. If you do not agree to be
 * bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software. The production use license in
 * Section 2.3 is expressly granted for this software.
 *
 * This file is derived from the Ethernet Interface Skeleton in lwIP with the following copyright:
 *
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 2.1, A project shall not contain unreachable code.
 * These are safety checks to avoid dereferencing NULL pointers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, A compatible declaration shall be
 * visible when an object or function with external linkage is defined.
 * These are symbols weak symbols defined in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter could be declared as pointing to const
 * Type definition is done in another file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.1, Unpermitted operand to operator '&&'
 * Variable is of essential boolean type
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, The value of an expression shall not be assigned to an
 * object with a narrower essential type or a different essential type category.
 * This is a string that will be concatenated to a macro variable to define a new one.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.4, Both operands of an operator in which the usual arithmetic
 * conversions are performed shall have the same essential type category.
 * These are bitwise operations used to enable flags or check their state.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 * Required in comparisons between constants and numerical types.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.2, Conversion between a pointer to incomplete type and another type
 * The is a fake finding
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type
 * and a pointer to a different object type.
 * This is used to check if transmission is complete.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type.
 * The conversion is needed to allocate or free the memory.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed between
 * pointer to void and an arithmetic type.
 * The cast is required to comply with the lwip API that mandates passing arguments
 * to threads using a pointer to void type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 13.5, The right hand operand of a logical && or || operator shall
 * not contain persistent side effects.
 * This is required in order to reduce code complexity.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 14.4, Conditional expression should have essentially Boolean type.
 * This is required for macro constructs in form do {...} while(0).
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 18.2, Substract operator applied to pointers.
 * Operation is required to compute the aligned address of the memory zone that further
 * needs to be freed.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 18.3, Relational or substract operator applied
 * to pointers.
 * Substraction is needed to compute a memory address.
 */

#include <string.h>

#include "eth_task.h" 
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "netifapi.h"

#include "lwip/sys.h"
#include "enet_driver.h"
#include "net_if_cfg.h"
#include "netif/etharp.h"
#include "drv_dp83tc811s.h"
#include "udp_task.h"
#include "wdt_task.h"

#ifndef IF_NAME_0
#define IF_NAME_0          'U','V'
#endif /* IF_NAME_0 */

/* Macros defining whether pbufs are chained or single */
#define ENETIF_SINGLE_PBUF  (1U)
#define ENETIF_CHAINED_PBUF (0U)

/* Custom data section for enet buffers */
#ifndef FEATURE_CUSTOM_DATA_SECTION
#define FEATURE_CUSTOM_DATA_SECTION ".data"
#endif

/*! @brief Media-Independent Interface (MII) default setting */
#ifndef ENET_MIIMODE
#define ENET_MIIMODE        ENET_RMII_MODE
#endif
#ifndef ENET_MIISPEED
#define ENET_MIISPEED       ENET_MII_SPEED_100M
#endif

#define S32_MAC     0x00U, 0x1fU, 0x7bU, 0x73U, 0x03U, 0xC6U
#define IMX_MAC     0x00U, 0x1fU, 0x7bU, 0x73U, 0x03U, 0x74U
#define EC25_MAC    0x00U, 0x80U, 0x48U, 0xbaU, 0xd1U, 0x30U
#define BRIDGE_MAC  0x00U, 0x0aU, 0xe7U, 0x2cU, 0x44U, 0x2aU

#define IMX_IP      192U, 168U, 10U, 50U
#define S32_IP      192U, 168U, 10U, 51U
#define EC25_IP     192U, 168U, 10U, 52U
#define BRIDGE_IP   192U, 168U, 225U, 1U

#ifndef LWIP_NETIF_HOSTNAME_TEXT_0
#define LWIP_NETIF_HOSTNAME_TEXT_0          ("uva-vcu")
#endif /* LWIP_NETIF_HOSTNAME_TEXT_0 */

typedef struct
{
	uint8_t destAddr[6];
	uint8_t srcAddr[6];
	uint16_t length;
	uint8_t payload[256];
}mac_frame_t;

static enet_state_t enetState;
struct netif network_interfaces[ETHIF_NUMBER];

__attribute__((section("ARM_LIB_STACK")))
uint64_t eth_thread_stk[ETH_TASK_STACK_SIZE];

/*! @brief Receive buffer descriptors for ring 0 */
ALIGNED(FEATURE_ENET_BUFFDESCR_ALIGNMENT) static enet_buffer_descriptor_t rx_descriptor[ENET_RXBD_NUM0_0];

/*! @brief Transmit buffer descriptors for ring 0 */
ALIGNED(FEATURE_ENET_BUFFDESCR_ALIGNMENT) static enet_buffer_descriptor_t tx_descriptor[ENET_TXBD_NUM0_0];

/*! @brief Receive data buffers for ring 0 */
ALIGNED(FEATURE_ENET_BUFF_ALIGNMENT) static uint8_t rx_buffer[ENET_RXBD_NUM0_0 * ENET_BUFF_ALIGN(1500U)];

static struct netif *g_netif[ENET_INSTANCE_COUNT];

#ifdef COVERAGE_ENABLED
    struct pbuf dummy_char2;
#endif

/* Lock to synchronize access on TX side, since the frames are sent from different threads */
mutex_t enetif_tx_lock;


/* This handler is called before a frame is dispatched from the ENET driver to the TCPIP stack.
   If extra processing is needed before the dispatch is done, one must implement this handler and
   register it via enetif_register_rx_buff_process_condition_handler.
*/
static rx_buff_process_condition_handler_t rx_buff_process_handler = NULL;

#if (NO_SYS == 0)

/* In order to support zero-copy operation, on the RX side we are using custom pbufs, with the payload pointing to the
   receive buffer obtained from the driver. When the pbuf is eventually freed, the receive buffer is given back to the driver.
   On the TX side we are incrementing the reference count on the pbuf and giving its payload storage to the driver. Once we
   detect the transmission is complete, we are freeing our reference to the pbuf. */

/* Memory pool for RX custom pbufs
   The pool only holds the pbuf_custom structures, not the storage for actual payload */
LWIP_MEMPOOL_DECLARE(RX_POOL, ENET_RXBD_NUM, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool")
/* Queue for passing RX buffers that need to be released back to the driver.
   The actual operation is performed on the same enetif_poll_thread as reception, thus avoiding additional synchronization for RX side */
static sys_mbox_t rx_buffs;

/* Queue for holding pbufs which have been sent to the driver for transmission. They will be released once transmission is complete
  (detected by polling ENET_DRV_GetTransmitStatus) */
static sys_mbox_t in_flight_tx_pbufs;

static sys_thread_t poll_thread;

/*! @brief The MAC address of the module */
static uint8_t enet_s32_mac[NETIF_MAX_HWADDR_LEN]       = {0x00U, 0x1fU, 0x7bU, 0x73U, 0x03U, 0xC6U};  /* 0x001f7b7303C6 */
static uint8_t enet_imx_mac[NETIF_MAX_HWADDR_LEN]       = {0x00U, 0x1fU, 0x7bU, 0x73U, 0x03U, 0x74U};  /* 0x001f7b730374 */
static uint8_t enet_ec25_mac[NETIF_MAX_HWADDR_LEN]      = {0x00U, 0x80U, 0x48U, 0xbaU, 0xd1U, 0x30U};  /* 0x008048bad130 */
static uint8_t enet_bridge_mac[NETIF_MAX_HWADDR_LEN]    = {0x00U, 0x0aU, 0xe7U, 0x2cU, 0x44U, 0x2aU};  /* 0x000ae72c442a */

netif_custom_t netifCfg_0 = {
    .num            = 0,
    .hwaddr         = { S32_MAC },
    .has_dhcp       = false,
    .has_auto_ip    = false,
    .has_IPv6       = true,
    .ip_addr        =  { 192, 168, 10, 51 },
    .netmask        =  { 255,255,255,0 },
    .gw             =  { 192, 168, 10, 1 },
    .hostname       =  LWIP_NETIF_HOSTNAME_TEXT_0,
    .name           =  { IF_NAME_0 },
};

/* Array of netif configurations */
netif_custom_t *netif_cfg[] = {
          &netifCfg_0
};

/**
 * Callback function called when a custom pbuf is freed
 *
 * @param p - the custom pbuf structure
 * Implements enet_pbuf_free_custom_Activity
 */
static void enet_pbuf_free_custom(struct pbuf *p)
{
    LWIP_ASSERT("NULL pointer", p != NULL);
    struct pbuf_custom* pc = (struct pbuf_custom*)p;
    uint8_t *alignedPtr = &rx_buffer[(((uint32_t)((uint8_t*)pc->pbuf.payload - (uint8_t*)rx_buffer) / (uint32_t)ENET_RXBUFF_SIZE) * (uint32_t)ENET_RXBUFF_SIZE)];
    sys_mbox_post((sys_mbox_t *)&rx_buffs, (void *)alignedPtr);
    LWIP_MEMPOOL_FREE(RX_POOL, pc);
}

/**
 * Transmit a packet.
 * The packet is contained in the pbuf that is passed to the function. This pbuf might be chained.
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * @param p - the pbuf structure
 * Implements enetif_low_level_output_Activity
 */
static err_t enetif_low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    enet_buffer_t bd;
    status_t status;
    err_t pbuf_status = ERR_OK;
    uint8_t pbuf_chain_type = ENETIF_SINGLE_PBUF;

    LWIP_ASSERT("Output packet buffer empty", p);
#if defined(LWIP_DEBUG) && LWIP_NETIF_TX_SINGLE_PBUF && !(LWIP_IPV4 && IP_FRAG) && (LWIP_IPV6 && LWIP_IPV6_FRAG)
    LWIP_ASSERT("p->next == NULL && p->len == p->tot_len", p->next == NULL && p->len == p->tot_len);
#endif

    /* Check whether this was single or a chained pbuf */
    if(NULL != p->next)
    {
       /* This is a chained pbuf, save info into a local variable, as p will be lost if allocation does not fail */
       pbuf_chain_type = ENETIF_CHAINED_PBUF;
    }

    /* Increment our reference on p */
    pbuf_ref(p);
    /* If p was a pbuf chain instead, p's ref was decreased and we got another q pbuf with ref 1
    Either way, q has a +1 ref that we need to free in case we're not keeping the buffer - ie in case of errors*/
    q = pbuf_coalesce(p, PBUF_RAW);

    /* If this was a chained pbuf, check allocation */
    if((ENETIF_CHAINED_PBUF == pbuf_chain_type) && (q == p))
    {
        /* Memory allocation failed */
        pbuf_status = ERR_MEM;
    }
    else
    {
        bd.data = q->payload;
        bd.length = q->tot_len;

        while(1)
        {
            while(OSIF_MutexLock(&enetif_tx_lock, 5) != STATUS_SUCCESS) {}
            status = ENET_DRV_SendFrame(netif->num, ENET_QUEUE, &bd, NULL);
            (void)OSIF_MutexUnlock(&enetif_tx_lock);

            /* Keep trying to send the frame as long as the driver says there is not enough space in the queue */
            if(status != STATUS_ENET_TX_QUEUE_FULL)
            {
                break;
            }
            OSIF_TimeDelay(0);
        }
        if(status == STATUS_SUCCESS)
        {
            /* Post the pbuf to mbox, so that it will be checked for completion in enetif_poll_thread*/
            sys_mbox_post((sys_mbox_t *)&in_flight_tx_pbufs, (void *)q);
        }
        else
        {
           /* Decrement the ref (either p's ref in case it was a single pbuf, or the coalesed q's ref) */
           (void)pbuf_free(q);
        }
        pbuf_status = ERR_OK;
    }

    return pbuf_status;
}

/**
 * This function is called when a packet is ready to be read from the interface.
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * @param data - the pointer to the received data buffer
 * @param size - the length of received data buffer
 * @return ERR_OK if the packet is being handled (we take ownership of the data buffer)
 *         ERR_MEM if the packet cannot be handled (we don't take ownership of the data buffer,
 *         therefore the caller should release it)
 * Implements enetif_input_Activity
 */
static err_t enetif_input(struct netif *netif, uint8_t * data, uint16_t size)
{
    err_t ret = ERR_MEM;

    /* Allocate a custom PBUF_REF pointing to the receive buffer */
    struct pbuf_custom* enet_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    if(enet_pbuf != NULL)
    {
        ret = ERR_OK;
        enet_pbuf->custom_free_function = enet_pbuf_free_custom;
        struct pbuf* p = pbuf_alloced_custom(PBUF_RAW, size, PBUF_REF, enet_pbuf, data, size);

        if(netif->input(p, netif) != ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("enetif_input: IP input error\n"));
            (void)pbuf_free(p);
        }
    }
    return ret;
}

/**
 * This function is run on a separate thread and handles communication with the enet interface:
 *  - polls the driver for incoming RX frames
 *  - releases processed receive buffers back to the driver
 *  - polls the driver for outgoing TX frames and subsequently releases user pbufs
 * @param arg - the instance number for this ethernetif
 * Implements enetif_poll_thread_Activity
 */
static void enetif_poll_thread(void *arg)
{
    uint8_t instance = (uintptr_t)arg;
    
    struct pbuf *p;
    enet_buffer_t bd;
    err_t ret;

#ifdef USE_NW_SW_DEBUG
    volatile uint8_t stat = 0U;
    volatile SJA1105P_macLevelErrors_t mac_errs[5];
    volatile SJA1105P_generalStatusL2Argument_t gen_stat;
    volatile SJA1105P_generalStatusHashconfsArgument_t hash_stat;
    volatile SJA1105P_generalStatusVlanArgument_t vlan_stat;
    volatile SJA1105P_generalStatusMem0Argument_t mem0_stat;
    volatile SJA1105P_generalStatusDropArgument_t drop_stat;
    volatile uint32_t mem1_watermark_stat;
    volatile uint32_t ram_err_stat;
    
	volatile SJA1105P_basicControlArgument_t    basicControl;
    volatile SJA1105P_digitalControl1Argument_t digitalControl_1;
    volatile SJA1105P_autonegControlArgument_t  autonegControl;
    volatile SJA1105P_basicStatusArgument_t     basicStatus;
    volatile SJA1105P_digitalStatusArgument_t   digitalStatus;
    volatile SJA1105P_digitalErrorCntArgument_t digitalError;
    
    uint32_t err_in_cnt[24] = {0U, };
    uint32_t err_eg_cnt[24] = {0U, };
    volatile uint16_t phy_data = 0U;
    
#endif /* USE_NW_SW_DEBUG */
    
    /* Check input parameter */
    LWIP_ASSERT("g_netif[instance] != NULL", g_netif[instance] != NULL);

    while(1)
    {
#ifdef USE_NW_SW_DEBUG
        for(uint32_t i = 0; i < 5U; i++)
        {
            stat += SJA1105P_getMacErrors(&mac_errs[i], i);
        }
        
        for(uint32_t i = 0U; i < 24; i++)
        {
            stat += SJA1105P_get32bitEtherStatCounter((SJA1105P_etherStat32_t)i, &err_in_cnt[i], 1U, SJA1105P_e_etherStatDirection_INGRESS);
            stat += SJA1105P_get32bitEtherStatCounter((SJA1105P_etherStat32_t)i, &err_eg_cnt[i], 4U, SJA1105P_e_etherStatDirection_EGRESS);
        }
        
        stat += SJA1105P_getGeneralStatusL2(&gen_stat, 1U);
        stat += SJA1105P_getGeneralStatusHashconfs(&hash_stat, 1U);
        stat += SJA1105P_getGeneralStatusVlan(&vlan_stat, 1U);
        stat += SJA1105P_getGeneralStatusMem0(&mem0_stat, 1U);
        stat += SJA1105P_getGeneralStatusMem1(&mem1_watermark_stat, 1U);
        stat += SJA1105P_getGeneralStatusDrop(&drop_stat, 1U);
        stat += SJA1105P_getGeneralStatusRamError(&ram_err_stat,  1U);
        

        stat += SJA1105P_getBasicControl(&basicControl, 1U);
        stat += SJA1105P_getDigitalControl1(&digitalControl_1, 1U);
        stat += SJA1105P_getAutonegControl(&autonegControl, 1U);
        stat += SJA1105P_getBasicStatus(&basicStatus, 1U);
        stat += SJA1105P_getDigitalStatus(&digitalStatus, 1U);
        stat += SJA1105P_getDigitalErrorCnt(&digitalError, 1U);
        
        /* Get Link status */
        phy_data = 0x1U;
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, phy_data, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, 0x0133U, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, 0x401FU, 50U);
        ENET_DRV_MDIORead(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, (uint16_t *)&phy_data, 50U);
        
        /* Get SGMII Aneg status */
        phy_data = 0x1U;
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, phy_data, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, 0x0459U, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, 0x401FU, 50U);
        ENET_DRV_MDIORead(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, (uint16_t *)&phy_data, 50U);

        /* Get SGMII SQR */
        phy_data = 0x1U;
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, phy_data, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, 0x0198U, 50U);
        ENET_DRV_MDIOWrite(0U, DP83TC811_PHY_ADDR, DP83TC811_REGCR, 0x401FU, 50U);
        ENET_DRV_MDIORead(0U, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, (uint16_t *)&phy_data, 50U);
        
       __nop();
#endif /* USE_NW_SW_DEBUG */

        /* Free any completed receive buffers */
        while (0 == sys_arch_mbox_tryfetch((sys_mbox_t *)&rx_buffs, (void**)&bd.data))
        {
            ENET_DRV_ProvideRxBuff(instance, ENET_QUEUE, &bd);
        }

        /* Check if there are any new RX frames and provide them to lwip stack */
        while (STATUS_SUCCESS == ENET_DRV_ReadFrame(instance, ENET_QUEUE, &bd, NULL))
        {
            if((NULL == rx_buff_process_handler) || (FORWARD_FRAME != rx_buff_process_handler(instance, &bd)))
            {
                ret = enetif_input((struct netif *)g_netif[instance], (uint8_t*)bd.data, (uint16_t)bd.length);
                if(ERR_OK != ret)
                {
                    ENET_DRV_ProvideRxBuff(instance, ENET_QUEUE, &bd);
                }
            }
        }

        /* Check if transmission is complete for any in-flight pbufs */
        while (0 == sys_arch_mbox_tryfetch((sys_mbox_t *)&in_flight_tx_pbufs, (void**)&p))
        {
            if(STATUS_BUSY == ENET_DRV_GetTransmitStatus(instance, ENET_QUEUE, p->payload, NULL))
            {
                /* transmission still in progress for the first buffer, repost to (front of) queue and give up for now */
                sys_mbox_post_to_front((sys_mbox_t *)&in_flight_tx_pbufs, (void *)p);
                break;
            }
            else
            {
                /* request to free the outstanding pbuf on tcpip thread */
                (void)pbuf_free_callback(p);
            }
        }
        
        OSIF_TimeDelay(2);
    }
}

#else /* (NO_SYS == 0) */

/**
 * Transmit a packet.
 * The packet is contained in the pbuf that is passed to the function. This pbuf might be chained.
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * @param p - the pbuf structure
 * Implements enetif_low_level_output_Activity
 */
static err_t enetif_low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    enet_buffer_t bd;
    status_t status;
    err_t pbuf_status = ERR_OK;
    uint8_t pbuf_chain_type = ENETIF_SINGLE_PBUF;

    LWIP_ASSERT("Output packet buffer empty", p);
#if defined(LWIP_DEBUG) && LWIP_NETIF_TX_SINGLE_PBUF && !(LWIP_IPV4 && IP_FRAG) && (LWIP_IPV6 && LWIP_IPV6_FRAG)
    LWIP_ASSERT("p->next == NULL && p->len == p->tot_len", p->next == NULL && p->len == p->tot_len);
#endif

    /* Check whether this was single or a chained pbuf */
    if(NULL != p->next)
    {
       /* This is a chained pbuf, save info into a local variable, as p will be lost if allocation does not fail */
       pbuf_chain_type = ENETIF_CHAINED_PBUF;
    }

    /* Increment our reference on p */
    pbuf_ref(p);
    /* If p was a pbuf chain instead, p's ref was decreased and we got another q pbuf with ref 1
    Either way, q has a +1 ref that we need to free in case we're not keeping the buffer - ie in case of errors*/
    q = pbuf_coalesce(p, PBUF_RAW);

    /* If this was a chained pbuf, check allocation */
    if((ENETIF_CHAINED_PBUF == pbuf_chain_type) && (q == p))
    {
        /* Memory allocation failed */
        pbuf_status = ERR_MEM;
    }
    else
    {
        bd.data = q->payload;
        bd.length = q->tot_len;

        /* Keep trying to send the frame as long as the driver says there is not enough space in the queue */
        do
        {
            status = ENET_DRV_SendFrame(netif->num, ENET_QUEUE, &bd, NULL);
        }
        while (status == STATUS_ENET_TX_QUEUE_FULL);

        if(status == STATUS_SUCCESS)
        {
            /* Wait for the frame to be trnasmitted - TODO: improve this! */
            while (STATUS_BUSY == ENET_DRV_GetTransmitStatus(netif->num, ENET_QUEUE, &bd, NULL)) {}
        }
        /* Decrement the ref (either p's ref in case it was a single pbuf, or the coalesed q's ref) */
        (void)pbuf_free(q);

        pbuf_status = ERR_OK;
    }

    return pbuf_status;
}

/**
 * This function is called when a packet is ready to be read from the interface.
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * @param data - the pointer to the received data buffer
 * @param size - the length of received data buffer
 * @return ERR_OK if the packet is being handled (we take ownership of the data buffer)
 *         ERR_MEM if the packet cannot be handled (we don't take ownership of the data buffer,
 *         therefore the caller should release it)
 * Implements enetif_input_Activity
 */
static err_t enetif_input(struct netif *netif, uint8_t * data, uint16_t size)
{
    err_t ret = ERR_MEM;

    /* Allocate a PBUF_REF pointing to the receive buffer */
    struct pbuf* p  = pbuf_alloc(PBUF_RAW, size, PBUF_REF);
    if(p != NULL)
    {
        ret = ERR_OK;
        p->payload = data;
        if(netif->input(p, netif) != ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("enetif_input: IP input error\n"));
            (void)pbuf_free(p);
        }
    }

    return ret;
}

/**
 * This function polls the driver for received frames
 * Implements enetif_poll_thread_Activity
 */
err_t enet_poll_interface(struct netif *netif)
{
    uint8_t instance = netif->num;
    enet_buffer_t bd;

    /* Check input parameter */
    LWIP_ASSERT("g_netif[instance] != NULL", g_netif[instance] != NULL);

    /* Check if there are any new RX frames and provide them to lwip stack */
    if(STATUS_SUCCESS == ENET_DRV_ReadFrame(instance, ENET_QUEUE, &bd, NULL))
    {
        if((NULL == rx_buff_process_handler) || (FORWARD_FRAME != rx_buff_process_handler(instance, &bd)))
        {
            (void)enetif_input((struct netif *)g_netif[instance], (uint8_t*)bd.data, (uint16_t)bd.length);
            ENET_DRV_ProvideRxBuff(instance, ENET_QUEUE, &bd);
        }
    }

    return ERR_OK;
}

#endif /* (NO_SYS == 0) */

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif - the already initialized lwip network interface structure
 *        for this ethernetif
 * Implements enetif_low_level_init_Activity
 */
static void enetif_low_level_init(struct netif *netif)
{
    enet_config_t enetConfig;
    uint8_t i = 0U;
    volatile uint16_t phy_data = 0U;
    volatile uint32_t phy_read_timeout = 0U;
    const uint32_t phy_read_max_timeout_count = 50U;
    status_t phy_status = STATUS_SUCCESS;

    const enet_buffer_config_t ethernet0_buffConfigArr0[] = {
        {
            ENET_RXBD_NUM0_0,
            ENET_TXBD_NUM0_0,
            &rx_descriptor[0],
            &tx_descriptor[0],
            &rx_buffer[0]
        },
    };
    
    /* set MAC hardware address length */
    netif->hwaddr_len = NETIF_MAX_HWADDR_LEN;

    /* set MAC hardware address */
    for (i = 0U; i < NETIF_MAX_HWADDR_LEN; i++)
    {
      netif->hwaddr[i] = netif_cfg[netif->num]->hwaddr[i];
    }

    /* maximum transfer unit */
    netif->mtu = ETH_MAX_FRAMELEN;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags = (u8_t)(NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET);
    
#if LWIP_IGMP
    netif->flags = netif->flags | (u8_t)NETIF_FLAG_IGMP;
    /*Will add the function igmp_mac_filter to the netif*/
    (netif)->igmp_mac_filter = igmp_enet_filter;
#endif /*LWIP_IGMP*/

    ENET_DRV_GetDefaultConfig(&enetConfig);

    enetConfig.miiMode = ENET_MIIMODE;
    enetConfig.miiSpeed = ENET_MIISPEED;

#if 0
#if FEATURE_ENET_RX_CONFIG_PAYLOAD_LEN_CHECK
    enetConfig.rxConfig |= (uint32_t)(ENET_RX_CONFIG_ENABLE_PAYLOAD_LEN_CHECK);
#endif /* FEATURE_ENET_RX_CONFIG_PAYLOAD_LEN_CHECK */

#if FEATURE_ENET_RX_CONFIG_STRIP_CRC_FIELD
    enetConfig.rxConfig |= (uint32_t)(ENET_RX_CONFIG_STRIP_CRC_FIELD);
#endif /* FEATURE_ENET_RX_CONFIG_STRIP_CRC_FIELD */

#if FEATURE_ENET_RX_CONFIG_REMOVE_PADDING
    enetConfig.rxConfig |= (uint32_t)(ENET_RX_CONFIG_REMOVE_PADDING);
#endif /* FEATURE_ENET_RX_CONFIG_REMOVE_PADDING */

    enetConfig.rxConfig |= (uint32_t)(ENET_RX_CONFIG_ENABLE_FLOW_CONTROL);

#if FEATURE_ENET_HAS_TX_CONFIG
    enetConfig.txConfig = 0;
#endif /* FEATURE_ENET_HAS_TX_CONFIG */

#if FEATURE_ENET_HAS_ACCELERATOR
    enetConfig.rxAccelerConfig = (uint8_t)(ENET_RX_ACCEL_ENABLE_IP_CHECK | ENET_RX_ACCEL_ENABLE_PROTO_CHECK | ENET_RX_ACCEL_REMOVE_PAD);
    enetConfig.txAccelerConfig = (uint8_t)(ENET_TX_ACCEL_INSERT_IP_CHECKSUM | ENET_TX_ACCEL_INSERT_PROTO_CHECKSUM);
#endif /* FEATURE_ENET_HAS_ACCELERATOR */
#endif /* 0 */

    enetConfig.ringCount = ETHERNET0_NUM_RINGS0;

    NETIF_SET_CHECKSUM_CTRL(netif, NETIF_CHECKSUM_SETTING);

    g_netif[netif->num] = netif;
    
    ENET_DRV_Init(netif->num, &enetState, &enetConfig, ethernet0_buffConfigArr0, netif->hwaddr);

#ifndef USE_FEATURE_FAST_TURN_ON
    ec25_pwr_init();
#endif

    ENET_DRV_EnableMDIO(netif->num, false);
    
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_BMCR, DP83TC811_BMCR_RESET, 50U);
    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_BMCR, (uint16_t *)&phy_data, 50U);
    
    while(1)
    {
        if(phy_data != DP83TC811_BMCR_RESET)
        {
            break;
        }
        
        phy_status = ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_BMCR, (uint16_t *)&phy_data, 50U);
        if(phy_status == STATUS_TIMEOUT)
        {
            phy_read_timeout++;
            if(phy_read_timeout > phy_read_max_timeout_count)
            {
                set_status_bit(STAT_VCU_PHY_LINK_RST_FAIL);
                break;
            }
        }
    }

    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_PHYID2, (uint16_t *)&phy_data, 50U);

    sw_asm_delay_us(200000U);
    
    phy_data = (1U << 12U);
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_CFG_SGMII, phy_data, 50U);
    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_CFG_SGMII, (uint16_t *)&phy_data, 50U);

    /* Get Link status */
    phy_data = 0x1FU;
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_REGCR, phy_data, 50U);
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, 0x0133U, 50U);
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_REGCR, 0x401FU, 50U);
    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, (uint16_t *)&phy_data, 50U);
    
    /* Get SGMII SQR */
    phy_data = 0x1FU;
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_REGCR, phy_data, 50U);
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, 0x0198U, 50U);
    (void)ENET_DRV_MDIOWrite(netif->num, DP83TC811_PHY_ADDR, DP83TC811_REGCR, 0x401FU, 50U);
    (void)ENET_DRV_MDIORead(netif->num, DP83TC811_PHY_ADDR, DP83TC811_ADDAR, (uint16_t *)&phy_data, 50U);

    __NOP();
    
}

static void nw_apps_init(void)
{
    udp_task_imx_init();
}

static void tcp_init_done(void* arg)
{    
#if NO_SYS
    LWIP_UNUSED_ARG(arg);
#else /* NO_SYS */
    
    sys_sem_t *init_sem = (sys_sem_t *)arg;
    LWIP_ASSERT("init_sem != NULL", init_sem != NULL);
#endif /* NO_SYS */
    
    /* init network interfaces */
    net_init();
    
    /* init n/w apps */
    nw_apps_init();
    
#if !NO_SYS
    sys_sem_signal(init_sem);
#endif /* !NO_SYS */
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 * Implements enet_ethernetif_init_Activity
 */
err_t enet_ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if (NO_SYS == 0)
    LWIP_MEMPOOL_INIT(RX_POOL);
    (void)sys_mbox_new((sys_mbox_t *)&rx_buffs, ENET_RXBD_NUM0_0);
    (void)sys_mbox_new((sys_mbox_t *)&in_flight_tx_pbufs, 2 * ENET_RXBD_NUM0_0);
    status_t status = OSIF_MutexCreate(&enetif_tx_lock);

    LWIP_ASSERT("status == STATUS_SUCCESS", status == STATUS_SUCCESS);
    (void)status;
#endif /* (NO_SYS == 0) */

    netif->name[0] = netif_cfg[netif->num]->name[0];
    netif->name[1] = netif_cfg[netif->num]->name[1];

#if LWIP_IPV4
#if LWIP_ARP
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = etharp_output;
#else /* LWIP_ARP */
    netif->output = NULL; /* not used for PPPoE */
#endif /* LWIP_ARP */
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
    if(netif_cfg[netif->num]->has_IPv6)
    {
        netif->output_ip6 = ethip6_output;
    }
#endif /* LWIP_IPV6 */
    netif->linkoutput = enetif_low_level_output;
#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    if(NULL != netif_cfg[netif->num]->hostname)
    {
        netif->hostname = netif_cfg[netif->num]->hostname;
    }
#endif /* LWIP_NETIF_HOSTNAME */
#if LWIP_SNMP
    /*
    * Initialize the snmp variables and counters inside the struct netif.
    * The last argument should be replaced with your link speed, in units
    * of bits per second.
    */
    NETIF_INIT_SNMP(netif, (u8_t)snmp_ifType_ethernet_csmacd, (u32_t)100000000);
#endif /* LWIP_SNMP */

    /* initialize the hardware */
    enetif_low_level_init(netif);

#if (NO_SYS == 0)
    /* Start the polling thread */
    poll_thread = sys_thread_new_rtx("enetif_poll_thread", enetif_poll_thread, (void*)((uint32_t)netif->num), ETH_TASK_STACK_SIZE, &eth_thread_stk[0], ETH_TASK_PRIO);
#endif /* (NO_SYS == 0) */

    return ERR_OK;
}

/**
 * Clean up network interface and internal structures
 *
 * @param netif - the lwip network interface structure for this ethernetif
 * Implements enet_ethernetif_shutdown_Activity
 */
void enet_ethernetif_shutdown(struct netif *netif)
{
#if (NO_SYS == 0)
    struct pbuf *p;
    uint8_t *ptr;

    LWIP_ASSERT("netif != NULL", (netif != NULL));

    /* Kill the polling thread */
    sys_thread_delete(poll_thread);

    /* Empty and free the mboxes */
    while (0 == sys_arch_mbox_tryfetch((sys_mbox_t *)&in_flight_tx_pbufs, (void**)&p))
    {
        (void)pbuf_free_callback(p);
    }
    
    sys_mbox_free((sys_mbox_t *)&in_flight_tx_pbufs);
    while (0 == sys_arch_mbox_tryfetch((sys_mbox_t *)&rx_buffs, (void**)&ptr)) {}
    sys_mbox_free((sys_mbox_t *)&rx_buffs);

    ENET_DRV_Deinit(netif->num);

    (void)OSIF_MutexDestroy(&enetif_tx_lock);

#else
    ENET_DRV_Deinit(netif->num);

#endif /* (NO_SYS == 0) */
}

#ifdef COVERAGE_ENABLED
void send_tx_pbuffs_dummy_char(void)
{
    struct pbuf* dummy  = pbuf_alloc(PBUF_RAW, ENET_RXBUFF_SIZE, PBUF_RAM);
    sys_mbox_post((sys_mbox_t *)&in_flight_tx_pbufs, dummy);
}

void send_rx_pbuffs_dummy_char(void)
{

    sys_mbox_post((sys_mbox_t *)&rx_buffs, &dummy_char2);
}
#endif /*COVERAGE_ENABLED*/

#if LWIP_IPV6
/**
 * @ingroup netif_ip6
 * Modify/Configure enet driver setting  to forward (or stop forwarding) multicast packet for MLD (ICMPv6)
 * if "action" = NETIF_ADD_MAC_FILTER , enet module will forward multicast packet of the group corresponding to "group"
 * if "action" = NETIF_DEL_MAC_FILTER , enet module will stop forwarding multicast packet of the group corresponding to "group"
 *
 * @param netif the network interface
 * @*group IP address of the Multicast group the message have to be forwarded by the enet module
 * @action action to be done (remove group from the forwarded packet or add group)
 * Implements design_id_IPv6_Activity
 */


err_t mld_enet_filter (struct netif *netif,
                       const ip6_addr_t *group,
                       enum netif_mac_filter_action action)
{

    /*Generate MAC address based on IP address*/
    uint8_t group_MAC[6];
    group_MAC[0] = 0x33;
    group_MAC[1] = 0x33;
    group_MAC[2] = (uint8_t)((IP6_ADDR_BLOCK7(group)) >> 8);
    group_MAC[3] = (uint8_t)(IP6_ADDR_BLOCK7(group));
    group_MAC[4] = (uint8_t)((IP6_ADDR_BLOCK8(group)) >> 8);
    group_MAC[5] = (uint8_t)(IP6_ADDR_BLOCK8(group));

    /*call function modifying the enet driver*/
    ENET_DRV_SetMulticastForward(netif->num, group_MAC, (action != NETIF_DEL_MAC_FILTER));

    return ERR_OK;


}

#endif /*LWIP_IPV6*/

#if LWIP_IGMP && LWIP_IPV4
/**
 * @ingroup netif_ip4
 * Modify/Configure enet driver setting  to forward (or stop forwarding) multicast packet for IGMP (IPv4)
 * if "action" = NETIF_ADD_MAC_FILTER , enet module will forward multicast packet of the group corresponding to "group"
 * if "action" = NETIF_DEL_MAC_FILTER , enet module will stop forwarding multicast packet of the group corresponding to "group"
 *
 * @param netif the network interface
 * @*group IP address of the Multicast group the message have to be forwarded by the enet module
 * @action action to be done (remove group from the forwarded packet or add group)
 * Implements design_id_IGMP_Activity
 */

err_t igmp_enet_filter (struct netif *netif,
                        const ip4_addr_t *group,
                        enum netif_mac_filter_action action)
{
    /*Generate MAC address based on IP address*/
    uint8_t group_MAC[6];
    group_MAC[0] = 0x01;
    group_MAC[1] = 0x00;
    group_MAC[2] = 0x5e;
    group_MAC[3] = (0x7f & ip4_addr2(group));
    group_MAC[4] = ip4_addr3(group);
    group_MAC[5] = ip4_addr4(group);

    /*call function modifying the enet driver*/
    ENET_DRV_SetMulticastForward(netif->num, group_MAC, (action != NETIF_DEL_MAC_FILTER));
    return ERR_OK;

}
#endif /*LWIP_IGMP && LWIP_IPV4*/

/**
 * Register pre-input handler
 * This handler is called before a frame is input to the TCPIP stack
 * If returns 0, the frame should be forwarded to the stack
 * If returns something else, the frame is used by other applications
 *
 * @param handler - the handler to be installed
 */
void enetif_register_rx_buff_process_condition_handler(rx_buff_process_condition_handler_t handler)
{
    rx_buff_process_handler = handler;
}

osThreadId_t eth_task_get_id(void)
{
    return poll_thread;
}

void net_init(void)
{
    ip4_addr_t ipaddr_imx;
    ip4_addr_t ipaddr_ec25;
    ip4_addr_t ipaddr_bridge;
    err_t e;
    
    struct eth_addr arp_mac_imx = {{IMX_MAC}};
    struct eth_addr arp_mac_ec25 = {{EC25_MAC}};
    struct eth_addr arp_mac_br0 = {{BRIDGE_MAC}};
    
    for(int32_t i = 0; i < ETHIF_NUMBER; i++)
    {
#if LWIP_IPV4
        ip4_addr_t ipaddr, netmask, gw;
#endif /* LWIP_IPV4 */
#if LWIP_DHCP || LWIP_AUTOIP
        err_t err;
#endif /* LWIP_AUTOIP || LWIP_DHCP */

#if LWIP_IPV4
        ip4_addr_set_zero(&gw);
        ip4_addr_set_zero(&ipaddr);
        ip4_addr_set_zero(&netmask);
        
        /* network_interfaces[i] takes the IPV4 addresses from the respective configuration */
        if ((!netif_cfg[i]->has_dhcp) && (!netif_cfg[i]->has_auto_ip))
        {
            IP4_ADDR((&gw), netif_cfg[i]->gw[0], netif_cfg[i]->gw[1], netif_cfg[i]->gw[2], netif_cfg[i]->gw[3]);
            IP4_ADDR((&ipaddr), netif_cfg[i]->ip_addr[0], netif_cfg[i]->ip_addr[1], netif_cfg[i]->ip_addr[2], netif_cfg[i]->ip_addr[3]);
            IP4_ADDR((&netmask), netif_cfg[i]->netmask[0], netif_cfg[i]->netmask[1], netif_cfg[i]->netmask[2], netif_cfg[i]->netmask[3]);
        }
#endif /* LWIP_IPV4 */

#if NO_SYS
        netif_set_default(netif_add(&network_interfaces[i], &ipaddr, &netmask, &gw, NULL, ETHIF_INIT, netif_input));
#else /* NO_SYS */
        (void)netif_add(&network_interfaces[i], &ipaddr, &netmask, &gw, NULL, ETHIF_INIT, tcpip_input);
        netif_set_default(&network_interfaces[i]);
#endif /* NO_SYS */

#if LWIP_IPV6
        if (netif_cfg[i]->has_IPv6)
        {
            netif_create_ip6_linklocal_address(&network_interfaces[i], 1);

#if PRINTF_SUPPORT
            printf("ip6 linklocal address: ");
#endif
            ip6_addr_debug_print(0xFFFFFFFFU & ~LWIP_DBG_HALT, netif_ip6_addr(&network_interfaces[i], 0));
        }
#endif /* LWIP_IPV6 */

#if LWIP_NETIF_STATUS_CALLBACK
        netif_set_status_callback(&network_interfaces[i], status_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */

#if LWIP_NETIF_LINK_CALLBACK
        netif_set_link_callback(&network_interfaces[i], link_callback);
#endif /* LWIP_NETIF_LINK_CALLBACK */

#if LWIP_AUTOIP
        if (netif_cfg[i]->has_auto_ip)
        {
            autoip_set_struct(&network_interfaces[i], &netif_autoip);
        }
#endif /* LWIP_AUTOIP */

#if LWIP_DHCP
        if (netif_cfg[i]->has_dhcp)
        {
          dhcp_set_struct(&network_interfaces[i], &netif_dhcp);
        }
#endif /* LWIP_DHCP */
        netif_set_up(&network_interfaces[i]);
        netif_set_link_up(&network_interfaces[i]);
        
        IP4_ADDR (&ipaddr_imx, 192, 168, 10, 50);
        e = netifapi_arp_add(&ipaddr_imx, &arp_mac_imx, NETIFAPI_ARP_PERM);

        IP4_ADDR (&ipaddr_ec25, 192, 168, 10, 52);
        e = netifapi_arp_add(&ipaddr_ec25, &arp_mac_ec25, NETIFAPI_ARP_PERM);
        
        IP4_ADDR (&ipaddr_bridge, 192, 168, 255, 1);
        e = netifapi_arp_add(&ipaddr_bridge, &arp_mac_br0, NETIFAPI_ARP_PERM);

        DEV_ASSERT(e == ERR_OK);
        
#if LWIP_DHCP
        if (netif_cfg[i]->has_dhcp)
        {
          err = dhcp_start((struct netif *)&network_interfaces[i]);
          LWIP_ASSERT("dhcp_start failed", err == (err_t)ERR_OK);
        }
#endif /* LWIP_DHCP */

#if LWIP_AUTOIP
        else if (netif_cfg[i]->has_auto_ip)
        {
          err = autoip_start(&network_interfaces[i]);
          LWIP_ASSERT("autoip_start failed", err == (err_t)ERR_OK);
        }
#endif /* LWIP_AUTOIP */
  }
}

void net_tasks_init(void)
{
    sys_sem_t tcp_init_sem;
    err_t err;
    
    err = sys_sem_new(&tcp_init_sem, 0U);
    
    tcpip_init(tcp_init_done, (void *)&tcp_init_sem);
    (void)sys_sem_wait(&tcp_init_sem);
    
    sys_sem_free(&tcp_init_sem);

#if (LWIP_SOCKET || LWIP_NETCONN) && LWIP_NETCONN_SEM_PER_THREAD
    netconn_thread_init();
#endif
}

void net_tasks_deinit(void)
{
    for(int32_t i = 0; i < ETHIF_NUMBER; i++)
    {
        netif_set_down(&network_interfaces[i]);
    }
    
    (void)osif_task_end(poll_thread);
}

