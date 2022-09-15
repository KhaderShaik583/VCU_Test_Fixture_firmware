 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
 * All Rights Reserved.
 * 
 * NOTICE: All information contained herein is, and remains the 
 * property of Ultraviolette Automotive Pvt. Ltd. and its suppliers, if 
 * any. The intellectual and technical concepts contained herein are 
 * proprietary to Ultraviolette Automotive  and its suppliers and may 
 * be covered by U.S. and Foreign Patents, patents in process,  and 
 * are protected by trade secret or copyright law. Dissemination of 
 * this information or reproduction of this material is strictly 
 * forbidden unless prior written permission is obtained from 
 * Ultraviolette Automotive Pvt. Ltd.
 * 
 *
 * Author : Rishi F. [010]
 *


 */

#include "lte_atc.h"
#include "ringbuff.h" 

#ifdef USE_NATIVE_LTE
static ringbuff_t uart_at_buff;
static uint8_t ring_buffer[MAX_AT_RING_BUF_SZ];
static uint8_t rx_byte = 0U;

static char scratchpad[MAX_SCR_BUF_LEN];

static osEventFlagsId_t evt_rx_complete;
#define EVT_RX_COMPLETE_FLAGS_MSK   (0x00000001U)

static lpuart_state_t lpuart0_atc_state;
static semaphore_t lte_atc_sem;

static uint32_t qmt_open_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_close_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_publish_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_sub_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_cgreg_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_conn_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_disc_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_qgpsloc_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_qgpsnmea_rmc_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_csq_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qiact_rsp_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_rsp_recv_parse(uint8_t *rxbuffer, uint32_t len);
static uint32_t qmt_rsp_recv_read_parse(uint8_t *rxbuffer, uint32_t len);

/* Update lte_at_cmd_e enum after adding new commands */
static at_cmd_t at_cmd_list[LTE_MAX_AT_CMDS] =  
{
#ifdef USE_FEATURE_ECHO_AT
    {"AT\r",                                    "OK",       "ERROR",    0U,     11U,    0, 300U,    NULL},                  /* AT */
#else
    {"ATE0\r",                                  "OK",       "ERROR",    0U,     11U,    0, 300U,    NULL},                  /* ATE0 */
#endif
    {"AT\r",                                    "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT */
    {"AT+QURCCFG=\"urcport\",\"uart1\"\r",      "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /*AT+QURCCFG="urcport","uart1"*/
    {"AT+CGREG?\r",                             "+CGREG:",  "ERROR",    0U,     2U,     1, 300U,    qmt_cgreg_rsp_parse},   /* AT+CGREG? */
    /*{"AT+CGSN\r",                             "OK",       "ERROR",    0U,     2U,     0, 300U,    NULL},*/                /* AT+CGSN */
    /*{"ATI\r",                                 "OK",       "ERROR",    0U,     2U,     0, 300U,    NULL},*/                /* ATI */
    {"AT+QCFG=\"nwscanmode\",0,1\r",            "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QCFG="nwscanmode",0,1 AUTO*/
    {"AT+QCFG=\"servicedomain\",1,1\r",         "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QCFG="servicedomain",1,1 */
    {"AT+QICSGP=1,1,\"airtelgprs.com\"\r",      "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QICSGP=1,1,"airtelgprs.com" */
    {"AT+QIDEACT=1\r",                          "OK",       "ERROR",    0U,     6U,     0, 40000U,  NULL},                  /* AT+QIDEACT=1 */
    {"AT+QIACT=1\r",                            "OK",       "ERROR",    0U,     6U,     0, 150000U, NULL},                  /* AT+QIACT=1 */
    {"AT\r",                                    "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT */
    
    {"AT+QGPS=1\r",                             "OK",       "ERROR",    0U,     6U,     0, 1500U,   NULL},                  /* AT+QGPS=1 */
    {"AT+QAUDMOD=1\r",                          "OK",       "ERROR",    0U,     6U,     0, 1000U,   NULL},                  /* AT+QAUDMOD=1 */
    {"AT+QDAI=2,0,0,4,0\r",                     "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QDAI=2,0,0,4,0 */
    {"AT+QPSND=1,\"audio_01.wav\",0\r",         "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QPSND=1,"audio_01.wav",0 */
    {"AT+QMTCFG=\"SSL\",0,1,2\r",               "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QMTCFG="SSL",0,1,2 */
    
    {"AT+QMTCFG=\"version\",0,4\r",             "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QMTCFG="version",0,4 */
    {"AT+QMTCFG=\"keepalive\",0,150\r",         "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QMTCFG="keepalive",0,3599 */
    {"AT+QMTCFG=\"recv/mode\",0,1,1\r",         "OK",       "ERROR",    0U,     6U,     0, 300U,    NULL},                  /* AT+QMTCFG=\"recv/mode",0,1,1 */

    {"AT+QSSLCFG=\"cacert\",2,\"cacert.pem\"\r",            "OK",   "ERROR",   0U, 5U,  0, 300U,   NULL},          /* AT+QSSLCFG="cacert",2,"cacert.pem" */
    {"AT+QSSLCFG=\"clientcert\",2,\"clientcert.pem\"\r",    "OK",   "ERROR",   0U, 5U,  0, 300U,   NULL},          /* AT+QSSLCFG="clientcert",2,"clientcert.pem" */
    {"AT+QSSLCFG=\"clientkey\",2,\"clientkey.pem\"\r",      "OK",   "ERROR",   0U, 5U,  0, 300U,   NULL},          /* AT+QSSLCFG="clientkey",2,"clientkey.pem" */
    
    {"AT+QSSLCFG=\"seclevel\",2,2\r",         "OK",         "ERROR",    0U,     6U,         0, 300U, NULL},        /* AT+QSSLCFG="seclevel",2,2 */
    {"AT+QSSLCFG=\"sslversion\",2,4\r",       "OK",         "ERROR",    0U,     6U,         0, 300U, NULL},        /* AT+QSSLCFG="sslversion",2,4 */
    {"AT+QSSLCFG=\"ciphersuite\",2,0xFFFF\r", "OK",         "ERROR",    0U,     6U,         0, 300U, NULL},        /* AT+QSSLCFG="ciphersuite",2,0xFFFF */        
    {"AT+QSSLCFG=\"ignorelocaltime\",1\r",    "+QSSLCFG:",  "ERROR",    0U,     2U,         0, 300U, NULL},        /* AT+QSSLCFG="ignorelocaltime",1 */
 
    {"AT+QMTOPEN=0,\"ec2-3-7-136-45.ap-south-1.compute.amazonaws.com\",1883\r", "+QMTOPEN:",    "ERROR",    0U, 2U,  0, 60000U, qmt_open_rsp_parse},    /* AT+QMTOPEN=0,"",1883 */
    
    {"",                                        "+QMTCONN:",    "ERROR",    0U, 2U, 0, 5000U,    qmt_conn_rsp_parse},               /* AT+QMTCONN=0,"VCU_ZERO" */
    {"AT+QMTSUB=0,1,\"vehtopic\",0\r",          "+QMTSUB:",     "ERROR",    0U, 2U, 0, 2000U,    qmt_sub_rsp_parse},                /* AT+QMTSUB=0,1,"vehtopic",0,*/
    {"AT+QMTPUB=0,1,1,0,\"bmstopic\"\r",        ">",            "ERROR",    0U, 2U, 0, 50U,      qmt_publish_rsp_parse},            /* AT+QMTPUB=0,1,1,0,"bmstopic" */
    {"AT+QMTRECV?\r",                          "+QMTRECV:",     "ERROR",    0U, 2U, 0, 300U,     qmt_rsp_recv_parse},               /* AT+QMTRECV=0*/
    {"",                                       "+QMTRECV:",     "ERROR",    0U, 2U, 0, 300U,     qmt_rsp_recv_read_parse},          /* AT+QMTRECV=0,<id>,<recv_idx>*/
    {"AT+QMTCLOSE=0\r",                         "OK",           "ERROR",    0U, 2U, 0, 2000U,    qmt_close_rsp_parse},              /* AT+QMTCLOSE=0 */
    {"AT+QGPSGNMEA=\"RMC\"\r",                  "+QGPSNMEA:",   "ERROR",    0U, 2U, 0, 50U,      qmt_qgpsnmea_rmc_rsp_parse},       /* AT+QGPSGNMEA="RMC" */
    {"AT+QGPSGNMEA=\"VTG\"\r",                  "+QGPSNMEA:",   "ERROR",    0U, 2U, 0, 50U,      qmt_qgpsloc_rsp_parse},            /* AT+QGPSGNMEA="VTG" */
    {"AT+QMTDISC=0\r",                          "OK",           "ERROR",    0U, 2U, 0, 30000U,   qmt_disc_rsp_parse},               /* AT+QMTDISC=0 */
    {"AT+CSQ\r",                                "OK",           "ERROR",    0U, 6U, 0, 300U,     qmt_csq_rsp_parse},                /* AT+CSQ */
    {"AT+QIACT?\r",                             "+QIACT:",      "ERROR",    0U, 6U, 0, 150000U,  qiact_rsp_parse},                  /* AT+QIACT? */
};

void lte_uart_reconfig(void)
{
    lpuart_user_config_t lpuart0_cfg;
    
    
    lpuart0_cfg.transferType    = LPUART_USING_INTERRUPTS;
    lpuart0_cfg.baudRate        = 115200;
    lpuart0_cfg.parityMode      = LPUART_PARITY_DISABLED;
    lpuart0_cfg.stopBitCount    = LPUART_ONE_STOP_BIT;
    lpuart0_cfg.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
    lpuart0_cfg.rxDMAChannel    = 0U;
    lpuart0_cfg.txDMAChannel    = 0U;
    
    LPUART_DRV_Init(1U, &lpuart0_atc_state, &lpuart0_cfg);

}

void init_at_cmd_list(void)
{
    uint32_t i = 0U;
    uint8_t buffer[16];
    
    for(i = 0U; i < LTE_MAX_AT_CMDS; i++)
    {
        at_cmd_list[i].atcmd_tx_len = strlen(at_cmd_list[i].atccmd);
    }
    
    /* Setup Client-ID as vehicle UID */
    buffer[0] = (uint8_t)(SIM->UIDL & 0x000000FFU) ^ (uint8_t)(SIM->UIDMH & 0x000000FFU);
    buffer[1] = (uint8_t)((SIM->UIDL & 0x0000FF00U) >> 8U) ^ (uint8_t)(SIM->UIDMH & 0x000000FFU);
    buffer[2] = (uint8_t)((SIM->UIDL & 0x00FF0000U) >> 16U) ^ (uint8_t)(SIM->UIDMH & 0x000000FFU);
    buffer[3] = (uint8_t)((SIM->UIDL & 0xFF000000U) >> 24U) ^ (uint8_t)(SIM->UIDMH & 0x000000FFU);

    buffer[4] = (uint8_t)(SIM->UIDML & 0x000000FFU) ^ (uint8_t)(SIM->UIDH & 0x000000FFU);
    buffer[5] = (uint8_t)((SIM->UIDML & 0x0000FF00U) >> 8U) ^ (uint8_t)(SIM->UIDH & 0x000000FFU);
    buffer[6] = (uint8_t)((SIM->UIDML & 0x00FF0000U) >> 16U) ^ (uint8_t)(SIM->UIDH & 0x000000FFU);
    buffer[7] = (uint8_t)((SIM->UIDML & 0xFF000000U) >> 24U) ^ (uint8_t)(SIM->UIDH & 0x000000FFU);

    at_cmd_list[LTE_AT_CMD_QMTCONN].atcmd_tx_len = sprintf(at_cmd_list[LTE_AT_CMD_QMTCONN].atccmd, "AT+QMTCONN=0,\"%s%x\"\r", "VEH_", *(uint32_t *)&buffer[0]);
}

void lte_atc_form_qmt_read_cmd(uint32_t recv_idx)
{
    at_cmd_list[LTE_AT_CMD_QMTRECV_READ].atcmd_tx_len = sprintf(at_cmd_list[LTE_AT_CMD_QMTRECV_READ].atccmd, "AT+QMTRECV=0,%d\r", recv_idx);
}


void lte_rx_cb(void *driverState, uart_event_t event, void *data)
{
    /* Unused parameters */
    UNUSED_PARAM(driverState);
    UNUSED_PARAM(data);

    if(event == UART_EVENT_RX_FULL)
    {
        ringbuff_write(&uart_at_buff, &rx_byte, 1U);
    }
    
    LPUART_DRV_SetRxBuffer(SYS_LTE_UART_IF, (uint8_t *)&rx_byte, 1U);
    LPUART_DRV_ReceiveData(SYS_LTE_UART_IF, (uint8_t *)&rx_byte, 1U);
}


void lte_send_cmd_blocking(uint8_t *buffer, uint32_t tx_len, uint32_t rx_len)
{
    uint32_t lc = 0U;

    LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, buffer, tx_len, 1000U);
}

lte_ret_codes lte_send_data_blocking(uint8_t *buffer, uint32_t tx_len, uint8_t *rxbuffer, uint32_t *rx_len)
{
    size_t available_bytes = 0U;
    uint8_t ctrlz[3] = {0x1A, AT_CR, AT_LF};
    uint32_t flags = 0U;
    lte_ret_codes rc = LTE_AT_ERR;
    
    uint8_t qmt_stat_urc = 0U;
    uint32_t err = 0U;
    
    uint32_t num_bytes = 0U;
    status_t uart_status = STATUS_BUSY;
    
    /* Check if ring buffer has data */
    available_bytes = ringbuff_get_full(&uart_at_buff);
    
    if(available_bytes > 0U)
    {
        /* At this point it is expected that the buffer will not contain data from pervious command */
        /* Read the data into processing buffer */
        ringbuff_read(&uart_at_buff, rxbuffer, available_bytes);
        
        /* Check what was received in the rx buffer */
        /* Ususally it could be a URC. QMTSTAT or straight up ERROR */
        /* In this event, the server could have disconnected the client */
        /* Do not send data and re-establish connection with server */
        qmt_stat_urc = is_qmtstat_urc(rxbuffer, available_bytes);
        err = is_atc_error(rxbuffer, available_bytes);
        
        if((qmt_stat_urc == 1U) || (err == 1U))
        {
            rc = LTE_AT_ERR;
            /* For maintaining the flow this return in between is required */
            return rc;
        }
    }
    
    /* Flush the rx ring */
    ringbuff_reset(&uart_at_buff);
    
    /* Check for any UART faults */
    uart_status = LPUART_DRV_GetTransmitStatus(SYS_LTE_UART_IF, NULL);
    
    if(uart_status == STATUS_SUCCESS)
    {
        LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)buffer, tx_len, 100U);
        LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, ctrlz, 1U, 100U);

        /* Block for specified milliseconds specific to command timeout */
        flags = osEventFlagsWait(evt_rx_complete, EVT_RX_COMPLETE_FLAGS_MSK, osFlagsWaitAny, MSEC_TO_TICK(300));
        /* This is expected to timeout */
        
        uart_status = LPUART_DRV_GetReceiveStatus(SYS_LTE_UART_IF, NULL);
        
        if((uart_status == STATUS_BUSY) || (uart_status == STATUS_SUCCESS))
        {
            /* Check if data available in Ring Buffer */
            available_bytes = ringbuff_get_full(&uart_at_buff);
            
            if(available_bytes > 0U)
            {
                /* Read the data into processing buffer */
#ifdef USE_FEATURE_ECHO_AT
                /* Read the echoed command */
                ringbuff_read(&uart_at_buff, scratchpad, tx_len);
#endif
                ringbuff_read(&uart_at_buff, rxbuffer, available_bytes);
                *rx_len = available_bytes;
                
                rc = LTE_AT_OK;
            }
            else
            {
                /* After the specified timeout the device must respond */
                /* If not, adjust timeout or flag as interface error */
                rc = LTE_AT_ERR;
            }
        }
        else if((uart_status == STATUS_UART_RX_OVERRUN)      ||
           (uart_status == STATUS_UART_TX_UNDERRUN)     ||
           (uart_status == STATUS_UART_ABORTED)         ||
           (uart_status == STATUS_UART_FRAMING_ERROR)   ||
           (uart_status == STATUS_UART_PARITY_ERROR)    ||
           (uart_status == STATUS_UART_NOISE_ERROR))
        {
            /* Framing Error on UART */
            /* Reset UART */
            LPUART_DRV_Deinit(SYS_LTE_UART_IF);
            OSIF_TimeDelay(5);
            lte_uart_reconfig();
 
            ringbuff_reset(&uart_at_buff);
            rc = LTE_AT_HW_ERR;
        }
    }
    else
    {
        rc = LTE_AT_ERR;
    }
    
    return rc;
}

lte_ret_codes lte_atc_at_start(uint32_t cmd_idx, uint8_t *rxbuffer, uint32_t *rx_len)
{
    size_t available_bytes = 0U;
    uint32_t flags = 0U;
    lte_ret_codes rc = LTE_AT_ERR;
    uint8_t qmt_stat_urc = 0U;
    uint32_t err = 0U;
    uint32_t num_bytes = 0U;
    status_t uart_status = STATUS_BUSY;
    volatile char *peek0 = NULL;
    volatile char *peek1 = NULL;
    volatile uint32_t cmd_timeout = 0U;

    
    DEV_ASSERT(cmd_idx < LTE_MAX_AT_CMDS);
    
    
    
    /* Check if ring buffer has data */
    available_bytes = ringbuff_get_full(&uart_at_buff);
    
    if(available_bytes > 0U)
    {
        /* At this point it is expected that the buffer will not contain data from pervious command */
        /* Read the data into processing buffer */
        ringbuff_read(&uart_at_buff, rxbuffer, available_bytes);
        
        /* Check what was received in the rx buffer */
        /* Ususally it could be a URC. QMTSTAT or straight up ERROR */
        /* In this event, the server could have disconnected the client */
        /* Do not send data and re-establish connection with server */
        qmt_stat_urc = is_qmtstat_urc(rxbuffer, available_bytes);
        err = is_atc_error(rxbuffer, available_bytes);
        
        if((qmt_stat_urc == 1U) || (err == 1U))
        {
            rc = LTE_AT_ERR;
            
            /* For maintaining the flow this return in between is required */
            return rc;
        }
    }
    
    /* Flush the rx ring */
    ringbuff_reset(&uart_at_buff);
    
    /* Check for any UART faults */
    uart_status = LPUART_DRV_GetTransmitStatus(SYS_LTE_UART_IF, NULL);
    
    if(uart_status == STATUS_SUCCESS)
    {
        /* Send the AT command */
        lte_send_cmd_blocking((uint8_t *)at_cmd_list[cmd_idx].atccmd, at_cmd_list[cmd_idx].atcmd_tx_len, at_cmd_list[cmd_idx].atcmd_rx_len);

        while(true)
        {
            available_bytes = ringbuff_get_full(&uart_at_buff);
            if(available_bytes > 0U)
            {
                ringbuff_peek(&uart_at_buff, 0U, scratchpad, available_bytes);
                peek0 = strstr((const char *)scratchpad, at_cmd_list[cmd_idx].at_exp_rsp0);
                peek1 = strstr((const char *)scratchpad, at_cmd_list[cmd_idx].at_exp_rsp1);
                if((peek0 != NULL) || (peek1 != NULL))
                {
                    break;
                }
            }
            
            flags = osEventFlagsWait(evt_rx_complete, EVT_RX_COMPLETE_FLAGS_MSK, osFlagsWaitAll, MSEC_TO_TICK(50U));
            /* This is expected to timeout */
            cmd_timeout = cmd_timeout + 50U;
            if((cmd_timeout >= at_cmd_list[cmd_idx].timeout_ms) ||  \
               (available_bytes == 0U))
            {
                break;
            }
        }
        
        memset(scratchpad, '\0', available_bytes);
        uart_status = LPUART_DRV_GetReceiveStatus(SYS_LTE_UART_IF, NULL);
        
        if((uart_status == STATUS_BUSY) || (uart_status == STATUS_SUCCESS))
        {

            /* Check if data available in Ring Buffer */
            available_bytes = ringbuff_get_full(&uart_at_buff);
        
            if(available_bytes > 0U)
            {
                /* Read the data into processing buffer */
                
#ifdef USE_FEATURE_ECHO_AT
                /* Read the echoed command */
                ringbuff_read(&uart_at_buff, scratchpad, at_cmd_list[cmd_idx].atcmd_tx_len);
                memset(scratchpad, '\0', at_cmd_list[cmd_idx].atcmd_tx_len);
#endif
                ringbuff_read(&uart_at_buff, rxbuffer, available_bytes);
                *rx_len = available_bytes;
                
                rc = LTE_AT_OK;
            }
            else
            {
                /* After the specified timeout the device must respond */
                /* If not, adjust timeout or flag as interface error */
                rc = LTE_AT_ERR;
            }
        }
        else if((uart_status == STATUS_UART_RX_OVERRUN)     ||
               (uart_status == STATUS_UART_TX_UNDERRUN)     ||
               (uart_status == STATUS_UART_ABORTED)         ||
               (uart_status == STATUS_UART_FRAMING_ERROR)   ||
               (uart_status == STATUS_UART_PARITY_ERROR)    ||
               (uart_status == STATUS_UART_NOISE_ERROR))
        {

            /* Framing Error on UART */
            /* Reset UART */
            LPUART_DRV_Deinit(SYS_LTE_UART_IF);
            OSIF_TimeDelay(100);
            lte_uart_reconfig();
            ringbuff_reset(&uart_at_buff);
            
            rc = LTE_AT_HW_ERR;
            
        }
        else
        {
            __NOP();
        }
    }
    else
    {
        rc = LTE_AT_ERR;
    }
    
    return rc;
}

void lte_atc_init(void)
{
    /* Initialize buffer */  
    ringbuff_init(&uart_at_buff, ring_buffer, sizeof(ring_buffer));  
    
    /* Setup bounds */
    ring_buffer[MAX_AT_RING_BUF_SZ - 1U] = 0xEU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 2U] = 0xBU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 3U] = 0xAU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 4U] = 0xBU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 5U] = 0xDU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 6U] = 0xAU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 7U] = 0xEU;
    ring_buffer[MAX_AT_RING_BUF_SZ - 8U] = 0xDU;
    
    /* Setup Callback */
    LPUART_DRV_InstallRxCallback(SYS_LTE_UART_IF, lte_rx_cb, NULL);

    /* Setup buffer */
    LPUART_DRV_SetRxBuffer(SYS_LTE_UART_IF, (uint8_t *)&rx_byte, 1U);
    
    /* Setup event flag */
    evt_rx_complete = osEventFlagsNew(NULL);
    DEV_ASSERT(evt_rx_complete != NULL);
    
    /* Start RX process */
    LPUART_DRV_ReceiveData(SYS_LTE_UART_IF, (uint8_t *)&rx_byte, 1U); 
    
}


static uint32_t qmt_open_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = LTE_AT_FAILED_TO_OPEN_NW;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'O') && (rxbuffer[i + 5] == 'P') &&           
           (rxbuffer[i + 6] == 'E') && (rxbuffer[i + 7] == 'N') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            if(rxbuffer[i] == '-')
            {
                return LTE_AT_FAILED_TO_OPEN_NW;
            }
            else
            {
                ret = rxbuffer[i] - '0';
            }
        }
    }
    
    return ret;
}

static uint32_t qmt_close_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'C') && (rxbuffer[i + 5] == 'L') &&           
           (rxbuffer[i + 6] == 'O') && (rxbuffer[i + 7] == 'S') &&
           (rxbuffer[i + 8] == 'E') && (rxbuffer[i + 9] == ':'))
        {    
            i = i + 9U + 4U;
            if(rxbuffer[i] == '-')
            {
                return LTE_AT_FAILED_TO_OPEN_NW;
            }
            else
            {
                ret = rxbuffer[i] - '0';
            }
        }
    }
    
    return ret;
}


static uint8_t qmt_disconnect_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'D') && (rxbuffer[i + 5] == 'I') &&           
           (rxbuffer[i + 6] == 'S') && (rxbuffer[i + 7] == 'C') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            if(rxbuffer[i] == '0')
            {
                ret = 0;
            }
            else
            {
                ret = LTE_AT_FAILED_TO_CLOSE_NW;
            }
        }
    }
    
    return ret;
}

static uint32_t qmt_publish_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
               (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
               (rxbuffer[i + 4] == 'P') && (rxbuffer[i + 5] == 'U') &&           
               (rxbuffer[i + 6] == 'B') && (rxbuffer[i + 7] == ':'))
            {    
                i = i + 7U + 6U;
                ret = (rxbuffer[i] - '0');
                
            }
            break;
        }
    }
    
    return ret;
}

static uint32_t qmt_sub_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'U') &&           
           (rxbuffer[i + 6] == 'B') && (rxbuffer[i + 7] == ':'))
        {    
            i = i + 7U + 6U;
            ret = (rxbuffer[i] - '0');
            
        }
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'T') &&           
           (rxbuffer[i + 6] == 'A') && (rxbuffer[i + 7] == 'T') &&
           (rxbuffer[i + 8] == ':'))
        { 
            ret = 4U;
        }
    }
    
    return ret;
}

static uint32_t qmt_cgreg_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'C') &&
           (rxbuffer[i + 2] == 'G') && (rxbuffer[i + 3] == 'R') &&
           (rxbuffer[i + 4] == 'E') && (rxbuffer[i + 5] == 'G') &&           
           (rxbuffer[i + 6] == ':'))
        {    
            i = i + 10U;
            ret = (rxbuffer[i] - '0');
            
        }
    }
    
    return ret;
}

static uint32_t qmt_qgpsloc_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'T') &&           
           (rxbuffer[i + 6] == 'A') && (rxbuffer[i + 7] == 'T') &&
           (rxbuffer[i + 8] == ':'))
        {    
            
            ret = 9U;
            
        }
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'G') && (rxbuffer[i + 3] == 'P') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'L') &&           
           (rxbuffer[i + 6] == 'O') && (rxbuffer[i + 7] == 'C') &&
           (rxbuffer[i + 8] == ':'))
        {    
            ret = 1U; 
        }
        else
        {
            ret = 0U;
        }
    }
    
    return ret;    
}

static uint32_t qmt_qgpsnmea_rmc_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'T') &&           
           (rxbuffer[i + 6] == 'A') && (rxbuffer[i + 7] == 'T') &&
           (rxbuffer[i + 8] == ':'))
        {    
            
            ret = 9U;
            
        }
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'G') && (rxbuffer[i + 3] == 'P') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'G') &&           
           (rxbuffer[i + 6] == 'N') && (rxbuffer[i + 7] == 'M') &&
           (rxbuffer[i + 8] == 'E') && (rxbuffer[i + 9] == 'A') &&
           (rxbuffer[i + 10] == ':'))
        {    
            ret = 1U; 
        }
        else
        {
            ret = 0U;
        }
    }
    
    return ret;    
}

static uint32_t qmt_csq_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'C') &&
           (rxbuffer[i + 2] == 'S') && (rxbuffer[i + 3] == 'Q') &&
           (rxbuffer[i + 4] == ':'))
        {    
            ret = 1;
        }
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'C') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'E') &&
           (rxbuffer[i + 4] == ' ') && (rxbuffer[i + 5] == 'E') &&           
           (rxbuffer[i + 6] == 'R') && (rxbuffer[i + 7] == 'R') &&
           (rxbuffer[i + 8] == 'O') && (rxbuffer[i + 9] == 'R') &&
           (rxbuffer[i + 10] == ':'))
        {    
            ret = 0U; 
        }

    }
    
    return ret;    
}

static uint32_t qiact_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'I') && (rxbuffer[i + 3] == 'A') &&
           (rxbuffer[i + 4] == 'C') && (rxbuffer[i + 5] == 'T') &&           
           (rxbuffer[i + 6] == ':'))
        {    
            i = i + 10U;
            ret = (rxbuffer[i] - '0');
        }

    }
    
    return ret;    
}



static uint32_t qmt_conn_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'C') && (rxbuffer[i + 5] == 'O') &&           
           (rxbuffer[i + 6] == 'N') && (rxbuffer[i + 7] == 'N') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            ret = (rxbuffer[i] - '0') & 0xFF;
            ret |= ((rxbuffer[i + 2U] - '0') & 0xFF) << 4U;

        }
    }
    
    return ret;
}

static uint32_t qmt_disc_rsp_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'D') && (rxbuffer[i + 5] == 'I') &&           
           (rxbuffer[i + 6] == 'S') && (rxbuffer[i + 7] == 'C') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            if(rxbuffer[i] == '0')
            {
                ret = 0;
            }
            else
            {
                ret = 1;
            }

        }
    }
    
    return ret;
}


static uint8_t buffer_contains_spcl_char(uint8_t *rxbuffer, uint32_t len)
{
    /* Check rxbuffer for '>' */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '>')
        {
            flag = 1U;
            break;
        }
    }
    
    return flag;    
}

uint32_t lte_atc_get_rsp(uint32_t cmd_idx)
{
    uint32_t ret = 0U;
    
    DEV_ASSERT(cmd_idx < LTE_MAX_AT_CMDS);
    
    ret = at_cmd_list[cmd_idx].success_rsp;
    
    return ret;
}

uint8_t is_atc_error(uint8_t *rxbuffer, uint32_t len)
{
    /* Check rxbuffer for ERROR */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    
    if(len > 5U)
    {
        for(i = 0; i < len - 5U; i++)
        {
            if((rxbuffer[i] == 'E') && (rxbuffer[i + 1] == 'R') &&
               (rxbuffer[i + 2] == 'R') && (rxbuffer[i + 3] == 'O') &&
               (rxbuffer[i + 4] == 'R'))
            {
                flag = 1U;
                break;
            }
        }
    }
    
    return flag;
}

uint8_t is_atc_cme_error(uint8_t *err_cause, uint8_t *rxbuffer, uint32_t len)
{
    /* Check rxbuffer for +CME ERROR */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    
    if(len > 11U)
    {
        for(i = 0; i < len - 11U; i++)
        {
            if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'C') &&
               (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'E') &&
               (rxbuffer[i + 4] == ' ') && (rxbuffer[i + 5] == 'E') &&           
               (rxbuffer[i + 6] == 'R') && (rxbuffer[i + 7] == 'R') &&
               (rxbuffer[i + 8] == 'O') & (rxbuffer[i + 9] == 'R') &&
               (rxbuffer[i + 10] == ':'))
            {
                i = i + 2;
                *err_cause = rxbuffer[i];
                flag = 1U;
                break;
            }
        }
    }
    
    return flag;
}

uint32_t qmt_rsp_recv_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'R') && (rxbuffer[i + 5] == 'E') &&           
           (rxbuffer[i + 6] == 'C') && (rxbuffer[i + 7] == 'V') &&
           (rxbuffer[i + 8] == ':'))
        {    
            for(i = 14U; i < 14U + 9U; i = i + 2U)
            {
                if(rxbuffer[i] == '0')
                {
                    ret = 5U;
                }
                else
                {
                    ret = i;
                    break;
                }
            }
        }
    }
    
    return ret;    
}

static uint32_t qmt_rsp_recv_read_parse(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'R') && (rxbuffer[i + 5] == 'E') &&           
           (rxbuffer[i + 6] == 'C') && (rxbuffer[i + 7] == 'V') &&
           (rxbuffer[i + 8] == ':'))
        {    
            

        }
    }
    
    return ret;   
}

uint8_t is_atc_ok(uint8_t *rxbuffer, uint32_t len)
{
    /* Check rxbuffer for OK */
    uint32_t i = 0U;
    uint32_t flag = 0U;
    

    for(i = 0; i < len - 2U; i++)
    {
        if(rxbuffer[i] == 'O' && rxbuffer[i + 1] == 'K')
        {
            flag = 1U;
            break;
        }
    }
    
    return flag;
}

lte_ret_codes lte_at_parse_response(uint32_t cmd_idx, uint8_t *rxbuffer, uint32_t len)
{
    DEV_ASSERT(cmd_idx < LTE_MAX_AT_CMDS);
    
    lte_ret_codes ret = LTE_AT_ERR;
    
    if(at_cmd_list[cmd_idx].resp_parser != NULL)
    {
        ret = at_cmd_list[cmd_idx].resp_parser(rxbuffer, len);
    }
    else
    {
        ret = at_cmd_list[cmd_idx].success_rsp;
    }
    
    return ret;
}

uint8_t is_qmtstat_urc(uint8_t *rxbuffer, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t flag = 0U; 
    uint8_t ret = 0U;    
    
    for(i = 0; i < len; i++)
    {
        if(rxbuffer[i] == '+')
        {
            flag = 1U;
            break;
        }
    }
    
    if(flag == 1U)
    {
        
        if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'S') && (rxbuffer[i + 5] == 'T') &&           
           (rxbuffer[i + 6] == 'A') && (rxbuffer[i + 7] == 'T') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            ret = (rxbuffer[i] - '0');
            
        }
        else if((rxbuffer[i] == '+') && (rxbuffer[i + 1] == 'Q') &&
           (rxbuffer[i + 2] == 'M') && (rxbuffer[i + 3] == 'T') &&
           (rxbuffer[i + 4] == 'R') && (rxbuffer[i + 5] == 'E') &&           
           (rxbuffer[i + 6] == 'C') && (rxbuffer[i + 7] == 'V') &&
           (rxbuffer[i + 8] == ':'))
        {    
            i = i + 8U + 4U;
            ret = 5U;
        }
        else
        {
            __NOP();
        }
    }
    
    return ret;    
}



void lte_atc_sem_create(void)
{
    OSIF_SemaCreate(&lte_atc_sem, 1U);
}

status_t lte_atc_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = OSIF_SemaWait(&lte_atc_sem, timeout);
    return s;
}

status_t lte_atc_sem_release(void)
{
    status_t s;
    s = OSIF_SemaPost(&lte_atc_sem);
    return s;
}
#endif /* USE_NATIVE_LTE */

