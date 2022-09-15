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

#ifndef LTE_ATC_H
#define LTE_ATC_H

#include <stdio.h>

#include "fw_common.h"
#include "lpuart_driver.h" 
#include "rtx_os.h"
#include "batterypack.h" 

#define AT_CR                       (0x0dU)
#define AT_LF                       (0x0aU)

#define LTE_ATC_STATE_SEND          (0U)
#define LTE_ATC_STATE_WAIT          (1U)
#define LTE_ATC_STATE_DONE          (2U)

#define LTE_AT_CALLBACK_FMT1        (0U)
#define LTE_AT_CALLBACK_FMT2        (1U)
#define LTE_AT_CALLBACK_FMT3        (2U)
#define LTE_AT_CALLBACK_FXDL        (3U)

#define LTE_RX_CB_STATE_AT_CMD_START_PHASE        (0U)
#define LTE_RX_CB_STATE_AT_CMD_PAYLOAD_PHASE      (1U)
#define LTE_RX_CB_STATE_AT_CMD_OK_PHASE           (2U)
#define LTE_RX_CB_STATE_AT_CMD_DONE_PHASE         (3U)
#define LTE_RX_CB_STATE_AT_CMD_ERR_PHASE          (4U)

#define MAX_AT_CMD_LEN      (128U)
#define MAX_AT_RING_BUF_SZ  (512U)

#define LTE_AT_FAILED_TO_OPEN_NW        (6U)
#define LTE_AT_FAILED_TO_CLOSE_NW       (7U)
#define LTE_AT_MQTT_ID_OCCUPIED         (2U)
#define LTE_AT_FAILED_PDP_ACTIVATION    (3U)
#define LTE_AT_NW_CONN_ERROR            (5U)

#define MAX_SCR_BUF_LEN                 (512U)

typedef struct
{
    char atccmd[MAX_AT_CMD_LEN];
    char at_exp_rsp0[MAX_AT_CMD_LEN];
    char at_exp_rsp1[MAX_AT_CMD_LEN];
    uint32_t atcmd_tx_len;
    uint32_t atcmd_rx_len;
    uint32_t success_rsp;
    uint32_t timeout_ms;
    uint32_t (*resp_parser)(uint8_t *rxbuffer, uint32_t len);
}at_cmd_t;

typedef enum 
{
    LTE_AT_OK = 0,
    LTE_AT_WAIT,
    LTE_AT_ERR,
    LTE_AT_HW_ERR,
    LTE_AT_ROP,
    LTE_AT_TIMEOUT,
}lte_ret_codes;

typedef enum
{
    LTE_AT_CMD_ATE0 = 0U,
    LTE_AT_CMD_AT,
    LTE_AT_CMD_URCPORT_CFG,
    LTE_AT_CMD_CGREG,
    /*LTE_AT_CMD_CGSN,*/
    /*LTE_AT_CMD_ATI,*/
    LTE_AT_CMD_QCFG_NW_SCAN_MODE,
    LTE_AT_CMD_QCFG_SVC_DOMAIN,
    LTE_AT_CMD_CFG_APN_PDP,
    LTE_AT_CMD_PDP_DEACT,
    LTE_AT_CMD_ACT_PDP,
    LTE_AT_CMD_AT_BREAK,
    
    LTE_AT_CMD_QGPS,
    LTE_AT_CMD_CONFIG_AUDIO_MODE,
    LTE_AT_CMD_CONFIG_AUDIO,
    LTE_AT_CMD_PLY_BOOT_TONE,
    LTE_AT_CMD_QMTCFG_SSL,
    LTE_AT_CMD_MQTT_VSN,
    LTE_AT_CMD_MQTT_TIMEOUT,
    LTE_AT_CMD_MQTT_RX_CFG,
    LTE_AT_CMD_QSSLCFG_CACERT,
    LTE_AT_CMD_QSSLCFG_CLIENTCERT,
    LTE_AT_CMD_QSSLCFG_USR_KEY,
    
    LTE_AT_CMD_QSSLCFG_SECLVL,
    LTE_AT_CMD_QSSLCFG_SSLVER,
    LTE_AT_CMD_QSSLCFG_CIPHERSUITE,
    LTE_AT_CMD_QSSLCFG_IGNLOCALTIME,
    

    LTE_AT_CMD_QMTOPEN,
    LTE_AT_CMD_QMTCONN,
    LTE_AT_CMD_QMTSUB,
    LTE_AT_CMD_QMTPUB,
    LTE_AT_CMD_QMTRECV,
    LTE_AT_CMD_QMTRECV_READ,
    LTE_AT_CMD_QMTCLOSE,
    LTE_AT_CMD_QGPSNMEA_RMC,
    LTE_AT_CMD_QGPSNMEA_VTG,
    LTE_AT_CMD_QMTDISC,
    LTE_AT_CMD_CSQ,
    
    LTE_AT_CMD_QUERY_PDP_STATE,
    
    LTE_MAX_AT_CMDS,
    
}lte_at_cmd_e;

void lte_atc_init(void);
void lte_at_rst_sm_ctxt(uint32_t lte_internal_state_change);
void lte_install_rx_cb(uint32_t cb);
lte_ret_codes lte_send_data_blocking(uint8_t *buffer, uint32_t tx_len, uint8_t *rxbuffer, uint32_t *rx_len);
lte_ret_codes lte_at_txrx_nonblocking(uint32_t cmd);
void lte_send_cmd_blocking(uint8_t *buffer, uint32_t tx_len, uint32_t rx_len);
lte_ret_codes lte_at_parse_response(uint32_t cmd_idx, uint8_t *rxbuffer, uint32_t len);
lte_ret_codes lte_at_mqtt_pub(char *data,  uint32_t data_len);
uint8_t is_atc_ok(uint8_t *rxbuffer, uint32_t len);
uint8_t is_qmtstat_urc(uint8_t *rxbuffer, uint32_t len);
uint8_t is_atc_error(uint8_t *rxbuffer, uint32_t len);
uint8_t is_atc_cme_error(uint8_t *err_cause, uint8_t *rxbuffer, uint32_t len);
void init_at_cmd_list(void);
void lte_atc_read_gps_data(char *gps_data);
void lte_uart_reconfig(void);
uint32_t lte_atc_get_rsp(uint32_t cmd_idx);
lte_ret_codes lte_atc_at_start(uint32_t cmd_idx, uint8_t *rxbuffer, uint32_t *rx_len);
void lte_atc_form_qmt_read_cmd(uint32_t recv_idx);

void lte_atc_sem_create(void);
status_t lte_atc_sem_acquire(uint32_t timeout);
status_t lte_atc_sem_release(void);

#endif /* LTE_ATC_H */
