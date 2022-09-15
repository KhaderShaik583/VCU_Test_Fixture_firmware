 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2019] - [2020] Ultraviolette Automotive Pvt. Ltd.
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
 * Author : Rishi F. [011]
 *
 */
 
#ifndef UDP_TASK_H
#define UDP_TASK_H

#include <string.h>
#include "board.h"
#include "fw_common.h"

#include "lwip/igmp.h"
#include "lwip/opt.h"
#include "udp_task.h"

#include "imu.h"

#if LWIP_NETCONN

#include "lwip/api.h"
#include "lwip/sys.h"

#define MAX_UDP_MSG_LEN (512U)

#define UDP_IMX_TASK_NAME           "thread_udp"
#define UDP_IMX_TASK_STACK_SIZE     STACK_SIZE(224U)

/* S32 to IMX messages */
#define MSG_ID_SWIF_S32_IMX                 (0x10070010U)
#define MSG_ID_DBG_AUX_S32_IMX              (0x10070027U)
#define MSG_ID_IMX_PWR_OFF_S32_IMX          (0x100700A0U)
#define MSG_ID_ABS_MODE_UPD_S32_IMX         (0x300A0B05U)
#define MSG_ID_ABS_MENU_CTRL_S32_IMX        (0x300A0B15U)
#define MSG_ID_CHARGER_EVT_S32_IMX          (0xC00001A5U)
#define MSG_ID_RIDE_MODE_CHANGE_EVT_S32_IMX (0x600002F0U)
#define MSG_ID_RIDE_MODE_NA_EVT_S32_IMX     (0x600002F3U)

/* IMX to S32 messages */
#define MSG_ID_BRIGHTNESS_CTRL_IMX_S32      (0x30001B00U)
#define MSG_ID_ABS_MODE_CTRL_IMX_S32        (0x300A0B05U)
#define MSG_ID_CLU_RDY_IMX_S32              (0x30001B5FU)
#define MSG_ID_TRIP_RESET_IMX_S32           (0x3000175CU)
#define MSG_ID_GET_REGEN_LEVEL_IMX_S32      (0x300017ACU)

/* S32 to LTE messages */
#define MSG_ID_DBG_S32_LTE                  (0x10070037U)
#define MSG_ID_DL_RESUME_S32_LTE            (0x10070038U)
#define MSG_ID_FW_VER_S32_LTE               (0x10070039U)
#define MSG_ID_ODO_INFO_S32_LTE             (0x1007003AU)
#define MSG_ID_IMU_DATA_S32_LTE             (0x100700C7U)
#define MSG_ID_GRACEFUL_SHUTDOWN_S32_LTE    (0x100700D7U)
#define MSG_ID_RTC_INFO_S32_LTE             (0x1007007CU)
#define MSG_ID_TMAP_INFO_S32_LTE            (0x10070073U)
#define MSG_ID_MEM_INFO_S32_LTE             (0x100700CCU)

/* Queue messages */
#define LTE_S32_UDP_FILE_UPD_START          (0x20001A00U)
#define LTE_S32_UDP_FILE_UPD_STOP           (0x20001A01U)
#define LTE_S32_UDP_MSG_FW_DL_START_NTF     (0x20001A02U)
#define LTE_S32_UDP_MSG_FW_DL               (0x20001A03U)
#define LTE_S32_UDP_MSG_FW_DL_DONE          (0x20001A04U)
#define LTE_S32_UDP_MSG_FW_VER_INFO         (0x20001A05U)
#define LTE_S32_UDP_MSG_ODO_INFO            (0x20001A06U)
#define LTE_S32_UDP_ODO_RESET               (0x20001A07U)    
#define LTE_S32_UDP_CALIB_IMU               (0x20001A08U)
#define LTE_S32_UDP_SET_TIME                (0x20001A09U)
#define LTE_S32_UDP_FILE_DL_START           (0x20001A10U)
#define LTE_S32_UDP_FILE_DL_STOP            (0x20001A11U)
#define LTE_S32_UDP_MISC                    (0xBEEFBABEU)
#define LTE_S32_UDP_GET_RTC_TIME            (0x20001BEFU)

/* Thread Flags */
#define UDP_IMX_TASK_START_FLAG     (0x0001U)
    
typedef struct 
{
    uint32_t cmd;
    uint32_t len;
}udp_msg_t;

typedef struct 
{
    uint32_t cmd;
    uint32_t len;
    uint8_t buffer[512];
}udp_rx_msg_t;

typedef struct 
{
    uint32_t cmd;
    uint32_t len;
    uint8_t buffer[512];
}udp_rx_fw_msg_t;

typedef struct 
{
    uint32_t msg_id;
    uint32_t msg_len;
    void *msg;
}udp_msg_queue_obj_t;

typedef struct 
{
    udp_msg_t msg;
    uint8_t buffer[sizeof(imu_data_t)];
}imu_udp_msg_t;

typedef enum 
{
    ROUTE_SRC_S32_DST_LTE = 0U,
    ROUTE_SRC_S32_DST_IMX,
}route_e;

extern osif_msg_queue_id_t udp_msg_queue;


osThreadId_t udp_task_imx_get_id(void);

void udp_task_imx_init(void);
void udp_task_kill(void);
void udp_flush_messages(void);

#endif /* LWIP_NETCONN */
#endif /* UDP_TASK_H */
