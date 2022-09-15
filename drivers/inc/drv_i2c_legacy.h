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

#ifndef DRV_I2C_LEGACY_H
#define DRV_I2C_LEGACY_H


#include "fw_common.h"
#include "lpi2c_driver.h"
#include "lpi2c_hw_access.h" 

#define I2C_IF_WRITE_CMD    0U
#define I2C_IF_READ_CMD     1U


status_t LPI2C_MasterCheckAndClearError(LPI2C_Type *base, uint32_t status);
status_t LPI2C_CheckForBusyBus(LPI2C_Type *base);
status_t LPI2C_MasterStart(LPI2C_Type *base, uint8_t address, uint32_t dir);
status_t LPI2C_MasterSend(LPI2C_Type *base, const void *txBuff, size_t txSize);
status_t LPI2C_MasterReceive(LPI2C_Type *base, uint8_t *rxBuff, size_t rxSize);
status_t LPI2C_MasterReceivePN532(LPI2C_Type *base, uint8_t *rxBuff, size_t rxSize);
status_t LPI2C_MasterStop(LPI2C_Type *base);
void LPI2C_MasterSetWatermarks(LPI2C_Type *base, size_t txWords, size_t rxWords);


#endif /* DRV_I2C_LEGACY_H */
