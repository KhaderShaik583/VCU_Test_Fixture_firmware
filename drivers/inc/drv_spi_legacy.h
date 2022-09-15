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
 * Author : Rishi F. [011]
 *


 */

#ifndef DRIVERS_SPI_LEGACY_H
#define DRIVERS_SPI_LEGACY_H

#include <stdint.h>

void lpspi_send_array(uint8_t *txbuffer, uint8_t len);

void lpspi_receive_array(uint8_t *rxbuffer, uint8_t len);

void lpspi_trancieve(uint8_t *tx_Data,   /*array of data to be written on SPI port */
                    uint8_t tx_len,     /*length of the tx data arry */
                    uint8_t *rx_data,   /*Input: array that will store the data read by the SPI port */
                    uint8_t rx_len      /*Option: number of bytes to be read from the SPI port */
                    );

void lpspi_trancieve_tlf_16(uint16_t *tx_Data, uint8_t tx_len, uint16_t *rx_data, uint8_t rx_len);

uint8_t lpspi_trancieve_sja_read32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);
uint8_t lpspi_trancieve_sja_write32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);

uint8_t lpspi_read_icm20948(uint8_t reg, uint8_t *rx_data, uint8_t rx_len);
uint8_t lpspi_write8_icm20948(uint8_t reg, const uint8_t *tx_data, uint8_t tx_len);
uint8_t lpspi_read8_icm20948(uint8_t reg, uint8_t *rx_data, uint8_t rx_len);

#endif /* DRIVERS_SPI_LEGACY_H */
