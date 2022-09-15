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
 
#ifndef AES_H
#define AES_H

#include <stdint.h>

#define AES_BLOCK_LENGTH            (16U)
#define AES128_KEY_SIZE             (16U)
#define CMAC_HASH_SIZE              (16U)
#define CMAC_KEY_SIZE               (16U)

void aes_encrypt_buffer_can_bms_vcu(const uint8_t *plaintext, uint8_t *ciphertext, uint32_t size);
void aes_decrypt_buffer_can_bms_vcu(uint8_t *plaintext, uint8_t *ciphertext, uint32_t size);
void aes_cmacl_can_bms_vcu(const uint8_t *ciphertext, uint16_t g_length, uint8_t *cmac_hash);
void aes_cmacnvm(const uint8_t *ciphertext, uint16_t g_length, uint8_t *cmac_hash);

#endif /* __AES_H__ */
