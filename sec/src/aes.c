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
 
#include "fw_common.h"
#include "csec_driver.h"
#include "aes.h"

static uint8_t v[AES_BLOCK_LENGTH] = {0xACU, 0xF5U, 0x1BU, 0x37U, 0x5AU, 0xECU, 0xB6U, 0x14U, 
                                      0x0EU, 0x3BU, 0x83U, 0x2AU, 0x44U, 0x5DU, 0x33U, 0x9DU};

/*FUNCTION**********************************************************************
 *
 * Function Name : aes_encrypt_buffer
 * Description   : Encrypt plaintext buffer received as argument.
 * This function encrypts the buffer pointed to by plaintext using key and places
 * the encrypted data in buffer pointed to by ciphertext.
 *
 * Implements    : aes_encrypt_buffer_Activity
 *END**************************************************************************/

void aes_encrypt_buffer_can_bms_vcu(const uint8_t *plaintext, uint8_t *ciphertext, uint32_t size)
{
    status_t status = STATUS_SUCCESS;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    status = CSEC_DRV_EncryptCBC(CSEC_KEY_3, plaintext, size, v, ciphertext, 10000U);
    (void)osif_exit_critical(lc);
    
    DEV_ASSERT(status == STATUS_SUCCESS);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : aes_decrypt_buffer
 * Description   : Decrypt ciphertext buffer received as argument.
 * This function decrypts the buffer pointed to by ciphertext using key and places
 * the encrypted data in buffer pointed to by plaintext.
 *
 * Implements    : aes_decrypt_buffer_Activity
 *END**************************************************************************/

void aes_decrypt_buffer_can_bms_vcu(uint8_t *ciphertext, uint8_t *plaintext, uint32_t size)
{
    status_t status = STATUS_SUCCESS;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    status =  CSEC_DRV_DecryptCBC(CSEC_KEY_3, ciphertext, size, v, plaintext, 10000U);
    (void)osif_exit_critical(lc);

    /*    
       Below code does one retry attempt to decrypt the packet in-case the error occurs.
       Apparently the retry does not work wither. Just return from function. Next level
       of error handling in packet handler will take care of it.
    */
    if(status == STATUS_SEC_SEQUENCE_ERROR)
    {
        return;
    }
    else
    {
        DEV_ASSERT(status == STATUS_SUCCESS);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : aes_cmac
 * Description   : Computes the AES-128 CMAC for the ciphertext buffer pointed to
 * by ciphertext using key and places it in buffer pointed to by cmac_hash
 *
 * Implements    : aes_cmac_Activity
 *END**************************************************************************/


void aes_cmacl_can_bms_vcu(const uint8_t *ciphertext, uint16_t g_length, uint8_t *cmac_hash)
{
    status_t status = STATUS_SUCCESS;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    status = CSEC_DRV_GenerateMAC(CSEC_KEY_2, ciphertext, (g_length * 8U), cmac_hash, 10000U);
    (void)osif_exit_critical(lc);
    
    DEV_ASSERT(status == 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : aes_cmacnvm
 * Description   : Computes the AES-128 CMAC for the ciphertext buffer pointed to
 * by ciphertext using key and places it in buffer pointed to by cmac_hash
 *
 * Implements    : aes_cmacnvm_Activity
 *END**************************************************************************/

void aes_cmacnvm(const uint8_t *ciphertext, uint16_t g_length, uint8_t *cmac_hash)
{
    status_t status = STATUS_SUCCESS;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    status = CSEC_DRV_GenerateMAC(CSEC_KEY_2, ciphertext, (g_length * 8U), cmac_hash, 10000U);
    (void)osif_exit_critical(lc);
    
    DEV_ASSERT(status == 0);
}


