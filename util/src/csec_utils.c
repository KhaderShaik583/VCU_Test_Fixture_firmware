/*
 * Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * csec_utils.c
 *
 *  Created on: Nov 10, 2016
 *      Author: B50609
 */

#include <stdint.h>
#include <stdbool.h>

#include "csec_utils.h"

#ifdef USE_FEATURE_CSEC_UTILS
bool get_uid(uint8_t *uid);
    
/* AuthId is the MASTER_ECU key */
static uint8_t g_emptyKey[16]  = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t g_authIdKey[16] = {0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0aU, 0x0bU, 0x0cU, 0x0dU, 0x0eU, 0x0fU};

static csec_key_id_t g_authId = CSEC_MASTER_ECU;

/* Constants defined by the SHE spec */
static uint8_t key_update_enc_c[16] = {0x01U, 0x01U, 0x53U, 0x48U, 0x45U, 0x00U, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xb0U};
static uint8_t key_update_mac_c[16] = {0x01U, 0x02U, 0x53U, 0x48U, 0x45U, 0x00U, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xb0U};
static uint8_t key_debug_key_c[16]  = {0x01U, 0x03U, 0x53U, 0x48U, 0x45U, 0x00U, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xb0U};

/* Derives a key with a given constant */
static status_t derive_key(const uint8_t key[], uint8_t constant[], uint8_t derivedKey[])
{
    uint8_t concat[32];
    int32_t i = 0;

    for (i = 0; i < 16; i++)
    {
        concat[i] = key[i];
        concat[i+16] = constant[i];
    }

    return CSEC_DRV_MPCompress(concat, 2U, derivedKey, 1U);
}

/* Computes the M1-M3 values */
static status_t computeM1M2M3(uint8_t authKey[], csec_key_id_t authId, csec_key_id_t keyId, const uint8_t key[], uint32_t counter,
                                uint8_t uid[], uint8_t m1[], uint8_t m2[], uint8_t m3[])
{
    status_t stat = STATUS_ERROR;
    int32_t i = 0;
    
    uint8_t iv[16]      = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t k1[16]      = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t k2[16]      = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m2Plain[32] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 
                           0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    
    uint8_t m1m2[48]    = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                           0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                           0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    /* Derive K1 and K2 from AuthID */
    (void)derive_key(authKey, key_update_enc_c, k1);
    (void)derive_key(authKey, key_update_mac_c, k2);

    /* Compute M1 = UID | ID | AuthID */
    for (i = 0; i < 15; i++)
    {
        m1[i] = uid[i];
    }
    m1[15] = (uint8_t)((keyId & 0xFU) << 4U) | (authId & 0xFU);

    /* Compute M2 (C = counter, F = 0) */
    for(i = 0; i < 16; i++)
    {
        m2Plain[i] = 0U;
        m2Plain[16 + i] = key[i];
    }
    m2Plain[0] = (counter & (uint32_t)0x0FF00000U) >> 20U;
    m2Plain[1] = (counter & (uint32_t)0x000FF000U) >> 12U;
    m2Plain[2] = (counter & (uint32_t)0x00000FF0U) >> 4U;
    m2Plain[3] = (uint8_t)((counter & (uint32_t)0xFU) << 4U);

    /* Encrypt M2 */
    stat = CSEC_DRV_LoadPlainKey(k1);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    stat = CSEC_DRV_EncryptCBC(CSEC_RAM_KEY, m2Plain, 32U, iv, m2, 1U);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    /* Compute M3 as CMAC(key=k2, m1|m2)*/
    for (i = 0; i < 16; i++)
    {
        m1m2[i] = m1[i];
    }
	
    for(i = 0; i < 32; i++)
    {
        m1m2[16 + i] = m2[i];
    }

    stat = CSEC_DRV_LoadPlainKey(k2);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    stat = CSEC_DRV_GenerateMAC(CSEC_RAM_KEY, m1m2, 384U, m3, 1U);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    return STATUS_SUCCESS;
}

#if 0 /* Unused function warning */
/* Computes the M4 and M5 values */
static status_t computeM4M5(csec_key_id_t authId, csec_key_id_t keyId, const uint8_t *key, uint32_t counter,
                                uint8_t *uid, uint8_t *m4, uint8_t *m5)
{
    status_t stat;
    int i;

    uint8_t k3[16];
    uint8_t k4[16];

    uint8_t m4StarPlain[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t m4StarCipher[16];

    /* Derive K4 and K5 from key ID */
    deriveKey(key, key_update_enc_c, k3);
    deriveKey(key, key_update_mac_c, k4);

    m4StarPlain[0] = (counter & 0xFF00000) >> 20;
    m4StarPlain[1] = (counter & 0xFF000) >> 12;
    m4StarPlain[2] = (counter & 0xFF0) >> 4;
    m4StarPlain[3] = (uint8_t)((counter & 0xF) << 4) | 0x8;

    /* Encrypt M4* */
    stat = CSEC_DRV_LoadPlainKey(k3);
    if (stat != STATUS_SUCCESS)
        return stat;

    stat = CSEC_DRV_EncryptECB(CSEC_RAM_KEY, m4StarPlain, 16U, m4StarCipher, 1U);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    /* Compute M4 = UID | ID | AuthID | M4* */
    for (i = 0; i < 15; i++)
    {
        m4[i] = uid[i];
    }
	
    m4[15] = (uint8_t)((keyId & 0xF) << 4) | (authId & 0xF);
    for (i = 0; i < 16; i++)
    {
        m4[16 + i] = m4StarCipher[i];
    }

    stat = CSEC_DRV_LoadPlainKey(k4);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    stat = CSEC_DRV_GenerateMAC(CSEC_RAM_KEY, m4, 256U, m5, 1U);
    if (stat != STATUS_SUCCESS)
	{
        return stat;
	}

    return STATUS_SUCCESS;
}
#endif /* 0 */

/* Sets the AuthID key (MASTER_ECU_KEY) for the first time */
bool setAuthKey(void)
{
    uint8_t uid[15] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m1[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m2[32]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m3[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m4[32]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m5[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    status_t stat;

    stat = computeM1M2M3(g_emptyKey, g_authId, CSEC_MASTER_ECU, g_authIdKey, 1, uid, m1, m2, m3);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    stat = CSEC_DRV_LoadKey(CSEC_MASTER_ECU, m1, m2, m3, m4, m5);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    return true;
}

/* Extracts the UID. */
bool get_uid(uint8_t uid[])
{
    status_t stat;
    uint8_t challenge[16] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U};
    uint8_t sreg;
    uint8_t mac[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t verif[32] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    bool verifStatus;
    uint8_t i = 0U;

    stat = CSEC_DRV_GetID(challenge, uid, &sreg, mac);
    if(stat != STATUS_SUCCESS)
	{
        return false;
	}

    for (i = 0U; i < 16U; i++) 
    {
        verif[i] = challenge[i];
    }
	
    for (i = 0U; i < 15U; i++) 
    {
        verif[16 + i] = uid[i];
    }
	
    verif[31] = CSEC_DRV_GetStatus() | CSEC_STATUS_BUSY;

    stat = CSEC_DRV_LoadPlainKey(g_authIdKey);
    if(stat != STATUS_SUCCESS)
	{
        return false;
	}

    stat = CSEC_DRV_VerifyMAC(CSEC_RAM_KEY, verif, 256U, mac, 128U, &verifStatus, 1U);
    if(stat != STATUS_SUCCESS)
	{
        return false;
	}

    return verifStatus;
}

/* Erases all the keys. */
bool eraseKeys(void)
{
    status_t stat;
    uint8_t challenge[16]   = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t auth[16]        = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t authPlain[31]   = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t k[16]           = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t uid[15]         = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    uint8_t i = 0U;

    CSEC_DRV_InitRNG();
    (void)get_uid(uid);

    (void)derive_key(g_authIdKey, key_debug_key_c, k);

    stat = CSEC_DRV_LoadPlainKey(k);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    stat = CSEC_DRV_DbgChal(challenge);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    for (i = 0U; i < 16U; i++)
    {
        authPlain[i] = challenge[i];
    }
	
    for (i = 0U; i < 15U; i++)
    {
        authPlain[i + 16] = uid[i];
    }

    stat = CSEC_DRV_GenerateMAC(CSEC_RAM_KEY, authPlain, 248U, auth, 1U);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    stat = CSEC_DRV_DbgAuth(auth);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    return true;
}

/* Loads/updates a non-volatile key. */
bool loadKey(csec_key_id_t keyId, uint8_t keyNew[], uint8_t counter)
{
    uint8_t uid[15] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m1[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m2[32]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m3[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m4[32]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t m5[16]  = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    status_t stat;

    stat = computeM1M2M3(g_authIdKey, g_authId, keyId, keyNew, counter, uid, m1, m2, m3);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    stat = CSEC_DRV_LoadKey(keyId, m1, m2, m3, m4, m5);
    if (stat != STATUS_SUCCESS)
	{
        return false;
	}

    return true;
}
#endif /* USE_FEATURE_CSEC_UTILS */
