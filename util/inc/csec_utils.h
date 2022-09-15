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
 * csec_utils.h
 *
 *  Created on: Nov 10, 2016
 *      Author: B50609
 */

#ifndef SOURCES_CSEC_UTILS_H_
#define SOURCES_CSEC_UTILS_H_

#include "csec_driver.h"

/* This function sets the MASTER_ECU key with a key (g_authIdKey) defined in the
 * csec_utils.c file. This key will be used as an authorization secret for updating
 * user keys.
 * Setting the MASTER_ECU key will work only for the first time, in order to use
 * another value for the key, there are two options:
 * - erase the keys and then update g_authIdKey;
 * - use loadKey with counter > 1 and then update g_authIdKey;
 * */
bool setAuthKey(void);

/* This function erases all the key. After using it, the Flash needs to be partitioned
 * again.
 * */
bool eraseKeys(void);

/* This function loads/updates a non-volatile key.
 * When updating the key, the counter needs to be greater then the previous one.
 * */
bool loadKey(csec_key_id_t keyId, uint8_t *keyNew, uint8_t counter);

#endif /* SOURCES_CSEC_UTILS_H_ */
