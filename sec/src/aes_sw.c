#include "fw_features.h"

#ifdef USE_SW_AES_MOD

#include <string.h>
#include "aes_sw.h"

/**
 * File: cmac.c
 * Created on: 21 ???. 2015 ?.
 * Description:
 * 
 *
 * Author: Roman Savrulin <romeo.deepmind@gmail.com>
 * Copyright: 2015 Roman Savrulin
 * Copying permission statement:
 * 
 *  This file is part of AES-CMAC.
 *
 *  AES-CMAC is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

/****************************************************************/
/* AES-CMAC with AES-128 bit                                    */
/* CMAC     Algorithm described in SP800-38B                    */
/* Author: Junhyuk Song (junhyuk.song@samsung.com)              */
/*         Jicheol Lee  (jicheol.lee@samsung.com)               */
/****************************************************************/



#ifdef DEBUG_CMAC
#include <stdio.h>
void print_hex(const char *str, const unsigned char *buf, int32_t len) {
	int32_t i;

	for (i = 0; i < len; i++) {
		if ((i % BLOCK_SIZE) == 0 && i != 0)
			printf("%s", str);
		printf("%02x", buf[i]);
		if ((i % 4) == 3)
			printf(" ");
		if ((i % BLOCK_SIZE) == LAST_INDEX)
			printf("\n");
	}
	if ((i % BLOCK_SIZE) != 0)
		printf("\n");
}
void print128(const unsigned char *bytes) {
	int32_t j;
	for (j = 0; j < BLOCK_SIZE; j++) {
		printf("%02x", bytes[j]);
		if ((j % 4) == 3)
			printf(" ");
	}
}

void print96(const unsigned char *bytes) {
	int32_t j;
	for (j = 0; j < 12; j++) {
		printf("%02x", bytes[j]);
		if ((j % 4) == 3)
			printf(" ");
	}
}
#endif

/* For CMAC Calculation */
static unsigned const char const_Rb[BLOCK_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87 };
static unsigned const char const_Zero[BLOCK_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

int AES_CMAC_CHECK(const unsigned char *key, const unsigned char *input, int length,
		const unsigned char *mac){
	unsigned char T[BLOCK_SIZE];
	AES_CMAC(key, input, length, T);
	/*print128(T);
	printf("\n");
	print128(mac);*/
	return memcmp(mac, T, BLOCK_SIZE);
}
        
const unsigned char sbox[256] =   {
//0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, //0
0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, //1
0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, //2
0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, //3
0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, //4
0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, //5
0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, //6
0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, //7
0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, //8
0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, //9
0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, //A
0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, //B
0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, //C
0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, //D
0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, //E
0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 }; //F

// inverse sbox
const unsigned char rsbox[256] =
{ 0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb
, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb
, 0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e
, 0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25
, 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92
, 0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84
, 0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06
, 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b
, 0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73
, 0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e
, 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b
, 0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4
, 0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f
, 0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef
, 0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61
, 0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };

// round constant
const unsigned char Rcon[10] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};


// multiply by 2 in the galois field
unsigned char galois_mul2(unsigned char value)
{
	if (value>>7)
	{
		return ((value << 1)^0x1b);
	} else
		return (value << 1);
}

// AES encryption and decryption function
// The code was optimized for memory (flash and ram)
// Combining both encryption and decryption resulted in a slower implementation
// but much smaller than the 2 functions separated
// This function only implements AES-128 encryption and decryption (AES-192 and 
// AES-256 are not supported by this code) 
void aes_enc_dec(unsigned char *state, unsigned char *key, unsigned char dir)
{
  unsigned char buf1, buf2, buf3, buf4, round, i;
   
  // In case of decryption
  if (dir) {
    // compute the last key of encryption before starting the decryption
    for (round = 0 ; round < 10; round++) {
      //key schedule
      key[0] = sbox[key[13]]^key[0]^Rcon[round];
      key[1] = sbox[key[14]]^key[1];
      key[2] = sbox[key[15]]^key[2];
      key[3] = sbox[key[12]]^key[3];
      for (i=4; i<16; i++) {
        key[i] = key[i] ^ key[i-4];
      }
    }
    
    //first Addroundkey
    for (i = 0; i <16; i++){
      state[i]=state[i] ^ key[i];
    }
  }
  
  // main loop
  for (round = 0; round < 10; round++){
    if (dir){
      //Inverse key schedule
      for (i=15; i>3; --i) {
	key[i] = key[i] ^ key[i-4];
      }  
      key[0] = sbox[key[13]]^key[0]^Rcon[9-round];
      key[1] = sbox[key[14]]^key[1];
      key[2] = sbox[key[15]]^key[2];
      key[3] = sbox[key[12]]^key[3]; 
    } else {
      for (i = 0; i <16; i++){
        // with shiftrow i+5 mod 16
	state[i]=sbox[state[i] ^ key[i]];
      }
      //shift rows
      buf1 = state[1];
      state[1] = state[5];
      state[5] = state[9];
      state[9] = state[13];
      state[13] = buf1;

      buf1 = state[2];
      buf2 = state[6];
      state[2] = state[10];
      state[6] = state[14];
      state[10] = buf1;
      state[14] = buf2;

      buf1 = state[15];
      state[15] = state[11];
      state[11] = state[7];
      state[7] = state[3];
      state[3] = buf1;
    }
    //mixcol - inv mix
    if ((round > 0 && dir) || (round < 9 && !dir)) {
      for (i=0; i <4; i++){
        buf4 = (i << 2);
        if (dir){
          // precompute for decryption
          buf1 = galois_mul2(galois_mul2(state[buf4]^state[buf4+2]));
          buf2 = galois_mul2(galois_mul2(state[buf4+1]^state[buf4+3]));
          state[buf4] ^= buf1; state[buf4+1] ^= buf2; state[buf4+2] ^= buf1; state[buf4+3] ^= buf2; 
        }
        // in all cases
        buf1 = state[buf4] ^ state[buf4+1] ^ state[buf4+2] ^ state[buf4+3];
        buf2 = state[buf4];
        buf3 = state[buf4]^state[buf4+1]; buf3=galois_mul2(buf3); state[buf4] = state[buf4] ^ buf3 ^ buf1;
        buf3 = state[buf4+1]^state[buf4+2]; buf3=galois_mul2(buf3); state[buf4+1] = state[buf4+1] ^ buf3 ^ buf1;
        buf3 = state[buf4+2]^state[buf4+3]; buf3=galois_mul2(buf3); state[buf4+2] = state[buf4+2] ^ buf3 ^ buf1;
        buf3 = state[buf4+3]^buf2;     buf3=galois_mul2(buf3); state[buf4+3] = state[buf4+3] ^ buf3 ^ buf1;
      }
    }
    
    if (dir) {
      //Inv shift rows
      // Row 1
      buf1 = state[13];
      state[13] = state[9];
      state[9] = state[5];
      state[5] = state[1];
      state[1] = buf1;
      //Row 2
      buf1 = state[10];
      buf2 = state[14];
      state[10] = state[2];
      state[14] = state[6];
      state[2] = buf1;
      state[6] = buf2;
      //Row 3
      buf1 = state[3];
      state[3] = state[7];
      state[7] = state[11];
      state[11] = state[15];
      state[15] = buf1;         
           
      for (i = 0; i <16; i++){
        // with shiftrow i+5 mod 16
        state[i]=rsbox[state[i]] ^ key[i];
      } 
    } else {
      //key schedule
      key[0] = sbox[key[13]]^key[0]^Rcon[round];
      key[1] = sbox[key[14]]^key[1];
      key[2] = sbox[key[15]]^key[2];
      key[3] = sbox[key[12]]^key[3];
      for (i=4; i<16; i++) {
        key[i] = key[i] ^ key[i-4];
      }
    }
  }
  if (!dir) {
  //last Addroundkey
    for (i = 0; i <16; i++){
      // with shiftrow i+5 mod 16
      state[i]=state[i] ^ key[i];
    } // enf for
  } // end if (!dir)
} // end function

static void AES_128_ENC(unsigned const char *key, unsigned const char* msg, unsigned char *cipher){
	unsigned char key_copy[BLOCK_SIZE];
	memcpy(cipher, msg, BLOCK_SIZE);
	memcpy(key_copy, key, BLOCK_SIZE);
	aes_enc_dec(cipher, key_copy, 0);
}

void AES_128_DEC(unsigned const char *key, unsigned const char* msg, unsigned char *cipher){
	unsigned char key_copy[BLOCK_SIZE];
	memcpy(cipher, msg, BLOCK_SIZE);
	memcpy(key_copy, key, BLOCK_SIZE);
	aes_enc_dec(cipher, key_copy, 1);
}

void xor_128(const unsigned char *a, const unsigned char *b, unsigned char *out) {
	int i;
	for (i = 0; i < BLOCK_SIZE; i++) {
		out[i] = a[i] ^ b[i];
	}
}

static void padding_AES(const unsigned char *lastb, unsigned char *pad, int length) {
	int j;
	length = length % BLOCK_SIZE;

	if(length == 0){
		memcpy(pad, lastb, BLOCK_SIZE);
		return;
	}

	/* original last block */
	for (j = 0; j < BLOCK_SIZE; j++) {
		if (j < length) {
			pad[j] = lastb[j];
		} else {
			pad[j] = 0x00;
		}
	}
}

int AES_CBC_ENC(const unsigned char *IV, const unsigned char *key, const unsigned char *input, int inputLength, unsigned char *output, int outputLength){
	unsigned char X[BLOCK_SIZE], Y[BLOCK_SIZE], M_last[BLOCK_SIZE];

	if (inputLength <= 0)
		return 0; //nothing to encode

	int n = (inputLength + LAST_INDEX) / BLOCK_SIZE; //TODO: last

	memcpy(X, IV, BLOCK_SIZE);
	padding_AES(&input[BLOCK_SIZE * (n - 1)], M_last, inputLength);

	for (int i = 0; (i < n) && outputLength > 0; i++) {
		unsigned const char * text = &input[BLOCK_SIZE * i];
		if(i == n - 1){
			text = M_last;
		}
		int outLen = (BLOCK_SIZE < outputLength)?BLOCK_SIZE:outputLength;
		xor_128(X, text, Y);
		AES_128_ENC(key, Y, X);
		memcpy(output, X, outLen);
		outputLength -= outLen;
		output += outLen;
	}

	return n * BLOCK_SIZE;
}

int AES_CBC_DEC(const unsigned char *IV, const unsigned char *key, const unsigned char *input, int inputLength, unsigned char *output, int outputLength){
	unsigned char X[BLOCK_SIZE], text[BLOCK_SIZE], Z[BLOCK_SIZE];

	if (inputLength <= 0)
		return 0; //nothing to encode

	inputLength = ( inputLength / BLOCK_SIZE ) * BLOCK_SIZE;

	int n = (inputLength + LAST_INDEX) / BLOCK_SIZE;

	memcpy(Z, IV, BLOCK_SIZE);
	for (int i = 0; (i < n) && outputLength > 0; i++) {
		unsigned const char * cipher = &input[BLOCK_SIZE * i];
		AES_128_DEC(key, cipher, X);
		xor_128(Z, X, text);
		memcpy(Z, cipher, BLOCK_SIZE);
		memcpy(output, text, BLOCK_SIZE);
		outputLength -= BLOCK_SIZE;
		output += BLOCK_SIZE;
	}

	return n * BLOCK_SIZE;
}

/* AES-CMAC Generation Function */

static void leftshift_onebit(const unsigned char *input, unsigned char *output) {
	int i;
	unsigned char overflow = 0;

	for (i = LAST_INDEX; i >= 0; i--) {
		output[i] = input[i] << 1;
		output[i] |= overflow;
		overflow = (input[i] & 0x80) ? 1 : 0;
	}
	return;
}

static void generate_subkey(const unsigned char *key, unsigned char *K1, unsigned
char *K2) {
	unsigned char L[BLOCK_SIZE];
	unsigned char tmp[BLOCK_SIZE];

	AES_128_ENC(key, const_Zero, L);

	if ((L[0] & 0x80) == 0) { /* If MSB(L) = 0, then K1 = L << 1 */
		leftshift_onebit(L, K1);
	} else { /* Else K1 = ( L << 1 ) (+) Rb */

		leftshift_onebit(L, tmp);
		xor_128(tmp, const_Rb, K1);
	}

	if ((K1[0] & 0x80) == 0) {
		leftshift_onebit(K1, K2);
	} else {
		leftshift_onebit(K1, tmp);
		xor_128(tmp, const_Rb, K2);
	}
	return;
}

static void padding(const unsigned char *lastb, unsigned char *pad, int length) {
	int j;

	/* original last block */
	for (j = 0; j < BLOCK_SIZE; j++) {
		if (j < length) {
			pad[j] = lastb[j];
		} else if (j == length) {
			pad[j] = 0x80;
		} else {
			pad[j] = 0x00;
		}
	}
}

void AES_CMAC(const unsigned char *key, const unsigned char *input, int length,
		unsigned char *mac) {
	unsigned char X[BLOCK_SIZE], Y[BLOCK_SIZE], M_last[BLOCK_SIZE], padded[BLOCK_SIZE];
	unsigned char K1[BLOCK_SIZE], K2[BLOCK_SIZE];
	int n, i, flag;
	generate_subkey(key, K1, K2);

	n = (length + LAST_INDEX) / BLOCK_SIZE; /* n is number of rounds */

	if (n == 0) {
		n = 1;
		flag = 0;
	} else {
		if ((length % BLOCK_SIZE) == 0) { /* last block is a complete block */
			flag = 1;
		} else { /* last block is not complete block */
			flag = 0;
		}
	}

	if (flag) { /* last block is complete block */
		xor_128(&input[BLOCK_SIZE * (n - 1)], K1, M_last);
	} else {
		padding(&input[BLOCK_SIZE * (n - 1)], padded, length % BLOCK_SIZE);
		xor_128(padded, K2, M_last);
	}

	memset(X, 0, BLOCK_SIZE);
	for (i = 0; i < n - 1; i++) {
		xor_128(X, &input[BLOCK_SIZE * i], Y); /* Y := Mi (+) X  */
		AES_128_ENC(key, Y, X); /* X := AES-128(KEY, Y); */
	}

	xor_128(X, M_last, Y);
	AES_128_ENC(key, Y, X);

	memcpy(mac, X, BLOCK_SIZE);
}
    

void aes_sw_enc(const uint8_t *buffer, uint8_t *enc_buffer, uint16_t enc_size)
{
   uint8_t v[BLOCK_SIZE] = {0xD1, 0x62, 0xAF, 0x85, 0x20, 0xAF, 0x5C, 0x05, 0x2C, 0x23, 0x0F, 0x77, 0x94, 0xC2, 0xC3, 0xEA};
   uint8_t k[BLOCK_SIZE] = {0xAC, 0x9E, 0x1C, 0xD7, 0x12, 0x75, 0x75, 0xDC, 0x04, 0xF5, 0x2D, 0xB2, 0x12, 0x4D, 0x44, 0x62};
   
   AES_CBC_ENC(v, k, buffer, enc_size, enc_buffer, enc_size);
}

void aes_sw_dec(const uint8_t *buffer, uint8_t *dec_buffer, uint16_t dec_size)
{
   uint8_t v[BLOCK_SIZE] = {0xD1, 0x62, 0xAF, 0x85, 0x20, 0xAF, 0x5C, 0x05, 0x2C, 0x23, 0x0F, 0x77, 0x94, 0xC2, 0xC3, 0xEA};
   uint8_t k[BLOCK_SIZE] = {0xAC, 0x9E, 0x1C, 0xD7, 0x12, 0x75, 0x75, 0xDC, 0x04, 0xF5, 0x2D, 0xB2, 0x12, 0x4D, 0x44, 0x62};

   AES_CBC_DEC(v, k, buffer, dec_size, dec_buffer, dec_size);
}
            
void aes_sw_cmac(uint8_t *buffer, uint16_t size, uint8_t *cmac_hash)
{
    uint8_t k[BLOCK_SIZE] = {0x62, 0x9E, 0x1C, 0xD7, 0x12, 0x2D, 0x75, 0xDC, 0x04, 0xF5, 0x75, 0xB2, 0x12, 0x4D, 0x44, 0xAC};
    
    AES_CMAC(k, buffer, size, cmac_hash);
}
        
#endif /* USE_SW_AES_MOD */
