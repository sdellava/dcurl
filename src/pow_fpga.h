#ifndef POW_C_H_
#define POW_C_H_

#include "trinary.h"
#include <stdint.h>

int8_t *PowFPGA(int8_t *trytes, int mwm, int index);
int pow_fpga_init();
void pow_fpga_destroy();

#define HBITS 0xFFFFFFFFuL
#define LBITS 0x00000000uL
#define HASH_LENGTH 243               // trits
#define NONCE_LENGTH 81               // trits
#define STATE_LENGTH 3 * HASH_LENGTH  // trits
#define TX_LENGTH 2673                // trytes
#define INCR_START HASH_LENGTH - NONCE_LENGTH + 4 + 27

#endif
