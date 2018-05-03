/* 
 * Copyright (C) 2018 dcurl Developers.
 * Copyright (C) 2016 Shinya Yagyu.
 * Use of this source code is governed by MIT license that can be
 * found in the LICENSE file.
 */

#include <bcm2835.h>

#include "pow_fpga.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "constants.h"
#include "curl.h"
#include "sys/time.h"

#define CS 5
//#define STATE_LENGTH 729
//#define HASH_LENGTH 243
#define FLAG_RUNNING (1<<0)
#define FLAG_FOUND (1<<1)
#define FLAG_OVERFLOW (1<<2)

#define CMD_NOP                         0x00000000
#define CMD_WRITE_FLAGS                 0x84000000       
#define CMD_RESET_WRPTR                 0x94000000
#define CMD_WRITE_DATA                  0x88000000
#define CMD_WRITE_MIN_WEIGHT_MAGNITUDE  0x90000000
#define CMD_READ_FLAGS                  0x04000000
#define CMD_READ_NONCE                  0x0c000000
#define CMD_READ_MASK                   0x10000000
#define CMD_LOOP_TEST                   0x54123456

// TODO: read back from FPGA
#define PARALLEL 7

static uint32_t nonce_first_trits_lo[2] = {0};
static uint32_t nonce_first_trits_hi[2] = {0};


uint32_t reverse(uint32_t cmd) {
    uint32_t rev = 0x00000000;

    rev |= (cmd & 0xff000000) >> 24;
    rev |= (cmd & 0x00ff0000) >> 8;
    rev |= (cmd & 0x0000ff00) << 8;
    rev |= (cmd & 0x000000ff) << 24;

    return rev;
}

void send(uint32_t cmd) {
    uint32_t rev = reverse(cmd);
    bcm2835_gpio_clr(CS);
    bcm2835_spi_transfern((char*) &rev, 4);
    bcm2835_gpio_set(CS);
}

uint32_t sendReceive(uint32_t cmd) {
    uint32_t zero = 0x00000000;
    uint32_t rev = reverse(cmd);
    uint32_t rcv = 0x00000000;
    
    bcm2835_gpio_clr(CS);
    bcm2835_spi_transfern((char*) &rev, 4);
    bcm2835_gpio_set(CS);
    bcm2835_gpio_clr(CS);
    bcm2835_spi_transfernb((char*) &zero, (char*) &rcv, 4);
    bcm2835_gpio_set(CS);
    
    return reverse(rcv);
}

// nop
// write flags
void cmd_write_flags(char flag_start) {
    uint32_t cmd = CMD_WRITE_FLAGS;
    
    if (flag_start)
        cmd |= 0x00000001;
    
    send(cmd);
}

uint32_t cmd_read_test() {
    uint32_t cmd = CMD_LOOP_TEST;
    return sendReceive(cmd);
}

void cmd_reset_wrptr() {
    uint32_t cmd = CMD_RESET_WRPTR;
    send(cmd);
}

void cmd_write_data(uint32_t tritshi, uint32_t tritslo) {
    uint32_t cmd = CMD_WRITE_DATA;
    
    cmd |= tritslo & 0x000001ff;
    cmd |= (tritshi & 0x000001ff) << 9;
    
    send(cmd);
}

/*void cmd_read_data(uint32_t address, uint32_t *tritshi, uint32_t *tritslo) {
    uint32_t cmd = 0x08000000;
    cmd |= (address & 0x000003ff) << 16;
    
    uint32_t rcv = sendReceive(cmd);
    
    *tritslo = rcv & 0x00000007;
    *tritshi = (rcv & 0x00000700) >> 8;
}*/


uint32_t cmd_read_binary_nonce() {
    uint32_t cmd = CMD_READ_NONCE;
    return sendReceive(cmd);
}

void cmd_write_min_weight_magnitude(int bits) {
    uint32_t cmd = CMD_WRITE_MIN_WEIGHT_MAGNITUDE;
    
    if (bits > 26)
        bits = 26;
    
    // generate bitmask
    cmd |= (1<<bits)-1;
    
    send(cmd);
}


uint32_t cmd_get_mask() {
    uint32_t cmd = CMD_READ_MASK;
    return sendReceive(cmd);
}


uint32_t cmd_get_flags() {
    uint32_t cmd = CMD_READ_FLAGS;
    return sendReceive(cmd);
}

void init_cs() {
    bcm2835_gpio_set(CS);
    delay(10);
    bcm2835_gpio_clr(CS);
    delay(10);
    bcm2835_gpio_set(CS);
    delay(10);
}

void para(int8_t in[], uint32_t l[], uint32_t h[])
{
    for (int i = 0; i < STATE_LENGTH; i++) {
        switch (in[i]) {
        case 0:
            l[i] = HBITS;
            h[i] = HBITS;
            break;
        case 1:
            l[i] = LBITS;
            h[i] = HBITS;
            break;
        case -1:
            l[i] = HBITS;
            h[i] = LBITS;
            break;
        }
    }
}

int8_t lh_to_trit(uint32_t h, uint32_t l) {
    if (h && l)
        return 0;
    if (h && !l) 
	return 1;
    if (!h && l)
	return -1;
//    if (h == 1 && l == 0)
    return -128;
}

uint32_t trit_to_bit_lo(int8_t trit) {
    switch (trit) {
        case 0: return 0x1;
        case -1: return 0x1;
        case 1: return 0x0;
        default: return 0;
    }
}

uint32_t trit_to_bit_hi(int8_t trit) {
    switch (trit) {
        case 0: return 0x1;
        case -1: return 0x0;
        case 1: return 0x1;
        default: return 0;
    }
}


static int8_t *tx_to_cstate(Trytes_t *tx)
{
    Curl *c = initCurl();
    if (!c)
        return NULL;

    int8_t *c_state = (int8_t *) malloc(c->state->len);
    if (!c_state)
        return NULL;

    int8_t tyt[(transactionTrinarySize - HashSize) / 3] = {0};

    /* Copy tx->data[:(transactionTrinarySize - HashSize) / 3] to tyt */
    memcpy(tyt, tx->data, (transactionTrinarySize - HashSize) / 3);

    Trytes_t *inn = initTrytes(tyt, (transactionTrinarySize - HashSize) / 3);
    if (!inn)
        return NULL;

    Absorb(c, inn);

    Trits_t *tr = trits_from_trytes(tx);
    if (!tr)
        return NULL;

    /* Prepare an array storing tr[transactionTrinarySize - HashSize:] */
    memcpy(c_state, tr->data + transactionTrinarySize - HashSize,
           tr->len - (transactionTrinarySize - HashSize));
    memcpy(c_state + tr->len - (transactionTrinarySize - HashSize),
           c->state->data + tr->len - (transactionTrinarySize - HashSize),
           c->state->len - tr->len + (transactionTrinarySize - HashSize));

    freeTrobject(inn);
    freeTrobject(tr);
    freeCurl(c);

    return c_state;
}

static int8_t *nonce_to_result(Trytes_t *tx, Trytes_t *nonce)
{
    int rst_len = tx->len - NonceTrinarySize / 3 + nonce->len;
    int8_t *rst = (int8_t *) malloc(rst_len);
    if (!rst)
        return NULL;

    memcpy(rst, tx->data, tx->len - NonceTrinarySize / 3);
    memcpy(rst + tx->len - NonceTrinarySize / 3, nonce->data,
           rst_len - (tx->len - NonceTrinarySize / 3));

    return rst;
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}

int pow3(int a) {
        int res = 1;
        for (int i=0;i<a;i++) {
                res *= 3;
        }
        return res;
}

int pow_fpga_init()
{
    if (!bcm2835_init()) 
        return 0;   /* init library */
    
    /* init spi interface */
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      /* default */
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   /* default */
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                      /* default */
		    
    bcm2835_gpio_fsel(CS,BCM2835_GPIO_FSEL_OUTP);
		  
    init_cs();
    
    
    for (int j=0;j<=1;j++) {
        for (int i=0;i<=PARALLEL-1;i++) {
            int x = i/pow3(j);
            if (x % 3 == 0) {
                    nonce_first_trits_lo[j] |= 1<<i;
                    nonce_first_trits_hi[j] |= 1<<i;
            }
            if (x % 3 == 1) {
                    nonce_first_trits_hi[j] |= 1<<i;
            }
            if (x % 3 == 2) {
                    nonce_first_trits_lo[j] |= 1<<i;
            }
        }
    }
    
    return 1;
}

void pow_fpga_destroy()
{
    bcm2835_spi_end();
}




int8_t *PowFPGA(int8_t *trytes, int mwm, int index)
{
    Trytes_t *trytes_t = initTrytes(trytes, 2673);

    int8_t *c_state = tx_to_cstate(trytes_t);
    if (!c_state)
        return NULL;

    int offset = HASH_LENGTH - NONCE_LENGTH;


    // reset write pointer - it gets incremented automatically with every write
    cmd_reset_wrptr();
    
    // convert trits to 9byte-wise data
    for (uint32_t i=0;i<STATE_LENGTH/9;i++) {
        uint32_t tritslo = 0;
        uint32_t tritshi = 0;
        for (uint32_t j=0;j<9;j++) {
            tritslo |= trit_to_bit_lo(c_state[i*9+j]) << j;
            tritshi |= trit_to_bit_hi(c_state[i*9+j]) << j;
        }
        cmd_write_data(tritshi, tritslo);
    }

    // write min weight magnitude
    cmd_write_min_weight_magnitude(mwm);
    
    // start
    cmd_write_flags(1); 
    
    long long start = current_timestamp();
    while (1) {
        uint32_t flags = cmd_get_flags();
        if (!(flags & FLAG_RUNNING))
            break;
        delay(1);
    }
    long long stop = current_timestamp();

    uint32_t binary_nonce = cmd_read_binary_nonce()-1; // -1 comes from pipelining the counter for speed on FPGA
    uint32_t mask = cmd_get_mask();


    printf("Found nonce: %08x (mask: %08x)\n", binary_nonce, mask);
    
    double nodespersec = (double) binary_nonce / (double) (stop-start) / 1000.0 * (double) PARALLEL;
    printf("Time: %llums  -  MH/s: %.3f\n", (stop-start), nodespersec);

    if (!mask)
        return 0;

    
    // find set bit in mask
    int found_bit = 0;
    for (int i=0;i<PARALLEL;i++) {
        if (mask & (1<<i)) {
            found_bit = i;
            break;
        }
    }

    uint8_t bitslo[NonceTrinarySize] = {0};
    uint8_t bitshi[NonceTrinarySize] = {0};

    uint8_t* ptr_bitslo = &bitslo[0];
    uint8_t* ptr_bitshi = &bitshi[0];
    
    for (int i=0;i<2;i++) {
        *ptr_bitslo++ = (nonce_first_trits_lo[i] & (1 << found_bit)) ? 0x1 : 0x0;
        *ptr_bitshi++ = (nonce_first_trits_hi[i] & (1 << found_bit)) ? 0x1 : 0x0;
    }
    
    for (int i=0;i<32;i++) {
        *ptr_bitslo++ = (binary_nonce >> i) & 0x1;
        *ptr_bitshi++ = ((~binary_nonce) >> i) & 0x1;
    }

    for (int i=33;i<NonceTrinarySize;i++) {
        *ptr_bitslo++ = 0x1;
        *ptr_bitshi++ = 0x1;
    }
  
    int8_t* nonceTrits = (int8_t*) malloc(NonceTrinarySize);
    for (int i=0;i<NonceTrinarySize;i++) {
        nonceTrits[i] = lh_to_trit(bitshi[i], bitslo[i]);
    }
 
    Trits_t *nonce_t = initTrits(nonceTrits, NonceTrinarySize);
    if (!nonce_t)
        return NULL;

    Trytes_t *nonce = trytes_from_trits(nonce_t);
    if (!nonce)
        return NULL;

    int8_t *last_result = nonce_to_result(trytes_t, nonce);
    
    /* Free memory */
    free(c_state);
    freeTrobject(trytes_t);
    freeTrobject(nonce_t);
    freeTrobject(nonce);
    free(nonceTrits);

    return last_result;
}
