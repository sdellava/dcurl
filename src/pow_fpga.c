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
#define FLAG_RUNNING                    (1<<0)
#define FLAG_FOUND                      (1<<1)
#define FLAG_OVERFLOW                   (1<<2)
#define FLAG_CURL_FINISHED              (1<<3)


#define FLAG_START                      (1<<0)
#define FLAG_CURL_RESET                 (1<<1)
#define FLAG_CURL_WRITE                 (1<<2)
#define FLAG_CURL_DO_CURL               (1<<3)

#define CMD_NOP                         0x00000000
#define CMD_WRITE_FLAGS                 0x80000000       
#define CMD_RESET_WRPTR                 0x40000000
#define CMD_WRITE_DATA                  0x20000000
#define CMD_WRITE_MIN_WEIGHT_MAGNITUDE  0x10000000
#define CMD_READ_FLAGS                  0x08000000
#define CMD_READ_NONCE                  0x04000000

static uint32_t parallel = 0;
static uint32_t log2 = 0;

int pow3(int a) {
        int res = 1;
        for (int i=0;i<a;i++) {
                res *= 3;
        }
        return res;
}

void set_parallel(uint32_t p) {
    parallel = p;
}

uint32_t get_parallel() {
    return parallel;
}

void set_log2(uint32_t u) {
    log2 = u;
}

uint32_t get_log2() {
    return log2;
}

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
    
    rcv = reverse(rcv);
//    printf("sent: %08x received: %08x\n",cmd, rcv);
    return rcv;
}

// write flags
void cmd_write_flags(char flag_start) {
    uint32_t cmd = CMD_WRITE_FLAGS;
    
    if (flag_start)
        cmd |= 0x00000001;
    
    send(cmd);
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
/*    
    uint32_t sent = sendReceive(cmd);
    if (sent != cmd) {
        printf("verify error! %08x vs %08x\n", cmd, sent);
    }*/
}

uint32_t cmd_read_parallel_level() {
    uint32_t cmd = CMD_READ_FLAGS;
    return (sendReceive(cmd) & 0x000000f0) >> 4;
}
    
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
    uint32_t cmd = CMD_READ_FLAGS;
    return ((sendReceive(cmd) >> 8) & ((1<<parallel)-1));
}


uint32_t cmd_get_flags() {
    uint32_t cmd = CMD_READ_FLAGS;
    return sendReceive(cmd) & 0x0000000f;
}

void init_cs() {
    bcm2835_gpio_set(CS);
    delay(10);
    bcm2835_gpio_clr(CS);
    delay(10);
    bcm2835_gpio_set(CS);
    delay(10);
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


int pow_fpga_init()
{
    if (!bcm2835_init()) 
        return 0;   /* init library */
    
    /* init spi interface */
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      /* default */
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   /* default */
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                      /* default */
		    
    bcm2835_gpio_fsel(CS,BCM2835_GPIO_FSEL_OUTP);
		  
    init_cs();
    // read parallel level from FPGA
    set_parallel(cmd_read_parallel_level());
    
    if (get_parallel() > 31) {
        printf("illegal parallel level detected!\n");
        return 0;
    }

    // log2(parallel)
    uint32_t log2 = 0;
    for (int i=0;i<32;i++) {
        if (get_parallel() & (1<<i)) {
            log2 = i;
        }
    }
    set_log2(log2);
    
    printf("parallel level detected: %u\n", get_parallel());
    return 1;
}

void pow_fpga_destroy()
{
    bcm2835_spi_end();
}


// convert and send block of data to FPGA mid-state
void send_data(int8_t* data) {
    cmd_reset_wrptr();

    for (uint32_t i=0;i<243/9;i++) {
        uint32_t tritslo = 0;
        uint32_t tritshi = 0;
        for (uint32_t j=0;j<9;j++) {
            tritslo |= trit_to_bit_lo(data[i*9+j]) << j;
            tritshi |= trit_to_bit_hi(data[i*9+j]) << j;
        }
        cmd_write_data(tritshi, tritslo);
    }    
}    

// write curl mid-state on FPGA and perform curl optionally
void cmd_curl_send_block(int8_t* data, uint8_t docurl) {
    send_data(data);
    
    uint32_t cmd = CMD_WRITE_FLAGS | FLAG_CURL_WRITE | ((docurl) ? FLAG_CURL_DO_CURL : 0); 
    send(cmd);
    
    while (!(cmd_get_flags() & FLAG_CURL_FINISHED)) {
        asm("nop");
    }
}

// initialize mid-state on FPGA
void cmd_curl_init_block() {
    uint32_t cmd = CMD_WRITE_FLAGS | FLAG_CURL_RESET;
    send(cmd);
    
    while (!(cmd_get_flags() & FLAG_CURL_FINISHED)) {
        asm("nop");
    }
}

int8_t *PowFPGA(int8_t *trytes, int mwm, int index)
{
    Trytes_t *trytes_t = initTrytes(trytes, 2673);
    Trits_t *tr = trits_from_trytes(trytes_t);
    
    long long start1 = current_timestamp();

    // calculate mid-state on FPGA
    cmd_curl_init_block();
    for (int blocknr = 0; blocknr < 33; blocknr++) {
        if (blocknr != 32) {
            cmd_curl_send_block(&tr->data[blocknr * 243], 1);
        } else {
            cmd_curl_send_block(&tr->data[blocknr * 243], 0);
        }
    }
    long long stop1 = current_timestamp();

    // write min weight magnitude
    cmd_write_min_weight_magnitude(mwm);
    
    // start
    cmd_write_flags(1); 
    
    long long start2 = current_timestamp();
    while (1) {
        uint32_t flags = cmd_get_flags();

        if (!(flags & FLAG_RUNNING) && ((flags & FLAG_FOUND) || flags & FLAG_OVERFLOW)) {
            break;
        }
        delay(1);
    }
    long long stop2 = current_timestamp();

    uint32_t binary_nonce = cmd_read_binary_nonce()-2; // -2 because of pipelining for speed on FPGA
    uint32_t mask = cmd_get_mask();

    printf("Found nonce: %08x (mask: %08x)\n", binary_nonce, mask);
    
    double nodespersec = (double) binary_nonce / (double) (stop2-start2) / 1000.0 * (double) get_parallel();
    printf("Mid-State Time: %4llums\n", stop1-start1);
    printf("PoW Time:       %4llums  -  MH/s: %.3f\n", (stop2-start2), nodespersec);

    if (!mask)
        return 0;

    // find set bit in mask
    int found_bit = 0;
    for (int i=0;i<parallel;i++) {
        if (mask & (1<<i)) {
            found_bit = i;
            break;
        }
    }

    // assemble nonce
    uint8_t bitslo[NonceTrinarySize] = {0};
    uint8_t bitshi[NonceTrinarySize] = {0};

    // initialize 
    for (int i=0;i<NonceTrinarySize;i++) {
        bitslo[i] = 0x1;
        bitshi[i] = 0x1;
    }

    // insert initial nonce trits bit thingies
    for (int j=0;j<=get_log2();j++) {
        bitslo[j] = (found_bit >> j) & 0x1;
        bitshi[j] = (~(found_bit >> j)) & 0x1;
    }

    // insert nonce counter
    for (int i=0;i<32;i++) {
        bitslo[NonceTrinarySize - 32 + i] = (binary_nonce >> i) & 0x1;
        bitshi[NonceTrinarySize - 32 + i] = ((~binary_nonce) >> i) & 0x1;
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
    freeTrobject(trytes_t);
    freeTrobject(nonce_t);
    freeTrobject(nonce);
    free(nonceTrits);

    return last_result;
}
