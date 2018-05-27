#include "main.h"

#include <bcm2835.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys/time.h"

#define nCONFIG     2
#define DATA0       3
#define DCK         4
#define nSTATUS     17
#define CONFDONE    7

#define FPGA_FLASH_DATA 0

int main(int argc, char* argv[])
{
    printf("file name: %s\n", argv[1]);
    FILE* fp = fopen(argv[1], "rb");
    fseek(fp, 0L, SEEK_END);
    size_t sz = ftell(fp);
    printf("file size: %u\n", sz);
    fseek(fp, 0L, SEEK_SET);
    
    uint8_t* fpga_data = (uint8_t*) malloc(sz);
    size_t r = fread((void*) fpga_data, 1, sz, fp);
    fclose(fp);
    
    printf("read bytes: %u\n", r);
    
    if (r != sz) {
        printf("size mismatch!\n");
        return 0;
    }
    
    
    uint8_t* fpga_flash_data_end = fpga_data + sz;
    
    if (!bcm2835_init()) 
        return 0;   /* init library */
    
    bcm2835_gpio_fsel(DATA0, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(DCK, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(nCONFIG, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(nSTATUS, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(CONFDONE, BCM2835_GPIO_FSEL_INPT);
		  
    
    bcm2835_gpio_clr(DCK);  
    /* pulling FPGA_CONFIG to low resets the FPGA */
    bcm2835_gpio_clr(nCONFIG);                /* FPGA config => low */
    delay(10);                                           /* give it some time to do its reset stuff */

    while ((bcm2835_gpio_lev(nSTATUS)) && (bcm2835_gpio_lev(CONFDONE)));

    bcm2835_gpio_set(nCONFIG);
    while (!(bcm2835_gpio_lev(nSTATUS)))
        ;                                               /* wait until status becomes high */

    uint8_t* fpga_data_ptr = fpga_data;
    do
    {
        uint8_t value = *fpga_data_ptr++;
        for (int i = 0; i < 8; i++, value >>= 1)
        {
            if (value & 1)
            {
                /* bit set -> toggle DATA0 to high */
                bcm2835_gpio_set(DATA0);
            }
            else
            {
                /* bit is cleared -> toggle DATA0 to low */
                bcm2835_gpio_clr(DATA0);
            }
            /* toggle DCLK -> FPGA reads the bit */
            bcm2835_gpio_set(DCK);
            bcm2835_gpio_clr(DCK);
        }
    } while ((!(bcm2835_gpio_lev(CONFDONE))) && (fpga_data_ptr < fpga_flash_data_end));    
    
    printf("finished\n");
    printf("confdone: %s\n", bcm2835_gpio_lev(CONFDONE) ? "true" : "false");
    
    free(fpga_data);
    
    return 1;
}

