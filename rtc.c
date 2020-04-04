#include <string.h>
#include "libdragon.h"
#include "regsinternal.h"
#include "rtc.h"

/**
 * @name SI status register bit definitions
 * @{
 */

/** @brief SI DMA busy */
#define SI_STATUS_DMA_BUSY  ( 1 << 0 )
/** @brief SI IO busy */
#define SI_STATUS_IO_BUSY   ( 1 << 1 )
/** @} */

/** @brief Structure used to interact with SI registers */
static volatile struct SI_regs_s * const SI_regs = (struct SI_regs_s *)0xa4800000;
/** @brief Location of the PIF RAM */
static void * const PIF_RAM = (void *)0x1fc007c0;

/**
 * @brief Wait until the SI is finished with a DMA request
 */
static void __SI_DMA_wait(void)
{
    while (SI_regs->status & (SI_STATUS_DMA_BUSY | SI_STATUS_IO_BUSY)) ;
}

/**
 * @brief Send a block of data to the PIF and fetch the result
 *
 * @param[in]  inblock
 *             The formatted block to send to the PIF
 * @param[out] outblock
 *             The buffer to place the output from the PIF
 */
static void __rtc_exec_PIF( void *inblock, void *outblock )
{
    volatile uint64_t inblock_temp[8];
    volatile uint64_t outblock_temp[8];

    data_cache_hit_writeback_invalidate(inblock_temp, 64);
    memcpy(UncachedAddr(inblock_temp), inblock, 64);

    /* Be sure another thread doesn't get into a resource fight */
    disable_interrupts();

    __SI_DMA_wait();

    SI_regs->DRAM_addr = inblock_temp; // only cares about 23:0
    MEMORY_BARRIER();
    SI_regs->PIF_addr_write = PIF_RAM; // is it really ever anything else?
    MEMORY_BARRIER();

    __SI_DMA_wait();

    data_cache_hit_writeback_invalidate(outblock_temp, 64);

    SI_regs->DRAM_addr = outblock_temp;
    MEMORY_BARRIER();
    SI_regs->PIF_addr_read = PIF_RAM;
    MEMORY_BARRIER();

    __SI_DMA_wait();

    /* Now that we've copied, its safe to let other threads go */
    enable_interrupts();

    memcpy(outblock, UncachedAddr(outblock_temp), 64);
}

static uint8_t __bcd_to_byte(uint8_t bcd)
{
    uint8_t res = bcd & 0x0F;
    res += (bcd >> 4) * 10;
    return res;
}

int rtc_present(void)
{
    static unsigned long long SI_rtc_status_block[8] =
    {
        0x00000000ff010306,
        0xfffffffffe000000,
        0,
        0,
        0,
        0,
        0,
        1
    };
    static unsigned long long output[8];

    __rtc_exec_PIF(SI_rtc_status_block, output);

    /* We are looking for 0x10 in the second byte returned, which
     * signifies that there is an RTC present.*/
    if( ((output[1] >> 48) & 0xFF) == 0x10 )
    {
        /* RTC found! */
        return 1;
    }
    else
    {
        /* RTC not found! */
        return 0;
    }
}

void rtc_read(struct tm * time)
{
    static unsigned long long SI_rtc_read_block[8] =
    {
        0x0000000002090702,
        0xffffffffffffffff,
        0xfffe000000000000,
        0,
        0,
        0,
        0,
        1
    };
    static unsigned long long output[8];

    __rtc_exec_PIF(SI_rtc_read_block, output);

    uint8_t * res = (uint8_t *)&output[1];
    time->tm_sec = __bcd_to_byte(res[0]);
    time->tm_min = __bcd_to_byte(res[1]);
    time->tm_hour = __bcd_to_byte(res[2] & ~0x80);
    time->tm_mday = __bcd_to_byte(res[3]);
    time->tm_mon = __bcd_to_byte(res[5]) - 1;
    
    // only supporting years 2000+
    time->tm_year = __bcd_to_byte(res[6]) + 2000;
    time->tm_wday = res[4];
    time->tm_yday = 0;  // not supported
    time->tm_isdst = 0; // not supported
}

void rtc_write(struct tm * time)
{
    
}
