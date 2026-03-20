#include "decoder_stats.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>


int dec_stats_open(dec_stats_t *handle, uintptr_t phys_addr)
{
    /* Register physical address */
    handle->phys_addr = phys_addr;

    /* Open /dev/mem directly */
    handle->fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (handle->fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    /* MMAP our axi stats registers */
    void *map = mmap(
        NULL,
        DEC_STATS_MAP_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        handle->fd,
        phys_addr
    );
    if (map == MAP_FAILED) {
        perror("mmap");
        close(handle->fd);
        return -1;
    }

    /* Mapped address */
    handle->regs = (volatile uint32_t *)map;

    return 0;
}


void dec_stats_close(dec_stats_t *handle)
{
    munmap((void*)handle->regs, DEC_STATS_MAP_SIZE);
    close(handle->fd);
}


void dec_stats_enable(dec_stats_t *handle)
{
    // Bit 0 - enable, bit 1 - reset counters
    handle->regs[DEC_STATS_CTRL/4] = 3;
}


void dec_stats_disable(dec_stats_t *handle)
{
    handle->regs[DEC_STATS_CTRL/4] = 0;
}


uint32_t dec_stats_read(dec_stats_t *handle, dec_stats_reg_t reg)
{
    return handle->regs[reg/4];
}


static dec_stat_desc_t stats[] = {
    {"Total cycles",            DEC_STATS_TOTAL},
    {"Idle cycles",             DEC_STATS_IDLE},
    {"Packet count",            DEC_STATS_PACKET_CNT},
    {"Packet burst count",      DEC_STATS_PACKET_BST_CNT},
    {"Min packet burst length", DEC_STATS_PACKET_MIN_BST},
    {"Max packet burst length", DEC_STATS_PACKET_MAX_BST},
    {"Sum packet burst length", DEC_STATS_PACKET_SUM_BST},
    {"Packet gap count",        DEC_STATS_PACKET_GAP_CNT},
    {"Min packet gap length",   DEC_STATS_PACKET_MIN_GAP},
    {"Max packet gap length",   DEC_STATS_PACKET_MAX_GAP},
    {"Sum packet gap length",   DEC_STATS_PACKET_SUM_GAP},
    {"Atom count",              DEC_STATS_ATOM_CNT},
    {"Atom burst count",        DEC_STATS_ATOM_BST_CNT},
    {"Min atom burst length",   DEC_STATS_ATOM_MIN_BST},
    {"Max atom burst length",   DEC_STATS_ATOM_MAX_BST},
    {"Sum atom burst length",   DEC_STATS_ATOM_SUM_BST},
    {"Atom gap count",          DEC_STATS_ATOM_GAP_CNT},
    {"Min Atom gap length",     DEC_STATS_ATOM_MIN_GAP},
    {"Max Atom gap length",     DEC_STATS_ATOM_MAX_GAP},
    {"Sum Atom gap length",     DEC_STATS_ATOM_SUM_GAP},
};

void dec_stats_print(dec_stats_t *handle)
{
    size_t n = sizeof(stats)/sizeof(stats[0]);

    for (size_t i = 0; i < n; i++)
    {
        printf("%-15s : %u\n",
               stats[i].name,
               dec_stats_read(handle, stats[i].reg));
    }
}