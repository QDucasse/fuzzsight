
#ifndef DECODER_STATS_H
#define DECODER_STATS_H

#include <stdint.h>
#include <stddef.h>

#define DEC_STATS_MAP_SIZE 0x1000

// Default base addresses (can/should be overridden)
#ifndef DEC_STATS_BASE_ETM
#define DEC_STATS_BASE_ETM 0x80000000
#endif

/* Register offsets */
typedef enum {
    DEC_STATS_CTRL           = 0x00,
    DEC_STATS_TOTAL          = 0x04,
    DEC_STATS_IDLE           = 0x08,
    DEC_STATS_PACKET_CNT     = 0x0C,
    DEC_STATS_PACKET_BST_CNT = 0x10,
    DEC_STATS_PACKET_MIN_BST = 0x14,
    DEC_STATS_PACKET_MAX_BST = 0x18,
    DEC_STATS_PACKET_SUM_BST = 0x1C,
    DEC_STATS_PACKET_GAP_CNT = 0x20,
    DEC_STATS_PACKET_MIN_GAP = 0x24,
    DEC_STATS_PACKET_MAX_GAP = 0x28,
    DEC_STATS_PACKET_SUM_GAP = 0x2C,
    DEC_STATS_ATOM_CNT       = 0x30,
    DEC_STATS_ATOM_BST_CNT   = 0x34,
    DEC_STATS_ATOM_MIN_BST   = 0x38,
    DEC_STATS_ATOM_MAX_BST   = 0x3C,
    DEC_STATS_ATOM_SUM_BST   = 0x40,
    DEC_STATS_ATOM_GAP_CNT   = 0x44,
    DEC_STATS_ATOM_MIN_GAP   = 0x48,
    DEC_STATS_ATOM_MAX_GAP   = 0x4C,
    DEC_STATS_ATOM_SUM_GAP   = 0x50,
} dec_stats_reg_t;


/* Handle */
typedef struct {
    int fd;
    uintptr_t phys_addr;
    volatile uint32_t *regs;
} dec_stats_t;

typedef struct {
    const char *name;
    dec_stats_reg_t reg;
} dec_stat_desc_t;

/* API */
int  dec_stats_open(dec_stats_t *h, uintptr_t phys_addr);
void dec_stats_close(dec_stats_t *h);

void dec_stats_enable(dec_stats_t *h);
void dec_stats_disable(dec_stats_t *h);

uint32_t dec_stats_read(dec_stats_t *h, dec_stats_reg_t reg);

void dec_stats_print(dec_stats_t *h);

#endif