#ifndef DECODER_AXI_H
#define DECODER_AXI_H

#include "axi_regs.h"

#define DECODER_AXI_MAP_SIZE 0x1000

// Default base addresses (can/should be overridden)
#ifndef DECODER_AXI_BASE
#pragma message("WARNING: EDGE_STATS_BASE not defined, using placeholder 0x80000000 - override for your Vivado project")
#define DECODER_AXI_BASE 0x80000000
#endif

/* Register offsets */
typedef enum {
    DECODER_AXI_CTRL    = 0x00,  // bit 0 = error count reset
    DECODER_AXI_FRAME   = 0x04,  // frame errors
    DECODER_AXI_BS_GEN  = 0x08,  // bytestream errors
    DECODER_AXI_DEMUX   = 0x0C,  // demux errors
    DECODER_AXI_STATUS  = 0x10,  // status register to poll soft reset
    DECODER_AXI_RANGE_BASE_LO  = 0x14, // Lower 32-bits of the base address
    DECODER_AXI_RANGE_BASE_HI  = 0x18, // Upper 32-bits of the base address
    DECODER_AXI_RANGE_END_LO   = 0x1C, // Lower 32-bits of the end address
    DECODER_AXI_RANGE_END_HI   = 0x20, // Upper 32-bits of the base address
} decoder_axi_reg_t;

/* Control register bits */
#define DECODER_AXI_CTRL_STATS_RESET  (0x1 << 0)
#define DECODER_AXI_CTRL_SOFT_RESET   (0x1 << 1)

/* Status register bits */
#define DECODER_AXI_STATUS_RESET_DONE (0x1 << 0)

typedef axi_regs_t decoder_axi_t;

/* Function mapping to axi_regs defaults */
static inline int  decoder_axi_open(decoder_axi_t *handle)
    { return axi_regs_open(handle, DECODER_AXI_BASE,DECODER_AXI_MAP_SIZE); }
static inline void decoder_axi_close(decoder_axi_t *handle)
    { axi_regs_close(handle); }
static inline void decoder_axi_stats_reset(decoder_axi_t *handle)
    { axi_regs_write(handle, DECODER_AXI_CTRL, DECODER_AXI_CTRL_STATS_RESET); }
static inline uint32_t decoder_axi_read(decoder_axi_t *handle, decoder_axi_reg_t reg)
    { return axi_regs_read(handle, reg); }

static inline void decoder_axi_soft_reset(decoder_axi_t *handle) {
    /* Trigger soft reset of decoder pipeline */
    axi_regs_write(handle, DECODER_AXI_CTRL, DECODER_AXI_CTRL_SOFT_RESET);
    /* Poll until reset complete (shift register drained) */
    while (!(axi_regs_read(handle, DECODER_AXI_STATUS) & DECODER_AXI_STATUS_RESET_DONE))
        ;
}

/* Range writers */
static inline void decoder_axi_set_range_base(decoder_axi_t *handle, uint64_t base) {
    axi_regs_write(handle, DECODER_AXI_RANGE_BASE_LO, (uint32_t)(base & 0xFFFFFFFF));
    axi_regs_write(handle, DECODER_AXI_RANGE_BASE_HI, (uint32_t)(base >> 32));
}

static inline void decoder_axi_set_range_end(decoder_axi_t *handle, uint64_t end) {
    axi_regs_write(handle, DECODER_AXI_RANGE_END_LO, (uint32_t)(end & 0xFFFFFFFF));
    axi_regs_write(handle, DECODER_AXI_RANGE_END_HI, (uint32_t)(end >> 32));
}

static inline void decoder_axi_set_range(decoder_axi_t *handle, uint64_t base, uint64_t end) {
    decoder_axi_set_range_base(handle, base);
    decoder_axi_set_range_end(handle, end);
}

static inline void decoder_axi_print_range(decoder_axi_t *handle) {
    uint32_t base_lo = axi_regs_read(handle, DECODER_AXI_RANGE_BASE_LO);
    uint32_t base_hi = axi_regs_read(handle, DECODER_AXI_RANGE_BASE_HI);
    uint32_t end_lo  = axi_regs_read(handle, DECODER_AXI_RANGE_END_LO);
    uint32_t end_hi  = axi_regs_read(handle, DECODER_AXI_RANGE_END_HI);
    uint64_t base = ((uint64_t)base_hi << 32) | base_lo;
    uint64_t end  = ((uint64_t)end_hi  << 32) | end_lo;
    fprintf(stderr, "[.] decoder range: 0x%016lx - 0x%016lx\n", base, end);
}

/* Description used in the print */
static axi_reg_desc_t decoder_axi_descs[] = {
    {"Frame errors",  DECODER_AXI_FRAME},
    {"Bytestream gen errors", DECODER_AXI_BS_GEN},
    {"Demux errors",  DECODER_AXI_DEMUX},
};

static inline void decoder_axi_print(decoder_axi_t *h)
    { axi_regs_print(h, decoder_axi_descs,
                     sizeof(decoder_axi_descs) / sizeof(decoder_axi_descs[0])); }

#endif