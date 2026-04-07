#ifndef EDGE_STATS_H
#define EDGE_STATS_H

#include "axi_regs.h"

#define EDGE_STATS_MAP_SIZE 0x1000

// Default base addresses (can/should be overridden)
#ifndef EDGE_STATS_BASE
#pragma message("WARNING: EDGE_STATS_BASE not defined, using placeholder 0x80000000 - override for your Vivado project")
#define EDGE_STATS_BASE 0x80000000
#endif

/* Register offsets */
typedef enum {
    EDGE_STATS_CTRL        = 0x00,  // bit 0 = stats_reset
    EDGE_STATS_TOTAL       = 0x04,  // edges_total
    EDGE_STATS_OVERFLOW    = 0x08,  // fifo_overflow_count
    EDGE_STATS_FREEZE_DROP = 0x0C,  // freeze_drop_count
} edge_stats_reg_t;

/* Control register bits */
#define EDGE_STATS_CTRL_STATS_RESET      (1 << 0)

typedef axi_regs_t edge_stats_t;

/* Function mapping to axi_regs defaults */
static inline int  edge_stats_open(edge_stats_t *handle)
    { return axi_regs_open(handle, EDGE_STATS_BASE,EDGE_STATS_MAP_SIZE); }
static inline void edge_stats_close(edge_stats_t *handle)
    { axi_regs_close(handle); }
static inline void edge_stats_reset(edge_stats_t *handle)
    { axi_regs_write(handle, EDGE_STATS_CTRL, EDGE_STATS_CTRL_STATS_RESET); }
static inline uint32_t edge_stats_read(edge_stats_t *handle, edge_stats_reg_t reg)
    { return axi_regs_read(handle, reg); }

/* Description used in the print */
static axi_reg_desc_t edge_stats_descs[] = {
    {"Edges total",  EDGE_STATS_TOTAL},
    {"FIFO overflow", EDGE_STATS_OVERFLOW},
    {"Freeze drops",  EDGE_STATS_FREEZE_DROP},
};

static inline void edge_stats_print(edge_stats_t *h)
    { axi_regs_print(h, edge_stats_descs,
                     sizeof(edge_stats_descs) / sizeof(edge_stats_descs[0])); }

#endif