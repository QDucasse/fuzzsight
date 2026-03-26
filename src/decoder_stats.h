#ifndef DECODER_STATS_H
#define DECODER_STATS_H

#include "axi_regs.h"

#define DEC_STATS_MAP_SIZE 0x1000

// Default base addresses (can/should be overridden)
#ifndef DEC_STATS_BASE_ETM
#pragma message("WARNING: EDGE_STATS_BASE not defined, using placeholder 0x80000000 - override for your Vivado project")
#define DEC_STATS_BASE_ETM 0x8000000
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

typedef axi_regs_t dec_stats_t;

/* Function mapping to axi_regs defaults */
static inline int  dec_stats_open(dec_stats_t *handle)
    { return axi_regs_open(handle, DEC_STATS_BASE_ETM, DEC_STATS_MAP_SIZE); }
static inline void dec_stats_close(dec_stats_t *handle)
    { axi_regs_close(handle); }
static inline void dec_stats_enable(dec_stats_t *handle)
    { axi_regs_write(handle, DEC_STATS_CTRL, 3); }
static inline void dec_stats_disable(dec_stats_t *handle)
    { axi_regs_write(handle, DEC_STATS_CTRL, 0); }
static inline uint32_t dec_stats_read(dec_stats_t *handle, dec_stats_reg_t reg)
    { return axi_regs_read(handle, reg); }

/* Description used in the print */
static axi_reg_desc_t dec_stats_descs[] = {
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

static inline void dec_stats_print(dec_stats_t *h)
    { axi_regs_print(h, dec_stats_descs,
                     sizeof(dec_stats_descs) / sizeof(dec_stats_descs[0])); }

#endif