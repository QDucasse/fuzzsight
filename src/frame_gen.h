#ifndef FRAME_GEN_H
#define FRAME_GEN_H

#include "axi_regs.h"

#define FRAME_GEN_MAP_SIZE 0x1000

#ifndef FRAME_GEN_BASE
#pragma message("WARNING: FRAME_GEN_BASE not defined, using placeholder 0x80000000 - override for your Vivado project")
#define FRAME_GEN_BASE 0x80000000
#endif

#define FRAME_GEN_STATUS_SYNC 0x1
#define FRAME_GEN_FREEZE      0x1
#define FRAME_GEN_UNFREEZE    0x0

/* Register offsets */
typedef enum {
    FRAME_GEN_STATUS_REG  = 0x00,
    FRAME_GEN_CONTROL_REG = 0x04,
} frame_gen_reg_t;

typedef axi_regs_t frame_gen_t;

/* Function mapping to axi_regs defaults */
static inline int  frame_gen_open(frame_gen_t *handle)
    { return axi_regs_open(handle, FRAME_GEN_BASE, FRAME_GEN_MAP_SIZE); }
static inline void frame_gen_close(frame_gen_t *handle)
    { axi_regs_close(handle); }

/* Returns 1 if frame generator has locked onto a sync packet */
static inline int frame_gen_synchronized(frame_gen_t *handle)
    { return (axi_regs_read(handle, FRAME_GEN_STATUS_REG) & FRAME_GEN_STATUS_SYNC); }

/* Freeze the decoder — call before stop_trace(), default state at reset */
static inline void frame_gen_freeze(frame_gen_t *handle)
    { axi_regs_write(handle, FRAME_GEN_CONTROL_REG, FRAME_GEN_FREEZE); }

/* Unfreeze the decoder — call after start_trace(), then spin on synchronized */
static inline void frame_gen_unfreeze(frame_gen_t *handle)
    { axi_regs_write(handle, FRAME_GEN_CONTROL_REG, FRAME_GEN_UNFREEZE); }

/* Block until frame generator has locked onto a sync packet */
static inline void frame_gen_wait_synchronized(frame_gen_t *handle)
    { while (!frame_gen_synchronized(handle)) {} }

#endif