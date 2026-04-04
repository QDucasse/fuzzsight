// bitmap_dma.h
#ifndef BITMAP_DMA_H
#define BITMAP_DMA_H

#include <stdint.h>
#include <stddef.h>
#include "axi_regs.h"

// AXI DMA registers (S2MM only)
#define DMA_MAP_SIZE                0x10000
#define S2MM_CONTROL_REGISTER       0x30
#define S2MM_STATUS_REGISTER        0x34
#define S2MM_DST_ADDRESS_REGISTER   0x48
#define S2MM_BUFF_LENGTH_REGISTER   0x58

#define STATUS_IDLE                 (1 << 1)
#define STATUS_IOC_IRQ              (1 << 12)
#define RESET_DMA                   0x00000004
#define RUN_DMA                     0x00000001
#define HALT_DMA                    0x00000000
#define ENABLE_ALL_IRQ              0x00007000

#ifndef DMA_BASE
#pragma message("WARNING: DMA_BASE not defined, using placeholder")
#define DMA_BASE 0x80000000
#endif

#ifndef UDMABUF_DST
#pragma message("WARNING: UDMABUF_DST not defined, using placeholder")
#define UDMABUF_DST 0x60000000
#endif

#ifndef UDMABUF_DEV
#pragma message("WARNING: UDMABUF_DEV not defined, using placeholder")
#define UDMABUF_DEV "/dev/udmabuf0"
#endif

// Bitmap reader AXI-Lite
#define BITMAP_READER_MAP_SIZE      0x1000
#define BITMAP_READER_CTRL          0x00
#define BITMAP_READER_STATUS        0x04
#define STATUS_FIFO_EMPTY           (1 << 0)
#define STATUS_DMA_DONE             (1 << 1)
#define STATUS_DMA_BUSY             (1 << 2)

#ifndef BITMAP_READER_BASE
#pragma message("WARNING: BITMAP_READER_BASE not defined, using placeholder")
#define BITMAP_READER_BASE 0x80022000
#endif

typedef struct {
    axi_regs_t      dma;
    axi_regs_t      reader;
    int             udmabuf_fd;
    void           *buf;
    size_t          buf_size;
} bitmap_dma_t;

int  bitmap_dma_open(bitmap_dma_t *h, size_t bitmap_size);
void bitmap_dma_close(bitmap_dma_t *h);
int  bitmap_dma_transfer(bitmap_dma_t *h);

#endif