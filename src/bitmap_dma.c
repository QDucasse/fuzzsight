// bitmap_dma.c
#include "bitmap_dma.h"

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

int bitmap_dma_open(bitmap_dma_t *h, size_t bitmap_size)
{
    h->buf_size = bitmap_size;

    // Map DMA AXI-Lite control
    if (axi_regs_open(&h->dma, DMA_BASE, DMA_MAP_SIZE) < 0) {
        perror("axi_regs_open DMA");
        return -1;
    }

    // Map bitmap reader AXI-Lite control
    if (axi_regs_open(&h->reader, BITMAP_READER_BASE, BITMAP_READER_MAP_SIZE) < 0) {
        perror("axi_regs_open bitmap reader");
        axi_regs_close(&h->dma);
        return -1;
    }

    // Map udmabuf as destination
    h->udmabuf_fd = open(UDMABUF_DEV, O_RDWR, O_SYNC);
    if (h->udmabuf_fd < 0) {
        perror("open udmabuf");
        axi_regs_close(&h->dma);
        axi_regs_close(&h->reader);
        return -1;
    }

    h->buf = mmap(NULL, bitmap_size, PROT_READ | PROT_WRITE,
                  MAP_SHARED, h->udmabuf_fd, 0);
    if (h->buf == MAP_FAILED) {
        perror("mmap udmabuf");
        close(h->udmabuf_fd);
        axi_regs_close(&h->dma);
        axi_regs_close(&h->reader);
        return -1;
    }

    // Reset and configure S2MM once
    axi_regs_write(&h->dma, S2MM_CONTROL_REGISTER, RESET_DMA);
    axi_regs_write(&h->dma, S2MM_CONTROL_REGISTER, HALT_DMA);
    axi_regs_write(&h->dma, S2MM_CONTROL_REGISTER, ENABLE_ALL_IRQ);
    axi_regs_write(&h->dma, S2MM_DST_ADDRESS_REGISTER, UDMABUF_DST);
    axi_regs_write(&h->dma, S2MM_CONTROL_REGISTER, RUN_DMA | ENABLE_ALL_IRQ);

    return 0;
}

void bitmap_dma_close(bitmap_dma_t *h)
{
    munmap(h->buf, h->buf_size);
    close(h->udmabuf_fd);
    axi_regs_close(&h->dma);
    axi_regs_close(&h->reader);
}

int bitmap_dma_transfer(bitmap_dma_t *h)
{
    // Trigger bitmap reader DMA
    axi_regs_write(&h->reader, BITMAP_READER_CTRL, 1);

    // Arm S2MM by writting the buffer length, triggering the transfer
    axi_regs_write(&h->dma, S2MM_BUFF_LENGTH_REGISTER, h->buf_size);

    // Wait for bitmap reader to confirm done
    while (!(axi_regs_read(&h->reader, BITMAP_READER_STATUS) & STATUS_DMA_DONE))
        ;

    // Wait for S2MM to complete
    uint32_t status;
    do {
        status = axi_regs_read(&h->dma, S2MM_STATUS_REGISTER);
    } while (!(status & STATUS_IOC_IRQ) || !(status & STATUS_IDLE));

    // Rearm S2MM for next transfer
    axi_regs_write(&h->dma, S2MM_DST_ADDRESS_REGISTER, UDMABUF_DST);
    axi_regs_write(&h->dma, S2MM_CONTROL_REGISTER, RUN_DMA | ENABLE_ALL_IRQ);

    return 0;
}