#include "axi_regs.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* Open /dev/mem and map corresponding address for the AXI-Lite */
int axi_regs_open(axi_regs_t *handle, uintptr_t phys_addr, size_t map_size)
{
    /* Register physical address */
    handle->phys_addr = phys_addr;
    handle->map_size  = map_size;

    handle->fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (handle->fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    void *map = mmap(NULL, map_size,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     handle->fd, phys_addr);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(handle->fd);
        return -1;
    }

    handle->regs = (volatile uint32_t *)map;
    return 0;
}

/* Close mapped /dev/mem portion */
void axi_regs_close(axi_regs_t *handle)
{
    munmap((void *)handle->regs, handle->map_size);
    close(handle->fd);
}

/* Read value at offset */
uint32_t axi_regs_read(axi_regs_t *handle, uint32_t offset)
{
    return handle->regs[offset / 4];
}

/* Write value at offset */
void axi_regs_write(axi_regs_t *handle, uint32_t offset, uint32_t value)
{
    handle->regs[offset / 4] = value;
}

/* Print read values as defined in the descriptions */
void axi_regs_print(axi_regs_t *handle, const axi_reg_desc_t *descs, size_t n)
{
    for (size_t i = 0; i < n; i++)
        printf("%-25s : %u\n", descs[i].name,
               axi_regs_read(handle, descs[i].offset));
}