#ifndef AXI_REGS_H
#define AXI_REGS_H

#include <stdint.h>
#include <stddef.h>

/* Main information, file descriptor, address+size and register offsets */
typedef struct {
    int fd;
    uintptr_t phys_addr;
    volatile uint32_t *regs;
    size_t map_size;
} axi_regs_t;

/* Description of one offset entry */
typedef struct {
    const char *name;
    uint32_t    offset;
} axi_reg_desc_t;

/* Common functions */
int      axi_regs_open(axi_regs_t *handle, uintptr_t phys_addr, size_t map_size);
void     axi_regs_close(axi_regs_t *handle);
uint32_t axi_regs_read(axi_regs_t *handle, uint32_t offset);
void     axi_regs_write(axi_regs_t *handle, uint32_t offset, uint32_t value);
void     axi_regs_print(axi_regs_t *handle, const axi_reg_desc_t *descs, size_t size);

#endif