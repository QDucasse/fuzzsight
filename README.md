# Fuzzsight: Hardware-accelerated CoreSight fuzzing


## Description


## Installation

The whole block design is defined in `bd/fuzzsight.tcl`. Running the synthesis/implementation is done with:

```bash
$ make PROJECT=fuzzsight synth
```

This generates the bitstream in `build/fuzzsight/fuzzsight.bin`. To generate the corresponding device tree overlay, run the command below and double-check that your addresses correspond to the ones in Vivado's address editor:

```bash
$ make PROJECT=fuzzsight dtbo
```

Using a running Linux system on the board (UltraScale+ ZCU104 in our case), we can load the bitstream using `fpgamanager`:

```bash
$ cp build/fuzzsight/fuzzsight.bin build/fuzzsight/fuzzsight.dtbo /lib/firmware
$ fpga util -b /lib/firmware/fuzzsight.bin -o /lib/firmware/fuzzsight.dtbo
Time taken to load BIN is 563.000000 Milli Seconds
BIN FILE loaded through FPGA manager successfully
```

## Usage

As soon as CoreSight is enabled (using [csal](https://github.com/arm-software/csal)), Fuzzsight fires up and writes edges in a bitmap stored in BRAM. Once the process finishes, the PS is expected to notify the bitmap reader using its AXI-Lite interface, which will read the bitmap and turn into an AXI-Stream, in turn passed to AXI-DMA.

The main logic is defined in `bitmap_dma.c`:

```c
int bitmap_dma_transfer(bitmap_dma_t *h)
{
    // Wait for edge extractor FIFO to drain
    while (!(axi_regs_read(&h->reader, BITMAP_READER_STATUS) & STATUS_FIFO_EMPTY))
        ;

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
```

Simple "drivers" to read/write and interact with the different Fuzzsight AXI-Lite interfaces are present in the `src/` folder. At a glance, three modules present AXI-Lite interfaces:

- `bitmap_read_bram`: Control register to request a DMA transfer, and status register to announce completion.
- `edge_extractor`: Information on dropped edges due to overflow or freeze requests.
- `decoder_stats_lut` : Stats on decoder gaps/bursts.
