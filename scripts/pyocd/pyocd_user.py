# add QSPI FLASH

def will_connect():
    # Add the external flash memory region.
    target.memory_map.add_region(
        FlashRegion(
            name="qspi_flash",
            start=0x90000000,
            length=0x00080000,
            blocksize=4096,
            page_size=256,
            is_boot_memory=False,
            flm="./scripts/pyocd/STM32H7x_QSPI_Bank1.FLM", # relative to pyocd project directory
        )
    )
