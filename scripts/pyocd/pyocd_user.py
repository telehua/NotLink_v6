# add SPI FLASH

def will_connect():
    # Add the external flash memory region.
    target.memory_map.add_region(
        FlashRegion(
            name="ext_spi_flash",
            start=0x90000000,
            length=0x00080000,
            blocksize=4096,
            page_size=256,
            is_boot_memory=False,
            flm="./scripts/pyocd/NotLink_v6_SPI_W25X40.FLM", # relative to pyocd project directory
        )
    )
