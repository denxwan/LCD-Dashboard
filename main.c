I (158) esp_image: segment 4: paddr=0006d80c vaddr=40380184 sizeESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x40378f4e
--- 0x40378f4e: esp_cpu_wait_for_intr at /home/denxwan/esp/v5.5.2/esp-idf/components/esp_hw_support/cpu.c:64
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x158c
load:0x403c8700,len:0xd24
load:0x403cb700,len:0x2f34
entry 0x403c8924
I (24) boot: ESP-IDF v5.5.2-dirty 2nd stage bootloader
I (24) boot: compile time Dec 30 2025 23:47:03
I (25) boot: Multicore bootloader
I (25) boot: chip revision: v0.2
I (28) boot: efuse block revision: v1.3
I (32) boot.esp32s3: Boot SPI Speed : 80MHz
I (35) boot.esp32s3: SPI Mode       : DIO
I (39) boot.esp32s3: SPI Flash Size : 2MB
I (43) boot: Enabling RNG early entropy source...
I (47) boot: Partition Table:
I (50) boot: ## Label            Usage          Type ST Offset   Length
I (56) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (63) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (69) boot:  2 factory          factory app      00 00 00010000 00100000
I (76) boot: End of partition table
I (79) esp_image: segment 0: paddr=00010020 vaddr=3c040020 size=10c44h ( 68676) map
I (99) esp_image: segment 1: paddr=00020c6c vaddr=3fc96400 size=03220h ( 12832) load
I (102) esp_image: segment 2: paddr=00023e94 vaddr=40374000 size=0c184h ( 49540) load
I (113) esp_image: segment 3: paddr=00030020 vaddr=42000020 size=3d7e4h (251876) map
I (158) esp_image: segment 4: paddr=0006d80c vaddr=40380184 size=061cch ( 25036) load
I (164) esp_image: segment 5: paddr=000739e0 vaddr=50000000 size=00020h (    32) load
I (172) boot: Loaded app from partition at offset 0x10000
I (172) boot: Disabling RNG early entropy source...
I (184) octal_psram: vendor id    : 0x0d (AP)
I (184) octal_psram: dev id       : 0x02 (generation 3)
I (184) octal_psram: density      : 0x03 (64 Mbit)
I (186) octal_psram: good-die     : 0x01 (Pass)
I (190) octal_psram: Latency      : 0x01 (Fixed)
I (195) octal_psram: VCC          : 0x01 (3V)
I (199) octal_psram: SRF          : 0x01 (Fast Refresh)
I (204) octal_psram: BurstType    : 0x01 (Hybrid Wrap)
I (209) octal_psram: BurstLen     : 0x01 (32 Byte)
I (213) octal_psram: Readlatency  : 0x02 (10 cycles@Fixed)
I (218) octal_psram: DriveStrength: 0x00 (1/1)
I (222) MSPI Timing: Enter psram timing tuning
I (227) esp_psram: Found 8MB PSRAM device
I (230) esp_psram: Speed: 80MHz
I (246) mmu_psram: Read only data copied and mapped to SPIRAM
I (273) mmu_psram: Instructions copied and mapped to SPIRAM
I (274) cpu_start: Multicore app
I (680) esp_psram: SPI SRAM memory test OK
I (688) cpu_start: GPIO 44 and 43 are used as console UART I/O pins
I (689) cpu_start: Pro cpu start user code
I (689) cpu_start: cpu freq: 160000000 Hz
I (691) app_init: Application information:
I (695) app_init: Project name:     rgb_panel
I (699) app_init: App version:      1
I (702) app_init: Compile time:     Dec 30 2025 23:46:53
I (707) app_init: ELF file SHA256:  19404b925...
I (711) app_init: ESP-IDF:          v5.5.2-dirty
I (716) efuse_init: Min chip rev:     v0.0
I (720) efuse_init: Max chip rev:     v0.99 
I (724) efuse_init: Chip rev:         v0.2
I (727) heap_init: Initializing. RAM available for dynamic allocation:
I (734) heap_init: At 3FCA3A90 len 00045C80 (279 KiB): RAM
I (739) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (744) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (749) heap_init: At 600FE000 len 00001FE8 (7 KiB): RTCRAM
I (755) esp_psram: Adding pool of 7808K of PSRAM memory to heap allocator
I (761) esp_psram: Adding pool of 60K of PSRAM memory gap generated due to end address alignment of drom to the heap allocator
I (773) spi_flash: detected chip: gd
I (775) spi_flash: flash io: dio
W (778) spi_flash: Detected size(8192k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (791) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (797) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (804) main_task: Started on CPU0
I (814) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (814) main_task: Calling app_main()
