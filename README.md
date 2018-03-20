Blue Flash - SPI NOR flash programmer using Blue Pill
=====================================================

This NOR flash programmer was designed to write Olimex's iCE40HX1K-EVB 
Lattice FPGA board equipped with Winbond W25Q16 NOR flash. It uses the so 
called Blue Pill development board with STM32F103 MCU. The software can 
automatically detect NOR flash's capacity.
It is a USB device, and uses DFU protocol. You can use latest version of 
dfu-util from the SVN repository (for Arch Linux users 
https://aur.archlinux.org/packages/dfu-util-git/).
If you want to debug the source code, please, remove resistor R10 and put
a 1.5 kOhm between PB9 and PA12, this way the software can switch the
pull-up resistor on USB's D+ line.

Hardware
========
Following wires should connect to the iCE40HX1K-EVB:
 * CDONE      -> PA2 (not used yet)
 * CRESET     -> PA3	
 * SPI\_CS    -> PA4
 * SPI1\_MISO -> PA6
 * SPI1\_MOSI -> PA7
 * SPI1\_SCK  -> PA5

Following wires should connect to the NOR flash:
* SPI\_CS     -> PA4
* SPI1\_MISO  -> PA6
* SPI1\_MOSI  -> PA7
* SPI1\_SCK   -> PA5
/WP and /HOLD pins should be pulled to VCC on the NOR flash.

Connect a USB-to-serial converter if you want to see debug prints or
program MCU through UART interface:
USART1\_RX	PA10
USART1\_TX	PA9

Programming STM32 via UART
==========================
Connect a USB-to-serial converter, set BOOT0 jumper to 1 and run stm32flash:
stm32flash /dev/ttyUSB0 -w spi\_flasher.hex 

You should see something like this:
stm32flash 0.5

http://stm32flash.sourceforge.net/

Using Parser : Intel HEX
Interface serial\_posix: 57600 8E1
Version      : 0x22
Option 1     : 0x00
Option 2     : 0x00
Device ID    : 0x0410 (STM32F10xxx Medium-density)
- RAM        : 20KiB  (512b reserved by bootloader)
- Flash      : 128KiB (size first sector: 4x1024)
- Option RAM : 16b
- System RAM : 2KiB
Write to memory
Erasing memory
Wrote address 0x0800d7a8 (100.00%) Done.

Restore BOOT0 jumper to 0 and reset the board.

Compile & debug
===============
Import spi\_flasher project into Atollic TrueSTUDIO for STM32 and hit the 
hammer.

Author
======
Copyright (C) Peter Ivanov &lt;ivanovp@gmail.com&gt;, 2018

