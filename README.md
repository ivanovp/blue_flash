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
CDONE       PA2 (not used yet)
CRESET      PA3	
SPI\_CS     PA4
SPI1\_MISO	PA6
SPI1\_MOSI	PA7
SPI1\_SCK	PA5

Following wires should connect to the NOR flash:
SPI\_CS     PA4
SPI1\_MISO	PA6
SPI1\_MOSI	PA7
SPI1\_SCK	PA5
/WP and /HOLD pins should be pulled to VCC on the NOR flash.

Connect a USB-to-serial converter if you want to see debug prints:
USART1\_RX	PA10
USART1\_TX	PA9

Compile
=======
Import spi\_flasher project into Atollic TrueSTUDIO for STM32 and hit the 
hammer.

Author
======
Copyright (C) Peter Ivanov &lt;ivanovp@gmail.com&gt;, 2018

