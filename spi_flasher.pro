TEMPLATE = app
CONFIG += console
CONFIG -= qt

include(other.pro)
SOURCES += ./spi_flasher/STM32F103C8_FLASH.ld \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src/usbd_dfu.c \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/list.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
./spi_flasher/Src/flash.c \
./spi_flasher/Src/freertos.c \
./spi_flasher/Src/isr_handlers.c \
./spi_flasher/Src/main.c \
./spi_flasher/Src/stm32f1xx_hal_msp.c \
./spi_flasher/Src/stm32f1xx_hal_timebase_TIM.c \
./spi_flasher/Src/stm32f1xx_it.c \
./spi_flasher/Src/system_stm32f1xx.c \
./spi_flasher/Src/uart.c \
./spi_flasher/Src/usb_device.c \
./spi_flasher/Src/usbd_conf.c \
./spi_flasher/Src/usbd_desc.c \
./spi_flasher/Src/usbd_dfu_if.c \
./spi_flasher/startup/startup_stm32f103xb.s

HEADERS += ./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h \
./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h \
./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Include/system_stm32f1xx.h \
./spi_flasher/Drivers/CMSIS/Include/arm_common_tables.h \
./spi_flasher/Drivers/CMSIS/Include/arm_const_structs.h \
./spi_flasher/Drivers/CMSIS/Include/arm_math.h \
./spi_flasher/Drivers/CMSIS/Include/cmsis_armcc.h \
./spi_flasher/Drivers/CMSIS/Include/cmsis_armcc_V6.h \
./spi_flasher/Drivers/CMSIS/Include/cmsis_gcc.h \
./spi_flasher/Drivers/CMSIS/Include/core_cm0.h \
./spi_flasher/Drivers/CMSIS/Include/core_cm0plus.h \
./spi_flasher/Drivers/CMSIS/Include/core_cm3.h \
./spi_flasher/Drivers/CMSIS/Include/core_cm4.h \
./spi_flasher/Drivers/CMSIS/Include/core_cm7.h \
./spi_flasher/Drivers/CMSIS/Include/core_cmFunc.h \
./spi_flasher/Drivers/CMSIS/Include/core_cmInstr.h \
./spi_flasher/Drivers/CMSIS/Include/core_cmSimd.h \
./spi_flasher/Drivers/CMSIS/Include/core_sc000.h \
./spi_flasher/Drivers/CMSIS/Include/core_sc300.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_dma_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_flash_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pcd.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pcd_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pwr.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_spi.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim_ex.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_usb.h \
./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h \
./spi_flasher/Inc/FreeRTOSConfig.h \
./spi_flasher/Inc/common.h \
./spi_flasher/Inc/flash.h \
./spi_flasher/Inc/flash_config.h \
./spi_flasher/Inc/main.h \
./spi_flasher/Inc/stm32f1xx_hal_conf.h \
./spi_flasher/Inc/stm32f1xx_it.h \
./spi_flasher/Inc/uart.h \
./spi_flasher/Inc/usb_device.h \
./spi_flasher/Inc/usbd_conf.h \
./spi_flasher/Inc/usbd_desc.h \
./spi_flasher/Inc/usbd_dfu_if.h \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc/usbd_dfu.h \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_core.h \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_ctlreq.h \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_def.h \
./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_ioreq.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOSConfig_template.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/StackMacros.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/croutine.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/deprecated_definitions.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/event_groups.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/list.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/mpu_prototypes.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/mpu_wrappers.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/portable.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/projdefs.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/queue.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/task.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include/timers.h \
./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/portmacro.h \
./spi_flasher/Src/pifs_debug.h

INCLUDEPATH += .
INCLUDEPATH += ./.metadata
INCLUDEPATH += ./spi_flasher
INCLUDEPATH += ./spi_flasher/.settings
INCLUDEPATH += ./spi_flasher/Debug
INCLUDEPATH += ./spi_flasher/Drivers
INCLUDEPATH += ./spi_flasher/Inc
INCLUDEPATH += ./spi_flasher/Middlewares
INCLUDEPATH += ./spi_flasher/Src
INCLUDEPATH += ./spi_flasher/startup
INCLUDEPATH += ./spi_flasher/Debug/Drivers
INCLUDEPATH += ./spi_flasher/Debug/Middlewares
INCLUDEPATH += ./spi_flasher/Debug/Src
INCLUDEPATH += ./spi_flasher/Debug/startup
INCLUDEPATH += ./spi_flasher/Debug/Drivers/STM32F1xx_HAL_Driver
INCLUDEPATH += ./spi_flasher/Debug/Drivers/STM32F1xx_HAL_Driver/Src
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library/Class
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library/Core
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library/Class/DFU
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/ST/STM32_USB_Device_Library/Core/Src
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source/portable
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang
INCLUDEPATH += ./spi_flasher/Debug/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS
INCLUDEPATH += ./spi_flasher/Drivers/STM32F1xx_HAL_Driver
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Include
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device/ST
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Include
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Source
INCLUDEPATH += ./spi_flasher/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates
INCLUDEPATH += ./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc
INCLUDEPATH += ./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Src
INCLUDEPATH += ./spi_flasher/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
INCLUDEPATH += ./spi_flasher/Middlewares/ST
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class/DFU
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
INCLUDEPATH += ./spi_flasher/Middlewares/ST/STM32_USB_Device_Library/Core/Src
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/include
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang
INCLUDEPATH += ./spi_flasher/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3

