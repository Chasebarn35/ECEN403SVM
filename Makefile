TOOLCHAIN_ROOT = 

VENDOR_ROOT = ./bsp/


TARGET = main.elf
SRC_DIR = src/
INC_DIR = inc/

CC = $(TOOLCHAIN_ROOT)arm-none-eabi-gcc
DB = $(TOOLCHAIN_ROOT)arm-none-eabi-gdb

SRC_FILES = $(wildcard $(SRC_DIR)*.c) $(wildcard $(SRC_DIR)*/*.c)
ASM_FILES = $(wildcard $(SRC_DIR)*.s) $(wildcard $(SRC_DIR)*/*.s)
LD_SCRIPT = $(SRC_DIR)/device/STM32G431RBTX_FLASH.ld #TODO see dif between normal and _FLASH

INCLUDES  = -I$(INC_DIR)


#TODO PULL FROM THE CUBE TO ACTUALLY GREP TO
ASM_FILES += $(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s
#SRC_FILES += $(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/STM32G4xx_Nucleo/stm32g4xx_nucleo.c #TODO CHECK
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c


#TODO VERIFY
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Core/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/DSP/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32G4xx/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/STM32G4xx_HAL_Driver/Inc
INCLUDES += -I$(VENDOR_ROOT)Drivers/BSP/STM32G4xx_Nucleo


CFLAGS  = -g -O0 -Wall -Wextra -Warray-bounds -Wno-unused-parameter
CFLAGS += -mcpu=cortex-m7 -mthumb -mlittle-endian -mthumb-interwork 
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 #TODO CHECK
CFLAGS += -DSTM32G431xx -DUSE_STM32G4XX_NUCLEO_144 -DUSE_HAL_DRIVER 
CFLAGS += $(INCLUDES) 

LFLAGS = -Wl,--gc-sections -Wl,-T$(LD_SCRIPT) --specs=rdimon.specs


CXX_OBJS = $(SRC_FILES:.c=.o) 
ASM_OBJS = $(ASM_FILES:.s=.o)
ALL_OBJS = $(ASM_OBJS) $(CXX_OBJS)

.PHONY: clean gdb-server_stlink gdb-server_openocd gdb-client 

all: $(TARGET)


 
#Compile 
$(CXX_OBJS): %.o: %.c
$(ASM_OBJS): %.o: %.s
$(ALL_OBJS): 
	@echo "[CC] $@" 
	@$(CC) $(CFLAGS) -c $< -o $@

#Link
%.elf: $(ALL_OBJS)
	@echo "[LD] $@"
	@$(CC) $(CFLAGS) $(LFLAGS) $(ALL_OBJS) -o $@

#Clean
clean:
	@rm -f $(ALL_OBJS) $(TARGET)

#Flash
flash:
	openocd -f ./openocd.cfg

#Debug
gdb-server_stlink:
	st-util

gdb-server_openocd:
	openocd -f ./openocd.cfg

gdb-client: $(TARGET)
	$(DB) -tui $(TARGET)
