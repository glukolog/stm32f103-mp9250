#-------------------------------------------------------------------------------
TARGET  = stm32_mp9250_st7920

DEFINES += USE_STDPERIPH_DRIVER
DEFINES += STM32F10X_MD
#DEFINES += STM32F10X_MD_VL

#DEFINES += GCC_ARMCM3
#DEFINES += VECT_TAB_FLASH

SPL_NAME   := stm32f10x_md

# Инструменты
#-------------------------------------------------------------------------------
AS = arm-none-eabi-gcc
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
AR = arm-none-eabi-ar
RM = rm

SPL_PATH   := ../stm32f10x
CMSIS_LIB  := $(SPL_PATH)/Libraries/CMSIS
SPL_DRIVER := $(SPL_PATH)/Libraries/STM32F10x_StdPeriph_Driver

#STARTUP = startup/startup_stm32f10x_md_vl.s
STARTUP = $(SPL_PATH)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s
#STARTUP = $(LIBSTM32)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md_vl.s

SOURCES := $(wildcard src/*.c)
#	./main.c \
#	./stm32f10x_it.c 
#LIB_SRCS += $(CMSIS_LIB)/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
SOURCES += $(CMSIS_LIB)/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
SOURCES += ../stmlibs/systick.c
#SOURCES += ../stmlibs/i2c-soft.c
SOURCES += ../stmlibs/bmp280.c

#CFLAGS = -mcpu=cortex-m3 -mthumb -Wall -g -O0
CFLAGS += -mthumb -mcpu=cortex-m3 -msoft-float -mfpu=vfp
#CFLAGS += -std=gnu99              # стандарт языка С
CFLAGS += -Wall -pedantic         # Выводить все предупреждения
CFLAGS += -Os                     # Оптимизация
#CFLAGS += -ggdb                   # Генерировать отладочную информацию для gdb
#CFLAGS += -fno-builtin
CFLAGS += --specs=nosys.specs
CFLAGS += -fdata-sections -ffunction-sections -fno-inline -flto

CFLAGS += -I $(CMSIS_LIB)/CM3/CoreSupport
CFLAGS += -I $(CMSIS_LIB)/CM3/DeviceSupport/ST/STM32F10x
CFLAGS += -I $(SPL_DRIVER)/inc
CFLAGS += -I src
CFLAGS += -I ../stmlibs
CFLAGS += -I u8g2/csrc
#CFLAGS += -Lout -l$(SPL_NAME)
CFLAGS += $(addprefix -D, $(DEFINES))

# Скрипт линкера
#-------------------------------------------------------------------------------
#LDSCRIPT   = ld-scripts/stm32f100rb.ld
LDSCRIPT   = STM32F103X8.ld
#LDSCRIPT   = stm32f103x8_flash.ld

# Настройки линкера
#-------------------------------------------------------------------------------
#LDFLAGS = -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(addprefix -I, $(INCLUDES)) -T $(LDSCRIPT)
#LDFLAGS += -nostartfiles --specs=nosys.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -mthumb -mcpu=cortex-m3 -msoft-float -mfpu=vfp
#LDFLAGS += -Wl,--gc-sections
LDFLAGS += -lgcc -lm -Wl,--start-group -Wl,--gc-sections -lnosys
#LDFLAGS += -Lld-scripts
LDFLAGS += -T$(LDSCRIPT)

OBJS += $(patsubst %.s, %.o, $(STARTUP))
OBJS += $(patsubst %.c, %.o, $(SOURCES))

#LIB_SRCS := $(notdir $(wildcard $(SPL_DRIVER)/src/*.c))
LIB_SRCS := $(wildcard $(SPL_DRIVER)/src/*.c)
LIB_OBJS := $(LIB_SRCS:.c=.o)
LIB_DEPS := $(LIB_SRCS:.c=.d)

all: $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	@$(CP) -Oihex $(TARGET).elf $(TARGET).hex
	@$(CP) -Obinary $(TARGET).elf $(TARGET).bin
	@echo "---------------------------------------------------"
	@$(SZ) $(TARGET).elf

# Линковка
#-------------------------------------------------------------------------------
$(TARGET).elf: $(OBJS)
	@echo $(LD)  $^ -o $@ $(LDFLAGS) -Lout -l$(SPL_NAME) -lu8g2
	@$(LD)  $^ -o $@ $(LDFLAGS) -Lout -l$(SPL_NAME) -lu8g2 -Wl,-Map,$(TARGET).map

lib: $(LIB_OBJS)
	@echo $(AR) -r out/lib$(SPL_NAME).a $(LIB_OBJS)
	$(AR) -r out/lib$(SPL_NAME).a $(LIB_OBJS)
	$(RM) $(LIB_OBJS)
#	$(RM) $(LIB_DEPS)

U8G2_SRCS := $(wildcard u8g2/csrc/*.c)
U8G2_OBJS := $(U8G2_SRCS:.c=.o)
U8G2_DEPS := $(U8G2_SRCS:.c=.d)
u8g2: $(U8G2_OBJS)
	@echo $(AR) -r out/libu8g2.a $(U8G2_OBJS)
	$(AR) -r out/libu8g2.a $(U8G2_OBJS)
	$(RM) $(U8G2_OBJS)

%.o: %.c
	@echo $(CC) $(CFLAGS) -c $< -o $@
	@$(CC) $(CFLAGS) -c $< -o $@

#$(info $$TOREMOVE is [${TOREMOVE}])

%.o: %.s
	@$(AS) $(CFLAGS) -c $< -o $@

clean:
	@$(RM) -f out/$(TARGET).*
	@$(RM) -f $(OBJS)
	@$(RM) -f $(patsubst %.s, %.o, $(STARTUP))

# Сгенерированные gcc зависимости
#-------------------------------------------------------------------------------
include $(wildcart *.d)

# Flash
install:
#	st-flash write $(TARGET).bin 0x08000000
	st-flash --format ihex write $(TARGET).hex
