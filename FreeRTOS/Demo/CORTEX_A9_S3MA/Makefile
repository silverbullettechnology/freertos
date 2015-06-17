NAME = FreeRTOSDemo

#CCPATH = /opt/gcc-linaro-4.9-2014.11-x86_64_arm-eabi/bin/
CC = $(CCPATH)arm-none-eabi-gcc
LD = $(CC5PATH)arm-none-eabi-gcc
OBJCOPY = $(CCPATH)arm-none-eabi-objcopy

DEFINES = -DPRINTF_FLOAT_SUPPORT

LIBS = -lm

SOURCE_DIR = ../..
BUILD_DIR = Build

ARCH_OPTS = 	-march=armv7-a \
                -mcpu=cortex-a9 \
                -mfloat-abi=hard \
                -mfpu=neon \
                -marm

C_OPTS =	-I"$(SOURCE_DIR)/Demo/CORTEX_A9_S3MA" \
			-I"$(SOURCE_DIR)/Demo/Common/include" \
			-I"$(SOURCE_DIR)/Source/include" \
			-I"$(SOURCE_DIR)/Source/portable/GCC/ARM_CA9" \
			-Wall \
			-fmessage-length=0 \
			-std=c99

ifeq ($(DEBUG),1)
  C_OPTS += -g -Og
else
  C_OPTS += -O3
endif

C_FILES =	Source/croutine.c \
			Source/list.c \
			Source/queue.c \
			Source/tasks.c \
			Source/timers.c \
			Source/portable/GCC/ARM_CA9/port.c \
			Source/portable/MemMang/heap_2.c \
			Demo/CORTEX_A9_S3MA/main.c \
			Demo/CORTEX_A9_S3MA/pl011.c \
			Demo/CORTEX_A9_S3MA/printf-stdarg.c \
			Demo/CORTEX_A9_S3MA/serial.c \
			Demo/CORTEX_A9_S3MA/port.c \
			Demo/CORTEX_A9_S3MA/ParTest.c \
			Demo/Common/Minimal/BlockQ.c \
			Demo/Common/Minimal/blocktim.c \
			Demo/Common/Minimal/comtest.c \
			Demo/Common/Minimal/countsem.c \
			Demo/Common/Minimal/dynamic.c \
			Demo/Common/Minimal/flop.c \
			Demo/Common/Minimal/GenQTest.c \
			Demo/Common/Minimal/integer.c \
			Demo/Common/Minimal/PollQ.c \
			Demo/Common/Minimal/QPeek.c \
			Demo/Common/Minimal/recmutex.c \
			Demo/Common/Minimal/semtest.c

S_FILES =	Source/portable/GCC/ARM_CA9/portASM.S \
			Demo/CORTEX_A9_S3MA/startup.S
			

C_OBJS = $(C_FILES:%.c=$(BUILD_DIR)/%.o)

S_OBJS = $(S_FILES:%.S=$(BUILD_DIR)/%.o)

AUTODEPENDENCY_CFLAGS=-MMD -MF$(@:.o=.d) -MT$@

ALL_CFLAGS = $(C_OPTS) $(ARCH_OPTS) $(DEFINES) $(CFLAGS) $(AUTODEPENDENCY_CFLAGS)

ALL_LDFLAGS_BASE =	$(LD_FLAGS) \
					-nostartfiles \
					-g

LINK_PATH = MultiCore/

ALL_LDFLAGS_0 =	$(ALL_LDFLAGS_BASE) $(ARCH_OPTS) \
				-Wl,-T,$(LINK_PATH)core.0.ld -Wl,-Map,$(BUILD_DIR)/core.0.map 

ALL_LDFLAGS_1 =	$(ALL_LDFLAGS_BASE) $(ARCH_OPTS) \
				-Wl,-T,$(LINK_PATH)core.1.ld -Wl,-Map,$(BUILD_DIR)/core.1.map

ALL_LDFLAGS_2 =	$(ALL_LDFLAGS_BASE) $(ARCH_OPTS) \
				-Wl,-T,$(LINK_PATH)core.2.ld -Wl,-Map,$(BUILD_DIR)/core.2.map

ALL_LDFLAGS_3 =	$(ALL_LDFLAGS_BASE) $(ARCH_OPTS) \
				-Wl,-T,$(LINK_PATH)core.3.ld -Wl,-Map,$(BUILD_DIR)/core.3.map

AUTODEPENDENCY_CFLAGS=-MMD -MF$(@:.o=.d) -MT$@


.SUFFIXES: .o .c .bin

all: $(NAME).0.uimg $(NAME).1.uimg $(NAME).2.uimg $(NAME).3.uimg

clean:
	rm -rf $(BUILD_DIR) $(NAME).0.elf $(NAME).1.elf $(NAME).2.elf $(NAME).3.elf $(NAME).0.bin $(NAME).1.bin $(NAME).2.bin $(NAME).3.bin $(NAME).0.uimg $(NAME).1.uimg $(NAME).2.uimg $(NAME).3.uimg

$(NAME).0.uimg: $(NAME).0.bin
	mkimage -A arm -O linux -T kernel -C none -a 0x80010000 -e 0x80010000 \
	-d $< -n FreeRTOS.0 $@

$(NAME).1.uimg: $(NAME).1.bin
	mkimage -A arm -O linux -T kernel -C none -a 0x81010000 -e 0x81010000 \
	-d $< -n FreeRTOS.0 $@

$(NAME).2.uimg: $(NAME).2.bin
	mkimage -A arm -O linux -T kernel -C none -a 0x82010000 -e 0x82010000 \
	-d $< -n FreeRTOS.0 $@

$(NAME).3.uimg: $(NAME).3.bin
	mkimage -A arm -O linux -T kernel -C none -a 0x83010000 -e 0x83010000 \
	-d $< -n FreeRTOS.0 $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

$(NAME).0.elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS_0) -o $@ $^ $(LIBS)

$(NAME).1.elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS_1) -o $@ $^ $(LIBS)

$(NAME).2.elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS_2) -o $@ $^ $(LIBS)

$(NAME).3.elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS_3) -o $@ $^ $(LIBS)

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.S
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

-include $(C_OBJS:.o=.d)

