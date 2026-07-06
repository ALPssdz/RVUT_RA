################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/hal_entry.c \
../src/hal_warmstart.c \
../src/usb_descriptor.c 

C_DEPS += \
./src/hal_entry.d \
./src/hal_warmstart.d \
./src/usb_descriptor.d 

CREF += \
RA8P1_USB_flat.cref 

OBJS += \
./src/hal_entry.o \
./src/hal_warmstart.o \
./src/usb_descriptor.o 

MAP += \
RA8P1_USB_flat.map 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	$(file > $@.in,-mcpu=cortex-m85 -mthumb -mlittle-endian -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -fno-strict-aliasing -fmessage-length=0 -funsigned-char -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Waggregate-return -Wno-parentheses-equality -Wfloat-equal -g3 -std=c99 -flax-vector-conversions -fshort-enums -fno-unroll-loops -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra_gen" -I"." -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra_cfg/fsp_cfg/bsp" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra_cfg/fsp_cfg" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/src" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra/fsp/inc" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra/fsp/inc/api" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra/fsp/inc/instances" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra/arm/CMSIS_6/CMSIS/Core/Include" -I"/home/joe1/e2_studio/workspace/RA8P1_USB_flat/ra/fsp/src/r_usb_basic/src/driver/inc" -D_RENESAS_RA_ -D_RA_CORE=CPU0 -D_RA_ORDINAL=1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -x c "$<" -c -o "$@")
	@clang --target=arm-none-eabi @"$@.in"
