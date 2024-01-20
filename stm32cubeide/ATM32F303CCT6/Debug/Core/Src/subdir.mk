################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AGS02MA.c \
../Core/Src/BME280.c \
../Core/Src/GFX_FUNCTIONS.c \
../Core/Src/PM25.c \
../Core/Src/SCD41.c \
../Core/Src/ST7735_fonts.c \
../Core/Src/ZE08.c \
../Core/Src/main.c \
../Core/Src/st7735.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/AGS02MA.o \
./Core/Src/BME280.o \
./Core/Src/GFX_FUNCTIONS.o \
./Core/Src/PM25.o \
./Core/Src/SCD41.o \
./Core/Src/ST7735_fonts.o \
./Core/Src/ZE08.o \
./Core/Src/main.o \
./Core/Src/st7735.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/AGS02MA.d \
./Core/Src/BME280.d \
./Core/Src/GFX_FUNCTIONS.d \
./Core/Src/PM25.d \
./Core/Src/SCD41.d \
./Core/Src/ST7735_fonts.d \
./Core/Src/ZE08.d \
./Core/Src/main.d \
./Core/Src/st7735.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AGS02MA.cyclo ./Core/Src/AGS02MA.d ./Core/Src/AGS02MA.o ./Core/Src/AGS02MA.su ./Core/Src/BME280.cyclo ./Core/Src/BME280.d ./Core/Src/BME280.o ./Core/Src/BME280.su ./Core/Src/GFX_FUNCTIONS.cyclo ./Core/Src/GFX_FUNCTIONS.d ./Core/Src/GFX_FUNCTIONS.o ./Core/Src/GFX_FUNCTIONS.su ./Core/Src/PM25.cyclo ./Core/Src/PM25.d ./Core/Src/PM25.o ./Core/Src/PM25.su ./Core/Src/SCD41.cyclo ./Core/Src/SCD41.d ./Core/Src/SCD41.o ./Core/Src/SCD41.su ./Core/Src/ST7735_fonts.cyclo ./Core/Src/ST7735_fonts.d ./Core/Src/ST7735_fonts.o ./Core/Src/ST7735_fonts.su ./Core/Src/ZE08.cyclo ./Core/Src/ZE08.d ./Core/Src/ZE08.o ./Core/Src/ZE08.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/st7735.cyclo ./Core/Src/st7735.d ./Core/Src/st7735.o ./Core/Src/st7735.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su

.PHONY: clean-Core-2f-Src

