################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/stm32f0xx_gpio.c \
../Drivers/src/stm32f0xx_i2c.c \
../Drivers/src/stm32f0xx_rcc.c \
../Drivers/src/stm32f0xx_spi.c \
../Drivers/src/stm32f0xx_usart.c 

OBJS += \
./Drivers/src/stm32f0xx_gpio.o \
./Drivers/src/stm32f0xx_i2c.o \
./Drivers/src/stm32f0xx_rcc.o \
./Drivers/src/stm32f0xx_spi.o \
./Drivers/src/stm32f0xx_usart.o 

C_DEPS += \
./Drivers/src/stm32f0xx_gpio.d \
./Drivers/src/stm32f0xx_i2c.d \
./Drivers/src/stm32f0xx_rcc.d \
./Drivers/src/stm32f0xx_spi.d \
./Drivers/src/stm32f0xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o Drivers/src/%.su Drivers/src/%.cyclo: ../Drivers/src/%.c Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -DSTM32F072B_DISCO -c -I../Inc -I"C:/Users/hansj/STM32CubeIDE/workspace_1.12.0/stm32f0xx_drivers/Drivers/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-src

clean-Drivers-2f-src:
	-$(RM) ./Drivers/src/stm32f0xx_gpio.cyclo ./Drivers/src/stm32f0xx_gpio.d ./Drivers/src/stm32f0xx_gpio.o ./Drivers/src/stm32f0xx_gpio.su ./Drivers/src/stm32f0xx_i2c.cyclo ./Drivers/src/stm32f0xx_i2c.d ./Drivers/src/stm32f0xx_i2c.o ./Drivers/src/stm32f0xx_i2c.su ./Drivers/src/stm32f0xx_rcc.cyclo ./Drivers/src/stm32f0xx_rcc.d ./Drivers/src/stm32f0xx_rcc.o ./Drivers/src/stm32f0xx_rcc.su ./Drivers/src/stm32f0xx_spi.cyclo ./Drivers/src/stm32f0xx_spi.d ./Drivers/src/stm32f0xx_spi.o ./Drivers/src/stm32f0xx_spi.su ./Drivers/src/stm32f0xx_usart.cyclo ./Drivers/src/stm32f0xx_usart.d ./Drivers/src/stm32f0xx_usart.o ./Drivers/src/stm32f0xx_usart.su

.PHONY: clean-Drivers-2f-src

