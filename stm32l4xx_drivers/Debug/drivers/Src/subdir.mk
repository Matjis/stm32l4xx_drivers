################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32l476xx_gpio_driver.c \
../drivers/Src/stm32l476xx_spi_driver.c 

OBJS += \
./drivers/Src/stm32l476xx_gpio_driver.o \
./drivers/Src/stm32l476xx_spi_driver.o 

C_DEPS += \
./drivers/Src/stm32l476xx_gpio_driver.d \
./drivers/Src/stm32l476xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32l476xx_gpio_driver.o: ../drivers/Src/stm32l476xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32l476xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32l476xx_spi_driver.o: ../drivers/Src/stm32l476xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32l476xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

