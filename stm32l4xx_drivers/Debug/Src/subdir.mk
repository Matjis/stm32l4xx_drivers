################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/spi_tx_testing.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/spi_tx_testing.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/spi_tx_testing.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/spi_tx_testing.o: ../Src/spi_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/spi_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

