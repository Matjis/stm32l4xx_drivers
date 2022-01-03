################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/spi_message_rcv_it.c 

OBJS += \
./Src/spi_message_rcv_it.o 

C_DEPS += \
./Src/spi_message_rcv_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/spi_message_rcv_it.o: ../Src/spi_message_rcv_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DDEBUG -DSTM32L476RGTx -c -I../Inc -I"C:/Users/Matiss/Documents/MCU1/stm32l4xx_drivers/drivers/Inc" -I"C:/Users/Matiss/Documents/MCU1/drivers/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/spi_message_rcv_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

