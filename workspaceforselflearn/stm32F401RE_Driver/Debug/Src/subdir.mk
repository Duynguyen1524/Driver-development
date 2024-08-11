################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006spi_tx_testing.c 

OBJS += \
./Src/006spi_tx_testing.o 

C_DEPS += \
./Src/006spi_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/Users/thanh/OneDrive/DREXEL_STUDY/workspaceforselflearn/stm32F401RE_Driver/drivers/Inc" -I"C:/Users/thanh/OneDrive/DREXEL_STUDY/workspaceforselflearn/stm32F401RE_Driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/006spi_tx_testing.cyclo ./Src/006spi_tx_testing.d ./Src/006spi_tx_testing.o ./Src/006spi_tx_testing.su

.PHONY: clean-Src

