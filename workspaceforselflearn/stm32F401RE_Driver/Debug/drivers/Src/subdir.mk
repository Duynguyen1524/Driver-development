################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/gpio_stm32f4xx_driver.c \
../drivers/Src/i2c_stm32f4xx_driver.c \
../drivers/Src/spi_stm32f4xx_driver.c 

OBJS += \
./drivers/Src/gpio_stm32f4xx_driver.o \
./drivers/Src/i2c_stm32f4xx_driver.o \
./drivers/Src/spi_stm32f4xx_driver.o 

C_DEPS += \
./drivers/Src/gpio_stm32f4xx_driver.d \
./drivers/Src/i2c_stm32f4xx_driver.d \
./drivers/Src/spi_stm32f4xx_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/Users/thanh/OneDrive/DREXEL_STUDY/workspaceforselflearn/stm32F401RE_Driver/drivers/Inc" -I"C:/Users/thanh/OneDrive/DREXEL_STUDY/workspaceforselflearn/stm32F401RE_Driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/gpio_stm32f4xx_driver.cyclo ./drivers/Src/gpio_stm32f4xx_driver.d ./drivers/Src/gpio_stm32f4xx_driver.o ./drivers/Src/gpio_stm32f4xx_driver.su ./drivers/Src/i2c_stm32f4xx_driver.cyclo ./drivers/Src/i2c_stm32f4xx_driver.d ./drivers/Src/i2c_stm32f4xx_driver.o ./drivers/Src/i2c_stm32f4xx_driver.su ./drivers/Src/spi_stm32f4xx_driver.cyclo ./drivers/Src/spi_stm32f4xx_driver.d ./drivers/Src/spi_stm32f4xx_driver.o ./drivers/Src/spi_stm32f4xx_driver.su

.PHONY: clean-drivers-2f-Src

