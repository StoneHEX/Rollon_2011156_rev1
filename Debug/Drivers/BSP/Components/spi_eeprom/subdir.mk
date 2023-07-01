################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/spi_eeprom/spi_eeprom.c 

OBJS += \
./Drivers/BSP/Components/spi_eeprom/spi_eeprom.o 

C_DEPS += \
./Drivers/BSP/Components/spi_eeprom/spi_eeprom.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/spi_eeprom/%.o Drivers/BSP/Components/spi_eeprom/%.su Drivers/BSP/Components/spi_eeprom/%.cyclo: ../Drivers/BSP/Components/spi_eeprom/%.c Drivers/BSP/Components/spi_eeprom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F745xx -c -I../Core/Inc -I../Core/Rollon -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/spi_eeprom -I../Drivers/BSP/Components/M95P32 -I../Drivers/BSP/Components/M95xx -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-spi_eeprom

clean-Drivers-2f-BSP-2f-Components-2f-spi_eeprom:
	-$(RM) ./Drivers/BSP/Components/spi_eeprom/spi_eeprom.cyclo ./Drivers/BSP/Components/spi_eeprom/spi_eeprom.d ./Drivers/BSP/Components/spi_eeprom/spi_eeprom.o ./Drivers/BSP/Components/spi_eeprom/spi_eeprom.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-spi_eeprom

