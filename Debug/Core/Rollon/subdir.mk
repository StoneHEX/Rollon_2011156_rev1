################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Rollon/rollon.c \
../Core/Rollon/tcpServer.c 

OBJS += \
./Core/Rollon/rollon.o \
./Core/Rollon/tcpServer.o 

C_DEPS += \
./Core/Rollon/rollon.d \
./Core/Rollon/tcpServer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Rollon/%.o Core/Rollon/%.su Core/Rollon/%.cyclo: ../Core/Rollon/%.c Core/Rollon/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F745xx -c -I../Core/Inc -I../Core/Rollon -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/spi_eeprom -I../Drivers/BSP/Components/M95P32 -I../Drivers/BSP/Components/M95xx -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Rollon

clean-Core-2f-Rollon:
	-$(RM) ./Core/Rollon/rollon.cyclo ./Core/Rollon/rollon.d ./Core/Rollon/rollon.o ./Core/Rollon/rollon.su ./Core/Rollon/tcpServer.cyclo ./Core/Rollon/tcpServer.d ./Core/Rollon/tcpServer.o ./Core/Rollon/tcpServer.su

.PHONY: clean-Core-2f-Rollon

