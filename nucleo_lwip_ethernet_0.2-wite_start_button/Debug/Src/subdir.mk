################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/can.c \
../Src/ethernetif.c \
../Src/lwip.c \
../Src/main.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_it.c \
../Src/system_stm32f7xx.c 

OBJS += \
./Src/can.o \
./Src/ethernetif.o \
./Src/lwip.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_it.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/can.d \
./Src/ethernetif.d \
./Src/lwip.d \
./Src/main.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_it.d \
./Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Inc" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/system" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Users/Eli/Documents/Stm workspace/nucleo_lwip_ethernet_0.2-wite_start_button/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


