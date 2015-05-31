################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f2-stdperiph/misc.c \
../system/src/stm32f2-stdperiph/stm32f2xx_exti.c \
../system/src/stm32f2-stdperiph/stm32f2xx_gpio.c \
../system/src/stm32f2-stdperiph/stm32f2xx_i2c.c \
../system/src/stm32f2-stdperiph/stm32f2xx_pwr.c \
../system/src/stm32f2-stdperiph/stm32f2xx_rcc.c \
../system/src/stm32f2-stdperiph/stm32f2xx_rtc.c \
../system/src/stm32f2-stdperiph/stm32f2xx_syscfg.c \
../system/src/stm32f2-stdperiph/stm32f2xx_usart.c 

OBJS += \
./system/src/stm32f2-stdperiph/misc.o \
./system/src/stm32f2-stdperiph/stm32f2xx_exti.o \
./system/src/stm32f2-stdperiph/stm32f2xx_gpio.o \
./system/src/stm32f2-stdperiph/stm32f2xx_i2c.o \
./system/src/stm32f2-stdperiph/stm32f2xx_pwr.o \
./system/src/stm32f2-stdperiph/stm32f2xx_rcc.o \
./system/src/stm32f2-stdperiph/stm32f2xx_rtc.o \
./system/src/stm32f2-stdperiph/stm32f2xx_syscfg.o \
./system/src/stm32f2-stdperiph/stm32f2xx_usart.o 

C_DEPS += \
./system/src/stm32f2-stdperiph/misc.d \
./system/src/stm32f2-stdperiph/stm32f2xx_exti.d \
./system/src/stm32f2-stdperiph/stm32f2xx_gpio.d \
./system/src/stm32f2-stdperiph/stm32f2xx_i2c.d \
./system/src/stm32f2-stdperiph/stm32f2xx_pwr.d \
./system/src/stm32f2-stdperiph/stm32f2xx_rcc.d \
./system/src/stm32f2-stdperiph/stm32f2xx_rtc.d \
./system/src/stm32f2-stdperiph/stm32f2xx_syscfg.d \
./system/src/stm32f2-stdperiph/stm32f2xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f2-stdperiph/%.o: ../system/src/stm32f2-stdperiph/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Werror -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Wfloat-equal  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F2XX -DUSE_STDPERIPH_DRIVER -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f2-stdperiph" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


