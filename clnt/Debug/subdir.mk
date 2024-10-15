################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../client.c \
../die_with_error.c 

O_SRCS += \
../client.o \
../die_with_error.o 

C_DEPS += \
./client.d \
./die_with_error.d 

OBJS += \
./client.o \
./die_with_error.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./client.d ./client.o ./die_with_error.d ./die_with_error.o

.PHONY: clean--2e-

