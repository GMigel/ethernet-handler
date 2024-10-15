################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../die_with_error.c \
../server.c 

O_SRCS += \
../server.o 

C_DEPS += \
./die_with_error.d \
./server.d 

OBJS += \
./die_with_error.o \
./server.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./die_with_error.d ./die_with_error.o ./server.d ./server.o

.PHONY: clean--2e-

