################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Brain/BrainThread.cpp 

OBJS += \
./Brain/BrainThread.o 

CPP_DEPS += \
./Brain/BrainThread.d 


# Each subdirectory must supply rules for building sources it contributes
Brain/%.o: ../Brain/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -fopenmp -I/usr/local/include -I/usr/local/include/opencv -I/home/robot/HROS5-Framework/Linux/include -I/home/robot/HROS5-Framework/Framework/include -I/home/robot/src/RobocupBIU/RoboCup2017/Include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


