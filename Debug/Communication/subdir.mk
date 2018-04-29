################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Communication/CommunicationThread.cpp \
../Communication/SocketCommunication.cpp \
../Communication/UdpListener.cpp \
../Communication/UdpSender.cpp \
../Communication/UdpUtils.cpp 

OBJS += \
./Communication/CommunicationThread.o \
./Communication/SocketCommunication.o \
./Communication/UdpListener.o \
./Communication/UdpSender.o \
./Communication/UdpUtils.o 

CPP_DEPS += \
./Communication/CommunicationThread.d \
./Communication/SocketCommunication.d \
./Communication/UdpListener.d \
./Communication/UdpSender.d \
./Communication/UdpUtils.d 


# Each subdirectory must supply rules for building sources it contributes
Communication/%.o: ../Communication/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -fopenmp -I/usr/local/include -I/usr/local/include/opencv -I/home/robot/HROS5-Framework/Linux/include -I/home/robot/HROS5-Framework/Framework/include -I/home/robot/src/RobocupBIU/RoboCup2017/Include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


