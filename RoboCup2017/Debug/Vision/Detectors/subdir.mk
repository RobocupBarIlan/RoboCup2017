################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Vision/Detectors/BallCandidateRansac.cpp \
../Vision/Detectors/BallDetector.cpp \
../Vision/Detectors/GoalCandidate.cpp \
../Vision/Detectors/GoalDetector.cpp \
../Vision/Detectors/GoalKeepersDetector.cpp 

OBJS += \
./Vision/Detectors/BallCandidateRansac.o \
./Vision/Detectors/BallDetector.o \
./Vision/Detectors/GoalCandidate.o \
./Vision/Detectors/GoalDetector.o \
./Vision/Detectors/GoalKeepersDetector.o 

CPP_DEPS += \
./Vision/Detectors/BallCandidateRansac.d \
./Vision/Detectors/BallDetector.d \
./Vision/Detectors/GoalCandidate.d \
./Vision/Detectors/GoalDetector.d \
./Vision/Detectors/GoalKeepersDetector.d 


# Each subdirectory must supply rules for building sources it contributes
Vision/Detectors/%.o: ../Vision/Detectors/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -fopenmp -I/usr/local/include -I/usr/local/include/opencv -I/home/robot/HROS5-Framework/Linux/include -I/home/robot/HROS5-Framework/Framework/include -I/home/robot/src/RobocupBIU/RoboCup2017/Include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


