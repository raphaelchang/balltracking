################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BallTracking.cpp \
../BlobExtractor.cpp \
../DisjointSet.cpp \
../RANSAC.cpp \
../Serial.cpp \
../main.cpp 

OBJS += \
./BallTracking.o \
./BlobExtractor.o \
./DisjointSet.o \
./RANSAC.o \
./Serial.o \
./main.o 

CPP_DEPS += \
./BallTracking.d \
./BlobExtractor.d \
./DisjointSet.d \
./RANSAC.d \
./Serial.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/raphael/OpenNI-Linux-x64-2.2/Include -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


