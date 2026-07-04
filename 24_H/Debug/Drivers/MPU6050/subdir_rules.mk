################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Drivers/MPU6050/%.o: ../Drivers/MPU6050/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI_m0/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -I"D:/STM32hal/TI_project/24_H/Drivers/LSM6DSV16X" -I"D:/STM32hal/TI_project/24_H/Drivers/VL53L0X" -I"D:/STM32hal/TI_project/24_H/Drivers/WIT" -I"D:/STM32hal/TI_project/24_H/Drivers/BNO08X_UART_RVC" -I"D:/STM32hal/TI_project/24_H/Drivers/Ultrasonic_GPIO" -I"D:/STM32hal/TI_project/24_H/Drivers/Ultrasonic_Capture" -I"D:/STM32hal/TI_project/24_H/Drivers/OLED_Hardware_I2C" -I"D:/STM32hal/TI_project/24_H/Drivers/OLED_Hardware_SPI" -I"D:/STM32hal/TI_project/24_H/Drivers/OLED_Software_I2C" -I"D:/STM32hal/TI_project/24_H/Drivers/OLED_Software_SPI" -I"D:/STM32hal/TI_project/24_H/Drivers/MPU6050" -I"D:/STM32hal/TI_project/24_H" -I"D:/STM32hal/TI_project/24_H/Debug" -I"C:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/STM32hal/TI_project/24_H/Drivers/MSPM0" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"Drivers/MPU6050/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


