################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/TI_m0/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./main.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./Drivers/MPU6050/inv_mpu.o" \
"./Drivers/MPU6050/inv_mpu_dmp_motion_driver.o" \
"./Drivers/MPU6050/mpu6050.o" \
"./Drivers/MPU6050/mspm0_i2c.o" \
"./Drivers/MSPM0/clock.o" \
"./Drivers/MSPM0/control.o" \
"./Drivers/MSPM0/encoder.o" \
"./Drivers/MSPM0/gyro.o" \
"./Drivers/MSPM0/interrupt.o" \
"./Drivers/MSPM0/key.o" \
"./Drivers/MSPM0/lpf.o" \
"./Drivers/MSPM0/motor.o" \
"./Drivers/MSPM0/pid.o" \
"./Drivers/MSPM0/sensor.o" \
"./Drivers/MSPM0/sys.o" \
"./Drivers/OLED_Hardware_I2C/oled_hardware_i2c.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \
-Wl,-l"C:/TI/mspm0_sdk_2_10_00_04/source/ti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Drivers/MPU6050/subdir_vars.mk
-include Drivers/MSPM0/subdir_vars.mk
-include Drivers/OLED_Hardware_I2C/subdir_vars.mk
-include subdir_rules.mk
-include Drivers/MPU6050/subdir_rules.mk
-include Drivers/MSPM0/subdir_rules.mk
-include Drivers/OLED_Hardware_I2C/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
xunji_demo.out 

EXE_OUTPUTS__QUOTED += \
"xunji_demo.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "xunji_demo.out"

# Tool invocations
xunji_demo.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"D:/TI_m0/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -Wl,-m"xunji_demo.map" -Wl,-i"C:/TI/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/STM32hal/TI_project/24_H" -Wl,-i"D:/STM32hal/TI_project/24_H/Debug/syscfg" -Wl,-i"D:/TI_m0/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="xunji_demo_linkInfo.xml" -Wl,--rom_model -o "xunji_demo.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "Drivers\MPU6050\inv_mpu.o" "Drivers\MPU6050\inv_mpu_dmp_motion_driver.o" "Drivers\MPU6050\mpu6050.o" "Drivers\MPU6050\mspm0_i2c.o" "Drivers\MSPM0\clock.o" "Drivers\MSPM0\control.o" "Drivers\MSPM0\encoder.o" "Drivers\MSPM0\gyro.o" "Drivers\MSPM0\interrupt.o" "Drivers\MSPM0\key.o" "Drivers\MSPM0\lpf.o" "Drivers\MSPM0\motor.o" "Drivers\MSPM0\pid.o" "Drivers\MSPM0\sensor.o" "Drivers\MSPM0\sys.o" "Drivers\OLED_Hardware_I2C\oled_hardware_i2c.o" 
	-$(RM) "main.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "Drivers\MPU6050\inv_mpu.d" "Drivers\MPU6050\inv_mpu_dmp_motion_driver.d" "Drivers\MPU6050\mpu6050.d" "Drivers\MPU6050\mspm0_i2c.d" "Drivers\MSPM0\clock.d" "Drivers\MSPM0\control.d" "Drivers\MSPM0\encoder.d" "Drivers\MSPM0\gyro.d" "Drivers\MSPM0\interrupt.d" "Drivers\MSPM0\key.d" "Drivers\MSPM0\lpf.d" "Drivers\MSPM0\motor.d" "Drivers\MSPM0\pid.d" "Drivers\MSPM0\sensor.d" "Drivers\MSPM0\sys.d" "Drivers\OLED_Hardware_I2C\oled_hardware_i2c.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

