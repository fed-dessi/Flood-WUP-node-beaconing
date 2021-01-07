################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/SiliconLabs/subdir.mk
-include gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/MemMang/subdir.mk
-include gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/GCC/ARM_CM4F/subdir.mk
-include gecko_sdk_3.0.0/util/third_party/freertos/Source/subdir.mk
-include gecko_sdk_3.0.0/platform/service/udelay/src/subdir.mk
-include gecko_sdk_3.0.0/platform/service/system/src/subdir.mk
-include gecko_sdk_3.0.0/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_3.0.0/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_3.0.0/platform/service/mpu/src/subdir.mk
-include gecko_sdk_3.0.0/platform/service/device_init/src/subdir.mk
-include gecko_sdk_3.0.0/platform/radio/rail_lib/plugin/rail_util_init/subdir.mk
-include gecko_sdk_3.0.0/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include gecko_sdk_3.0.0/platform/emlib/src/subdir.mk
-include gecko_sdk_3.0.0/platform/emdrv/uartdrv/src/subdir.mk
-include gecko_sdk_3.0.0/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_3.0.0/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_3.0.0/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_3.0.0/platform/common/src/subdir.mk
-include gecko_sdk_3.0.0/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/subdir.mk
-include gecko_sdk_3.0.0/platform/Device/SiliconLabs/EFR32MG12P/Source/subdir.mk
-include gecko_sdk_3.0.0/hardware/driver/mx25_flash_shutdown/src/subdir.mk
-include gecko_sdk_3.0.0/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: flood\ wup\ node\ beaconing.axf

# Tool invocations
flood\ wup\ node\ beaconing.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node beaconing/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="flood wup node beaconing.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o "flood wup node beaconing.axf" -Wl,--start-group "./app_init.o" "./app_process.o" "./main.o" "./autogen/rail_config.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_rail_util_callbacks.o" "./autogen/sl_uartdrv_init.o" "./gecko_sdk_3.0.0/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_3.0.0/hardware/board/src/sl_board_init.o" "./gecko_sdk_3.0.0/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown.o" "./gecko_sdk_3.0.0/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/startup_efr32mg12p.o" "./gecko_sdk_3.0.0/platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o" "./gecko_sdk_3.0.0/platform/common/src/sl_slist.o" "./gecko_sdk_3.0.0/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_3.0.0/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_3.0.0/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_3.0.0/platform/emdrv/uartdrv/src/uartdrv.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_assert.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_cmu.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_core.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_emu.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_gpio.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_ldma.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_leuart.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_rtcc.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_system.o" "./gecko_sdk_3.0.0/platform/emlib/src/em_usart.o" "./gecko_sdk_3.0.0/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./gecko_sdk_3.0.0/platform/radio/rail_lib/plugin/pa-conversions/pa_curves_efr32.o" "./gecko_sdk_3.0.0/platform/radio/rail_lib/plugin/rail_util_init/sl_rail_util_init.o" "./gecko_sdk_3.0.0/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_3.0.0/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_3.0.0/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_3.0.0/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_3.0.0/platform/service/mpu/src/sl_mpu.o" "./gecko_sdk_3.0.0/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_3.0.0/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_3.0.0/platform/service/power_manager/src/sl_power_manager_hal_s0_s1.o" "./gecko_sdk_3.0.0/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_3.0.0/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_3.0.0/platform/service/system/src/sl_system_init.o" "./gecko_sdk_3.0.0/platform/service/system/src/sl_system_kernel.o" "./gecko_sdk_3.0.0/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_3.0.0/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/croutine.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/event_groups.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/list.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/queue.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/tasks.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/timers.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/GCC/ARM_CM4F/port.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/MemMang/heap_4.o" "./gecko_sdk_3.0.0/util/third_party/freertos/Source/portable/SiliconLabs/tick_power_manager.o" "D:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.0//platform/radio/rail_lib/autogen/librail_release/librail_efr32xg12_gcc_release.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: flood wup node beaconing.hex'
	arm-none-eabi-objcopy -O ihex "flood wup node beaconing.axf" "flood wup node beaconing.hex"
	@echo ' '

	@echo 'Building bin file: flood wup node beaconing.bin'
	arm-none-eabi-objcopy -O binary "flood wup node beaconing.axf" "flood wup node beaconing.bin"
	@echo ' '

	@echo 'Building s37 file: flood wup node beaconing.s37'
	arm-none-eabi-objcopy -O srec "flood wup node beaconing.axf" "flood wup node beaconing.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "flood wup node beaconing.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) "flood wup node beaconing.axf"
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
