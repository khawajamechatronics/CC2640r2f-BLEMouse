################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Application/board_key.obj: ../Application/board_key.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_app" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Application" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Startup" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/PROFILES" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Include" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/heapmgr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/sdata" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=4 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_MAX_NUM_TASKS=8 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Application/board_key.d" --obj_directory="Application" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/gyro.obj: ../Application/gyro.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_app" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Application" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Startup" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/PROFILES" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Include" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/heapmgr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/sdata" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=4 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_MAX_NUM_TASKS=8 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Application/gyro.d" --obj_directory="Application" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/hidemukbd.obj: ../Application/hidemukbd.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_app" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Application" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Startup" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/PROFILES" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Include" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/heapmgr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/sdata" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=4 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_MAX_NUM_TASKS=8 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Application/hidemukbd.d" --obj_directory="Application" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/spi.obj: ../Application/spi.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_app" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Application" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Startup" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/PROFILES" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Include" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/heapmgr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/sdata" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=4 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_MAX_NUM_TASKS=8 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Application/spi.d" --obj_directory="Application" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/util.obj: ../Application/util.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_app" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Application" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Startup" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/PROFILES" --include_path="E:/ccs_work/ble5_hid_gyromouse_app/Include" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/heapmgr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/sdata" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=4 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=ICALL_MAX_NUM_ENTITIES=8 --define=ICALL_MAX_NUM_TASKS=8 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Application/util.d" --obj_directory="Application" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


