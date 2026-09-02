################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
z3_system/%.obj: ../z3_system/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"D:/CCS_V21/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -O0 --opt_for_speed=2 --fp_mode=relaxed --fp_reassoc=on --include_path="D:/CCS_V21/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/include" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z7_data" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z8_control" --include_path="C:/Users/a/workspace_ccstheia/InverterBase" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/f2837xs/common/include" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/f2837xs/headers/include" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z3_system" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z4_scheduler" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z5_task" --include_path="C:/Users/a/workspace_ccstheia/InverterBase/z6_bsp" --include_path="D:/CCSV12/C2000Ware_26_01_00_00/libraries/math/CLAmath/c28/include" --define=CPU1 --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="z3_system/$(basename $(<F)).d_raw" --obj_directory="z3_system" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


