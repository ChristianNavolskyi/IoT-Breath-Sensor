################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI/smartrf_settings" --include_path="C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc13xxware_2_04_03_17272" --include_path="C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-844547167:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-844547167-inproc

build-844547167-inproc: ../adcBufContinuousSampling.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_32_00_06_core/xs" --xdcpath="C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/packages;C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/tidrivers_cc13xx_cc26xx_2_21_00_04/packages;C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/bios_6_46_01_37/packages;C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/uia_2_01_00_01/packages;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC1350F128 -r release -c "C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS" --compileOptions "-mv7M3 --code_state=16 --float_support=vfplib -me --include_path=\"C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI\" --include_path=\"C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI\" --include_path=\"C:/Users/user/workspace_v9/rfPacketTx_CC1350_LAUNCHXL_TI/smartrf_settings\" --include_path=\"C:/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc13xxware_2_04_03_17272\" --include_path=\"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include\" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi  " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-844547167 ../adcBufContinuousSampling.cfg
configPkg/compiler.opt: build-844547167
configPkg/: build-844547167


