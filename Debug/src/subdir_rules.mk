################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/%.obj: ../src/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-msp430_18.1.4.LTS/bin/cl430" -vmspx --data_model=restricted --use_hw_mpy=F5 --include_path="C:/ti/ccsv8/ccs_base/msp430/include" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/Communication" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/Control" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/DataBase" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/driverlib" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/GCS" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/HAL" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/HardWare" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/MATH" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/MidWare" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01/src" --include_path="C:/Users/Cal/Documents/files/CCS_workSpace_planeWithNoBody_01/feiKong_01" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-msp430_18.1.4.LTS/include" --advice:power="all" --define=__MSP430F5529__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="src/$(basename $(<F)).d_raw" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


