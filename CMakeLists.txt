cmake_minimum_required(VERSION 3.2)
project(RoboMaster_MainControl)

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_VERBOSE_MAKEFILE off)

set(tools /usr/local/gcc-arm-none-eabi)
set(CROSS_COMPILER_PREFIX arm-none-eabi-)

include (CMakeForceCompiler)
cmake_force_c_compiler(${tools}/bin/${CROSS_COMPILER_PREFIX}gcc GNU)
cmake_force_cxx_compiler(${tools}/bin/${CROSS_COMPILER_PREFIX}g++ GNU)


set(SOURCE_FILES
    include/mylib/12864.h
    include/mylib/ad.h
    include/mylib/buzzer.h
    include/mylib/can1.h
    include/mylib/can2.h
    include/mylib/delay.h
    include/mylib/driver.h
    include/mylib/encoder.h
    include/mylib/exti.h
    include/mylib/gyroscope.h
    include/mylib/hall.h
    include/mylib/key.h
    include/mylib/led.h
    include/mylib/oled.h
    include/mylib/pwm.h
    include/mylib/receiver.h
    include/mylib/relay.h
    include/mylib/sensor.h
    include/mylib/tim6.h
    include/mylib/usart3.h
    include/user/app.h
    include/user/debug.h
    include/user/main.h
    include/user/stm32f4xx_conf.h
    include/user/stm32f4xx_it.h
    src/mylib/12864.c
    src/mylib/ad.c
    src/mylib/buzzer.c
    src/mylib/can1.c
    src/mylib/can2.c
    src/mylib/delay.c
    src/mylib/driver.c
    src/mylib/encoder.c
    src/mylib/exti.c
    src/mylib/gyroscope.c
    src/mylib/hall.c
    src/mylib/key.c
    src/mylib/led.c
    src/mylib/oled.c
    src/mylib/pwm.c
    src/mylib/receiver.c
    src/mylib/relay.c
    src/mylib/sensor.c
    src/mylib/tim6.c
    src/mylib/usart3.c
    src/user/app.c
    src/user/debug.c
    src/user/main.c
    src/user/stm32f4xx_it.c
    system/include/arm/semihosting.h
    system/include/cmsis/arm_common_tables.h
    system/include/cmsis/arm_const_structs.h
    system/include/cmsis/arm_math.h
    system/include/cmsis/cmsis_device.h
    system/include/cmsis/core_cm0.h
    system/include/cmsis/core_cm0plus.h
    system/include/cmsis/core_cm3.h
    system/include/cmsis/core_cm4.h
    system/include/cmsis/core_cm4_simd.h
    system/include/cmsis/core_cmFunc.h
    system/include/cmsis/core_cmInstr.h
    system/include/cmsis/core_sc000.h
    system/include/cmsis/core_sc300.h
    system/include/cmsis/system_stm32f4xx.h
    system/include/cortexm/ExceptionHandlers.h
    system/include/diag/Trace.h
    system/include/stm32f4-stdperiph/misc.h
    system/include/stm32f4-stdperiph/stm32f4xx.h
    system/include/stm32f4-stdperiph/stm32f4xx_adc.h
    system/include/stm32f4-stdperiph/stm32f4xx_can.h
    system/include/stm32f4-stdperiph/stm32f4xx_crc.h
    system/include/stm32f4-stdperiph/stm32f4xx_cryp.h
    system/include/stm32f4-stdperiph/stm32f4xx_dac.h
    system/include/stm32f4-stdperiph/stm32f4xx_dbgmcu.h
    system/include/stm32f4-stdperiph/stm32f4xx_dcmi.h
    system/include/stm32f4-stdperiph/stm32f4xx_dma.h
    system/include/stm32f4-stdperiph/stm32f4xx_exti.h
    system/include/stm32f4-stdperiph/stm32f4xx_flash.h
    system/include/stm32f4-stdperiph/stm32f4xx_fsmc.h
    system/include/stm32f4-stdperiph/stm32f4xx_gpio.h
    system/include/stm32f4-stdperiph/stm32f4xx_hash.h
    system/include/stm32f4-stdperiph/stm32f4xx_i2c.h
    system/include/stm32f4-stdperiph/stm32f4xx_iwdg.h
    system/include/stm32f4-stdperiph/stm32f4xx_pwr.h
    system/include/stm32f4-stdperiph/stm32f4xx_rcc.h
    system/include/stm32f4-stdperiph/stm32f4xx_rng.h
    system/include/stm32f4-stdperiph/stm32f4xx_rtc.h
    system/include/stm32f4-stdperiph/stm32f4xx_sdio.h
    system/include/stm32f4-stdperiph/stm32f4xx_spi.h
    system/include/stm32f4-stdperiph/stm32f4xx_syscfg.h
    system/include/stm32f4-stdperiph/stm32f4xx_tim.h
    system/include/stm32f4-stdperiph/stm32f4xx_usart.h
    system/include/stm32f4-stdperiph/stm32f4xx_wwdg.h
    system/src/cmsis/stack_heap_stm32f4xx.c
    system/src/cmsis/system_stm32f4xx.c
    system/src/cmsis/vectors_stm32f4xx.c
    system/src/cortexm/_initialize_hardware.c
    system/src/cortexm/_reset_hardware.c
    system/src/cortexm/exception_handlers.c
    system/src/diag/Trace.c
    system/src/diag/trace_impl.c
    system/src/newlib/_cxx.cpp
    system/src/newlib/_exit.c
    system/src/newlib/_sbrk.c
    system/src/newlib/_startup.c
    system/src/newlib/_syscalls.c
    system/src/newlib/assert.c
    system/src/stm32f4-stdperiph/misc.c
    system/src/stm32f4-stdperiph/stm32f4xx_adc.c
    system/src/stm32f4-stdperiph/stm32f4xx_can.c
    system/src/stm32f4-stdperiph/stm32f4xx_crc.c
    system/src/stm32f4-stdperiph/stm32f4xx_cryp.c
    system/src/stm32f4-stdperiph/stm32f4xx_cryp_aes.c
    system/src/stm32f4-stdperiph/stm32f4xx_cryp_des.c
    system/src/stm32f4-stdperiph/stm32f4xx_cryp_tdes.c
    system/src/stm32f4-stdperiph/stm32f4xx_dac.c
    system/src/stm32f4-stdperiph/stm32f4xx_dbgmcu.c
    system/src/stm32f4-stdperiph/stm32f4xx_dcmi.c
    system/src/stm32f4-stdperiph/stm32f4xx_dma.c
    system/src/stm32f4-stdperiph/stm32f4xx_exti.c
    system/src/stm32f4-stdperiph/stm32f4xx_flash.c
    system/src/stm32f4-stdperiph/stm32f4xx_fsmc.c
    system/src/stm32f4-stdperiph/stm32f4xx_gpio.c
    system/src/stm32f4-stdperiph/stm32f4xx_hash.c
    system/src/stm32f4-stdperiph/stm32f4xx_hash_md5.c
    system/src/stm32f4-stdperiph/stm32f4xx_hash_sha1.c
    system/src/stm32f4-stdperiph/stm32f4xx_i2c.c
    system/src/stm32f4-stdperiph/stm32f4xx_iwdg.c
    system/src/stm32f4-stdperiph/stm32f4xx_pwr.c
    system/src/stm32f4-stdperiph/stm32f4xx_rcc.c
    system/src/stm32f4-stdperiph/stm32f4xx_rng.c
    system/src/stm32f4-stdperiph/stm32f4xx_rtc.c
    system/src/stm32f4-stdperiph/stm32f4xx_sdio.c
    system/src/stm32f4-stdperiph/stm32f4xx_spi.c
    system/src/stm32f4-stdperiph/stm32f4xx_syscfg.c
    system/src/stm32f4-stdperiph/stm32f4xx_tim.c
    system/src/stm32f4-stdperiph/stm32f4xx_usart.c
    system/src/stm32f4-stdperiph/stm32f4xx_wwdg.c)

include_directories(include)
include_directories(include/mylib)
include_directories(include/user)
include_directories(system/include)
include_directories(system/include/cmsis)
include_directories(system/include/stm32f4-stdperiph)

add_executable(RoboMaster_MainControl ${SOURCE_FILES})

set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS
    HSE_VALUE=25000000
    STM32F405xx
    USE_STDPERIPH_DRIVER
    STDOUT_USART2
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Debug>:TRACE>
    $<$<CONFIG:Debug>:OS_USE_TRACE_SEMIHOSTING_DEBUG>
)

set(OPT_LEVEL "s")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -O${OPT_LEVEL} -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -std=gnu11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m4 -mthumb -O${OPT_LEVEL} -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -std=gnu++11 -fabi-version=0")

set(LINKER_SCRIPTS_DIR ${PROJECT_SOURCE_DIR}/Eclipse/ldscripts)
set(MEM_LINKER_SCRIPT ${LINKER_SCRIPTS_DIR}/mem.ld)
set(SECTIONS_LINKER_SCRIPT ${LINKER_SCRIPTS_DIR}/sections.ld)

set(BUILD_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE}")

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${BUILD_OUTPUT_PATH}")


set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.elf")
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-T ${MEM_LINKER_SCRIPT} -T ${SECTIONS_LINKER_SCRIPT} -nostartfiles -Xlinker --gc-sections -Wl,-Map,\"${BUILD_OUTPUT_PATH}/${PROJECT_NAME}.map\" --specs=nano.specs")

add_custom_command(TARGET ${PROJECT_NAME} COMMAND ${tools}/bin/${CROSS_COMPILER_PREFIX}objcopy -O ihex \"${BUILD_OUTPUT_PATH}/${PROJECT_NAME}.elf\" \"${BUILD_OUTPUT_PATH}/${PROJECT_NAME}.hex\")
add_custom_command(TARGET ${PROJECT_NAME} COMMAND ${tools}/bin/${CROSS_COMPILER_PREFIX}size --format=berkeley \"${BUILD_OUTPUT_PATH}/${PROJECT_NAME}.elf\")
