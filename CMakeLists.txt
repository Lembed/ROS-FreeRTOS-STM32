cmake_minimum_required(VERSION 2.8)
project(FreeRTOS C CXX)


# the name of the target operating system
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm-eabi)

# Default toolchain path
set(TOOLCHAIN_PATH "/opt/CodeSourcery/arm-2011.09/bin/")

# which compilers to use for C and C++
set(CMAKE_C_COMPILER       ${TOOLCHAIN_PATH}arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER     ${TOOLCHAIN_PATH}arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER     ${TOOLCHAIN_PATH}arm-none-eabi-as)
set(CMAKE_OBJCOPY     	   ${TOOLCHAIN_PATH}arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP     	   ${TOOLCHAIN_PATH}arm-none-eabi-objdump)

set (PROJECT_SRC "${CMAKE_SOURCE_DIR}/src")

SET(CMAKE_C_FLAGS "-mthumb -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -std=gnu99  -nostartfiles " CACHE INTERNAL "c compiler flags" FORCE)
SET(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -std=gnu++0x -fabi-version=6 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -nostdinc++  -nostartfiles " CACHE INTERNAL "cxx compiler flags" FORCE)
SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m4" CACHE INTERNAL "asm compiler flags" FORCE)

SET(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -Xlinker --gc-sections -T${PROJECT_SRC}/ldscripts/libs.ld -T${PROJECT_SRC}/ldscripts/mem.ld -T${PROJECT_SRC}/ldscripts/sections.ld  -nostartfiles" CACHE INTERNAL "exe link flags")


# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment,
# search programs in the host environment
#
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)



include_directories(${PROJECT_SRC}/rcl/)
include_directories(${PROJECT_SRC}/os/main/include/)
include_directories(${PROJECT_SRC}/os/main/)
include_directories(${PROJECT_SRC}/rmw/)
include_directories(${PROJECT_SRC}/apps/)
include_directories(${PROJECT_SRC}/stm32/he/inc/)
include_directories(${PROJECT_SRC}/stm32/core/hwe/CPU/startup/)
include_directories(${PROJECT_SRC}/stm32/de/Ethernet/)
include_directories(${PROJECT_SRC}/stm32/de/net/)
include_directories(${PROJECT_SRC}/stm32/de/Shell/)
include_directories(${PROJECT_SRC}/stm32/he/STM32F4x7_ETH_Driver/inc/)
include_directories(${PROJECT_SRC}/os/kernel/include/)
include_directories(${PROJECT_SRC}/os/syslibs/math/Include/)
include_directories(${PROJECT_SRC}/os/kernel/portable/GCC/ARM_CM4F/)
include_directories(${PROJECT_SRC}/transport/lwip/src/include/)
include_directories(${PROJECT_SRC}/transport/lwip/src/include/ipv4/)
include_directories(${PROJECT_SRC}/transport/lwip/port/STM32F4x7//)
include_directories(${PROJECT_SRC}/transport/lwip/src/include/lwip/)
include_directories(${PROJECT_SRC}/transport/lwip/port/STM32F4x7/FreeRTOS/)
include_directories(${PROJECT_SRC}/transport/)
include_directories(${PROJECT_SRC}/apps/libs/)


# Linker flags
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")   # remove -rdynamic for C
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "") # remove -rdynamic for CXX

set(ELF_FILE "FreeRTOS.elf")
set(HEX_FILE "FreeRTOS.hex")
set(BIN_FILE "FreeRTOS.bin")

# get all *.cpp files recursively
file(GLOB_RECURSE CXX_FILES ${PROJECT_SRC}/*.cpp)
file(GLOB_RECURSE C_FILES ${PROJECT_SRC}/*.c)
add_executable(${ELF_FILE} ${CXX_FILES} ${C_FILES})


#add_custom_target(${HEX_FILE} DEPENDS ${ELF_FILE} COMMAND ${CMAKE_OBJCOPY} -Oihex ${ELF_FILE} ${HEX_FILE})
#add_custom_target(${BIN_FILE} DEPENDS ${ELF_FILE} COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE})



ADD_CUSTOM_COMMAND(TARGET ${ELF_FILE}
          POST_BUILD
			COMMAND ${CMAKE_OBJCOPY} -Oihex FreeRTOS.elf ${HEX_FILE})

ADD_CUSTOM_COMMAND(TARGET ${ELF_FILE}
          POST_BUILD
			COMMAND ${CMAKE_OBJCOPY} -Obinary FreeRTOS.elf ${BIN_FILE})

set(STLINK_CMD st-flash write ${BIN_FILE} 0x8000000)
add_custom_target(flash DEPENDS ${ELF_FILE} COMMAND ${STLINK_CMD})