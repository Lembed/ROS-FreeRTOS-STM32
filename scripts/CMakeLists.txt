cmake_minimum_required(VERSION 2.8)
project(eROS C CXX)


# the name of the target operating system
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm-eabi)

# Default toolchain path
#set(TOOLCHAIN_PATH "/opt/CodeSourcery/arm-2011.09/bin/")
set(TOOLCHAIN_PATH "")

# which compilers to use for C and C++
set(CMAKE_C_COMPILER       ${TOOLCHAIN_PATH}arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER     ${TOOLCHAIN_PATH}arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER     ${TOOLCHAIN_PATH}arm-none-eabi-as)
set(CMAKE_OBJCOPY     	   ${TOOLCHAIN_PATH}arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP     	   ${TOOLCHAIN_PATH}arm-none-eabi-objdump)

set (PROJECT_SRC "${CMAKE_SOURCE_DIR}/../src")

SET(CMAKE_C_FLAGS "-mthumb -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -std=gnu99  -nostartfiles " CACHE INTERNAL "c compiler flags" FORCE)
SET(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -std=gnu++0x -fabi-version=6 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -nostdinc++  -nostartfiles " CACHE INTERNAL "cxx compiler flags" FORCE)
SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m4" CACHE INTERNAL "asm compiler flags" FORCE)

SET(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -Xlinker --gc-sections -T${PROJECT_SRC}/bsp/ldscripts/libs.ld -T${PROJECT_SRC}/bsp/ldscripts/mem.ld -T${PROJECT_SRC}/bsp/ldscripts/sections.ld  -nostartfiles" CACHE INTERNAL "exe link flags")


# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment,
# search programs in the host environment
#
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


include_directories(${PROJECT_SRC})
include_directories(${PROJECT_SRC}/main/include/)
include_directories(${PROJECT_SRC}/main/)
include_directories(${PROJECT_SRC}/core/rcl/)
include_directories(${PROJECT_SRC}/core/rmw/)
include_directories(${PROJECT_SRC}/apps/)

include_directories(${PROJECT_SRC}/board/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/drivers/inc/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/CPU/startup/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/net/Ethernet/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/net/net/)
include_directories(${PROJECT_SRC}/board/Shell/)
include_directories(${PROJECT_SRC}/board/stm32f4xx/net/STM32F4x7_ETH_Driver/inc/)

include_directories(${PROJECT_SRC}/core/lwip-1.4.0/src/include/)
include_directories(${PROJECT_SRC}/core/lwip-1.4.0/src/include/ipv4/)
include_directories(${PROJECT_SRC}/core/lwip-1.4.0/port/STM32F4x7//)
include_directories(${PROJECT_SRC}/core/lwip-1.4.0/src/include/lwip/)
include_directories(${PROJECT_SRC}/core/lwip-1.4.0/port/STM32F4x7/FreeRTOS/)

include_directories(${PROJECT_SRC}/core/os/freertos-9.0.0/include/)
include_directories(${PROJECT_SRC}/core/os/freertos-9.0.0/utils/include/)
include_directories(${PROJECT_SRC}/core/os/freertos-9.0.0/portable/GCC/ARM_CM4F/)
include_directories(${PROJECT_SRC}/core/os/syslibs/math/Include/)
include_directories(${PROJECT_SRC}/device/)


# Linker flags
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")   # remove -rdynamic for C
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "") # remove -rdynamic for CXX

set(ELF_FILE "eROS.elf")
set(HEX_FILE "eROS.hex")
set(BIN_FILE "eROS.bin")

# get all *.cpp files recursively
file(GLOB_RECURSE CXX_FILES ${PROJECT_SRC}/*.cpp)
file(GLOB_RECURSE C_FILES ${PROJECT_SRC}/*.c)
# Adding header files is not required for build, it is only required for some IDEs to include headers in project navigator.
file(GLOB_RECURSE H_FILES ${PROJECT_SRC}/*.h)
add_executable(${ELF_FILE} ${CXX_FILES} ${C_FILES} ${H_FILES})

#add_custom_target(${HEX_FILE} DEPENDS ${ELF_FILE} COMMAND ${CMAKE_OBJCOPY} -Oihex ${ELF_FILE} ${HEX_FILE})
#add_custom_target(${BIN_FILE} DEPENDS ${ELF_FILE} COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE})

ADD_CUSTOM_COMMAND(TARGET ${ELF_FILE}
          POST_BUILD
			COMMAND ${CMAKE_OBJCOPY} -Oihex eROS.elf ${HEX_FILE})

ADD_CUSTOM_COMMAND(TARGET ${ELF_FILE}
          POST_BUILD
			COMMAND ${CMAKE_OBJCOPY} -Obinary eROS.elf ${BIN_FILE})

set(STLINK_CMD st-flash write ${BIN_FILE} 0x8000000)
add_custom_target(flash DEPENDS ${ELF_FILE} COMMAND ${STLINK_CMD})
