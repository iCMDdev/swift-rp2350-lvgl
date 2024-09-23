cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(hacky_cmake_helper.cmake)

project(swift-rp2350-lvgl-st7789)
pico_sdk_init()

if(APPLE)
execute_process(COMMAND xcrun -f swiftc OUTPUT_VARIABLE SWIFTC OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
execute_process(COMMAND which swiftc OUTPUT_VARIABLE SWIFTC OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()


set(SWIFT_TARGET "armv6m-none-none-eabi") # default for rp2040
list(APPEND CLANG_ARCH_ABI_FLAGS "-Xcc") 

if(PICO_PLATFORM STREQUAL "rp2350-arm-s")
    message(STATUS "PICO_PLATFORM is set to rp2350-arm-s, using armv7em")
    set(SWIFT_TARGET "armv7em-none-none-eabi")
    list(APPEND CLANG_ARCH_ABI_FLAGS "-mfloat-abi=soft")
elseif(PICO_PLATFORM STREQUAL "rp2040")
    message(STATUS "PICO_PLATFORM is set to RP2040, using armv6m")
    list(APPEND CLANG_ARCH_ABI_FLAGS "-mfloat-abi=soft")
elseif(PICO_PLATFORM STREQUAL "rp2350-riscv")
    message(STATUS "PICO_PLATFORM is set to rp2350-riscv, using riscv32.")
    set(SWIFT_TARGET "riscv32-none-none-eabi")
    list(APPEND CLANG_ARCH_ABI_FLAGS "-march=rv32imac_zicsr_zifencei_zba_zbb_zbs_zbkb" "-Xcc" "-mabi=ilp32")
endif()

gather_vars(COMPILE_DEFINITIONS INTERFACE_COMPILE_DEFINITIONS _touched_cd swift-rp2350-lvgl-st7789)
list(REMOVE_DUPLICATES COMPILE_DEFINITIONS)
list(PREPEND COMPILE_DEFINITIONS "") # -Xcc -D
string(REPLACE "$<TARGET_PROPERTY:PICO_TARGET_BINARY_TYPE>" "$<TARGET_PROPERTY:swift-rp2350-lvgl-st7789,PICO_TARGET_BINARY_TYPE>" COMPILE_DEFINITIONS "${COMPILE_DEFINITIONS}")
string(REPLACE ";" " -Xcc -D" COMPILE_DEFINITIONS "${COMPILE_DEFINITIONS}")

file(GENERATE OUTPUT ${CMAKE_BINARY_DIR}/swiftc_flags.txt CONTENT "${COMPILE_DEFINITIONS}")

add_executable(swift-rp2350-lvgl-st7789 hstx_spi_lcd.c)
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/_swiftcode.o
    COMMAND
        ${SWIFTC}
        -target ${SWIFT_TARGET} -Xcc -fshort-enums
        @${CMAKE_BINARY_DIR}/swiftc_flags.txt
        ${CLANG_ARCH_ABI_FLAGS}
        -Xfrontend -function-sections -enable-experimental-feature Embedded -wmo -parse-as-library
        $$\( echo '$<TARGET_PROPERTY:swift-blinky,INCLUDE_DIRECTORIES>' | tr '\;' '\\n' | sed -e 's/\\\(.*\\\)/-Xcc -I\\1/g' \)
        $$\( echo '${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES}'             | tr ' '  '\\n' | sed -e 's/\\\(.*\\\)/-Xcc -I\\1/g' \)
        -import-bridging-header ${CMAKE_CURRENT_LIST_DIR}/BridgingHeader.h
        ${CMAKE_CURRENT_LIST_DIR}/Main.swift
        -c -o ${CMAKE_CURRENT_BINARY_DIR}/_swiftcode.o
    DEPENDS
        ${CMAKE_CURRENT_LIST_DIR}/BridgingHeader.h
        ${CMAKE_CURRENT_LIST_DIR}/Main.swift
)
add_custom_target(swift-rp2350-lvgl-st7789-swiftcode DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/_swiftcode.o)


# Specify path to own LVGL config header
set(LV_CONF_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h
    CACHE STRING "" FORCE)
    
# Fetch LVGL from GitHub
FetchContent_Declare(lvgl GIT_REPOSITORY https://github.com/lvgl/lvgl.git)
FetchContent_MakeAvailable(lvgl)

target_link_libraries(swift-rp2350-lvgl-st7789
    pico_stdlib hardware_uart hardware_gpio hardware_spi hardware_gpio hardware_pwm hardware_clocks hardware_pio
    hardware_resets pico_sync
    ${CMAKE_CURRENT_BINARY_DIR}/_swiftcode.o
    lvgl::lvgl
    lvgl::demos
)
add_dependencies(swift-rp2350-lvgl-st7789 swift-rp2350-lvgl-st7789-swiftcode)



pico_add_extra_outputs(swift-rp2350-lvgl-st7789)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
