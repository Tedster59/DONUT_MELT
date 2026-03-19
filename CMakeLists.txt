cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(donut_melt C CXX ASM)

pico_sdk_init()

# add_executable(donut_melt 
#     src/donut_meltier.c 
#     src/led_driver.c 
#     src/receiver.c 
#     src/motor_driver.c 
#     src/donut_drive.c 
#     src/donut_telemetry.c
# )

#---------------TESTS---------------

# add_executable(donut_melt tests/just_blink_test.c src/led_driver.c)
#add_executable(donut_melt tests/donut_blink_test.c src/led_driver.c)

 add_executable(donut_melt tests/dshot_ramp_test.c src/motor_driver.c)
# add_executable(donut_melt tests/dshot_from_tx_test.c src/motor_driver.c src/led_driver.c src/receiver.c)

#---------------END TESTS---------------


target_include_directories(donut_melt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(lib/c_pico_bidir_dshot)
add_subdirectory(lib/pico_crsf)
add_subdirectory(lib/pico_H3LIS331DL)

pico_enable_stdio_uart(donut_melt 0)
pico_enable_stdio_usb(donut_melt 1)

target_link_libraries(donut_melt 
    pico_stdlib
    pico_stdio

    c_pico_bidir_dshot
    pico_crsf
    pico_H3LIS331DL

    hardware_pio
    hardware_uart
    hardware_watchdog
    pico_time
)

pico_add_extra_outputs(donut_melt)