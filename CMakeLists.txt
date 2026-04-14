# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pokemon-spi-sd C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Fetch the SD card library
include(FetchContent)
FetchContent_Declare(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    GIT_REPOSITORY https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico.git
    GIT_TAG main
    SOURCE_SUBDIR src
)
FetchContent_MakeAvailable(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico)

add_executable(pokemon-spi-sd pokemon-spi-sd.c hw_config.c)

pico_generate_pio_header(pokemon-spi-sd ${CMAKE_CURRENT_LIST_DIR}/i2s_out.pio)

pico_set_program_name(pokemon-spi-sd "pokemon-spi-sd")
pico_set_program_version(pokemon-spi-sd "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pokemon-spi-sd 0)
pico_enable_stdio_usb(pokemon-spi-sd 1)

# Add the standard library to the build
target_link_libraries(pokemon-spi-sd
        pico_stdlib hardware_pio hardware_clocks hardware_spi hardware_dma hardware_irq pico_util pico_multicore no-OS-FatFS-SD-SDIO-SPI-RPi-Pico)

# Add the standard include files to the build
target_include_directories(pokemon-spi-sd PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${no-os-fatfs-sd-spi-rpi-pico_SOURCE_DIR}/src
        ${no-os-fatfs-sd-spi-rpi-pico_SOURCE_DIR}/FatFs_SPI/sd_driver
)

pico_add_extra_outputs(pokemon-spi-sd)

